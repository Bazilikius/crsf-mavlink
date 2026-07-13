import serial
import serial.tools.list_ports
import struct
import threading
import time
import socket

# --- PC Mux Protocol Parser ---
SYNC1 = 0xAA
SYNC2 = 0x55

CHAN_CRSF = 0x01
CHAN_MAVLINK = 0x02
CHAN_CONFIG = 0x03

class MuxParser:
    def __init__(self):
        self.state = 0
        self.chan_id = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0

    def parse_byte(self, b):
        if self.state == 0:
            if b == SYNC1:
                self.state = 1
        elif self.state == 1:
            if b == SYNC2:
                self.state = 2
            elif b == SYNC1:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 2:
            if b in [CHAN_CRSF, CHAN_MAVLINK, CHAN_CONFIG]:
                self.chan_id = b
                self.state = 3
            elif b == SYNC1:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 3:
            self.length = b
            self.payload = bytearray()
            if b == 0:
                self.state = 5
            else:
                self.state = 4
        elif self.state == 4:
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.state = 5
        elif self.state == 5:
            self.checksum = b
            self.state = 0
            calc = (self.chan_id + len(self.payload)) & 0xFF
            for x in self.payload:
                calc = (calc + x) & 0xFF
            if calc == self.checksum:
                return True, self.chan_id, bytes(self.payload)
        return False, 0, b""

def mux_encode(chan_id, payload):
    out = bytearray([SYNC1, SYNC2, chan_id, len(payload)])
    out.extend(payload)
    cksum = (chan_id + len(payload)) & 0xFF
    for b in payload:
        cksum = (cksum + b) & 0xFF
    out.append(cksum)
    return bytes(out)


class SerialConnection:
    def __init__(self, on_config_received_cb=None, log_message_cb=None):
        self.ser = None
        self.read_thread = None
        self.running = False
        self.on_config_received_cb = on_config_received_cb
        self.log_message_cb = log_message_cb

        # MAVLink UDP Proxy Settings
        self.udp_sock = None
        self.udp_thread = None
        self.udp_client_addr = None
        self.udp_port = 14550 # Standard Mission Planner / QGC UDP Port

        # Parse state
        self.usb_mux_parser = MuxParser()

    @staticmethod
    def list_ports():
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def log(self, msg):
        if self.log_message_cb:
            self.log_message_cb(msg)
        else:
            print(msg)

    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.running = True

            # Start Background Read Thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            # Try to bind the MAVLink UDP Proxy Socket
            try:
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock.bind(('127.0.0.1', self.udp_port))
                self.udp_sock.settimeout(0.1)

                # Start Background UDP Thread if bind succeeded
                self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
                self.udp_thread.start()
                self.log(f"MAVLink UDP proxy server successfully started on port {self.udp_port}.")
            except OSError as e:
                self.udp_sock = None
                self.udp_thread = None
                self.log(f"Warning: Could not bind MAVLink UDP port {self.udp_port} ({e}). "
                         "This typically means Mission Planner/QGC is already running or port is in use. "
                         "Direct UDP telemetry proxy is disabled, but config/switching will work normally.")

            return True
        except Exception as e:
            self.log(f"Error connecting to serial port: {e}")
            self.disconnect()
            return False

    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.udp_thread:
            self.udp_thread.join(timeout=1.0)

        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

        if self.udp_sock:
            self.udp_sock.close()
        self.udp_sock = None
        self.udp_client_addr = None

    def send_command(self, payload):
        if self.ser and self.ser.is_open:
            try:
                framed = mux_encode(CHAN_CONFIG, payload)
                self.ser.write(framed)
                self.ser.flush()
                return True
            except Exception as e:
                self.log(f"Error writing to serial: {e}")
        return False

    def set_mode(self, mode):
        self.log(f"Sending Mode Set Command: {mode}")
        return self.send_command([0x10, mode])

    def set_calibration(self, az_min, az_max, az_trim, az_rev, el_min, el_max, el_trim, el_rev):
        payload = [
            0x20,
            (az_min >> 8) & 0xFF, az_min & 0xFF,
            (az_max >> 8) & 0xFF, az_max & 0xFF,
            (az_trim >> 8) & 0xFF, az_trim & 0xFF,
            1 if az_rev else 0,
            (el_min >> 8) & 0xFF, el_min & 0xFF,
            (el_max >> 8) & 0xFF, el_max & 0xFF,
            (el_trim >> 8) & 0xFF, el_trim & 0xFF,
            1 if el_rev else 0
        ]
        self.log(f"Sending Servo Calibration Command: {payload}")
        return self.send_command(payload)

    def set_home_position(self, lat, lon, alt):
        lat_bytes = struct.pack('<f', lat)
        lon_bytes = struct.pack('<f', lon)
        alt_bytes = struct.pack('<f', alt)
        payload = [0x30] + list(lat_bytes) + list(lon_bytes) + list(alt_bytes)
        self.log(f"Sending Set Home Command: Lat={lat}, Lon={lon}, Alt={alt}")
        return self.send_command(payload)

    def set_vrx_config(self, rc_chan, positions_count, mapped_list):
        # mapped_list: list of 8 [band, channel] pairs
        payload = [0x40, rc_chan, positions_count]
        for band, chan in mapped_list:
            payload.append(band)
            payload.append(chan)

        self.log(f"Sending Extended VRX Table Config: RC_Chan={rc_chan}, Positions={positions_count}")
        return self.send_command(payload)

    def request_config_read(self):
        self.log("Requesting configuration from board...")
        return self.send_command([0x50])

    def set_cam_switch_config(self, active_camera, cam_rc_channel, manual_override):
        payload = [
            0x60,
            1 if active_camera else 0,
            cam_rc_channel,
            1 if manual_override else 0
        ]
        self.log(f"Sending Cam Switch Config: ActiveCam={active_camera}, RC_Chan={cam_rc_channel}, ManualPot={manual_override}")
        return self.send_command(payload)

    def _read_loop(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)

                        for b in data:
                            success, chan, payload = self.usb_mux_parser.parse_byte(b)
                            if success:
                                if chan == CHAN_CONFIG:
                                    self._parse_config_packet(payload)
                                elif chan == CHAN_MAVLINK:
                                    if self.udp_sock and self.udp_client_addr:
                                        try:
                                            self.udp_sock.sendto(payload, self.udp_client_addr)
                                        except Exception:
                                            pass
                except Exception as e:
                    self.log(f"Error in serial reading thread: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _udp_loop(self):
        while self.running:
            if self.udp_sock:
                try:
                    data, addr = self.udp_sock.recvfrom(2048)
                    if data:
                        self.udp_client_addr = addr
                        if self.ser and self.ser.is_open:
                            framed = mux_encode(CHAN_MAVLINK, data)
                            self.ser.write(framed)
                            self.ser.flush()
                except socket.timeout:
                    pass
                except Exception as e:
                    self.log(f"Error in UDP proxy thread: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _parse_config_packet(self, packet):
        # Packed config packet is now 57 bytes long inside CHAN_CONFIG multiplexer frame
        if len(packet) < 57:
            return

        system_mode = packet[0]
        az_min = (packet[1] << 8) | packet[2]
        az_max = (packet[3] << 8) | packet[4]
        az_trim = (packet[5] << 8) | packet[6]
        az_rev = packet[7]

        el_min = (packet[8] << 8) | packet[9]
        el_max = (packet[10] << 8) | packet[11]
        el_trim = (packet[12] << 8) | packet[13]
        el_rev = packet[14]

        home_lat = struct.unpack('<f', packet[15:19])[0]
        home_lon = struct.unpack('<f', packet[19:23])[0]
        home_alt = struct.unpack('<f', packet[23:27])[0]
        home_set = packet[27] == 1

        vrx_rc_chan = packet[28]
        vrx_band = packet[29]
        vrx_chan = packet[30]
        vrx_mhz = (packet[31] << 8) | packet[32]

        manual_override = packet[33]
        active_camera = packet[34]
        cam_rc_chan = packet[35]
        live_az = (packet[36] << 8) | packet[37]
        live_el = (packet[38] << 8) | packet[39]

        # New positions switch mappings
        vrx_positions_count = packet[40]
        vrx_mapped_channels = []
        idx = 41
        for i in range(8):
            vrx_mapped_channels.append([packet[idx], packet[idx+1]])
            idx += 2

        config_dict = {
            'system_mode': system_mode,
            'az_min': az_min,
            'az_max': az_max,
            'az_trim': az_trim,
            'az_rev': az_rev,
            'el_min': el_min,
            'el_max': el_max,
            'el_trim': el_trim,
            'el_rev': el_rev,
            'home_lat': home_lat,
            'home_lon': home_lon,
            'home_alt': home_alt,
            'home_set': home_set,
            'vrx_rc_chan': vrx_rc_chan,
            'vrx_band': vrx_band,
            'vrx_chan': vrx_chan,
            'vrx_mhz': vrx_mhz,
            'manual_override': manual_override,
            'active_camera': active_camera,
            'cam_rc_chan': cam_rc_chan,
            'live_az': live_az,
            'live_el': live_el,
            'vrx_positions_count': vrx_positions_count,
            'vrx_mapped_channels': vrx_mapped_channels
        }

        if self.on_config_received_cb:
            self.on_config_received_cb(config_dict)
ZOOM = 1.0
