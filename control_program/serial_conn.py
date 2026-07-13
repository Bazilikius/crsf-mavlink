import serial
import serial.tools.list_ports
import struct
import threading
import time
import socket

class SerialConnection:
    def __init__(self, on_config_received_cb=None):
        self.ser = None
        self.read_thread = None
        self.running = False
        self.on_config_received_cb = on_config_received_cb

        # MAVLink UDP Proxy Settings
        self.udp_sock = None
        self.udp_thread = None
        self.udp_client_addr = None
        self.udp_port = 14550 # Standard Mission Planner / QGC UDP Port

    @staticmethod
    def list_ports():
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.running = True

            # Start MAVLink UDP Proxy Socket
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.bind(('127.0.0.1', self.udp_port))
            self.udp_sock.settimeout(0.1)

            # Start Background Threads
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.udp_thread.start()

            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
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
                self.ser.write(bytes(payload))
                self.ser.flush()
                return True
            except Exception as e:
                print(f"Error writing to serial: {e}")
        return False

    def set_mode(self, mode):
        print(f"Sending Mode Set Command: {mode}")
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
        print(f"Sending Servo Calibration Command: {payload}")
        return self.send_command(payload)

    def set_home_position(self, lat, lon, alt):
        lat_bytes = struct.pack('<f', lat)
        lon_bytes = struct.pack('<f', lon)
        alt_bytes = struct.pack('<f', alt)
        payload = [0x30] + list(lat_bytes) + list(lon_bytes) + list(alt_bytes)
        print(f"Sending Set Home Command: Lat={lat}, Lon={lon}, Alt={alt}")
        return self.send_command(payload)

    def set_vrx_config(self, rc_chan, band, chan, mhz):
        payload = [
            0x40,
            rc_chan,
            band,
            chan,
            (mhz >> 8) & 0xFF, mhz & 0xFF
        ]
        print(f"Sending VRX Config Command: RC_Chan={rc_chan}, Band={band}, Chan={chan}, MHz={mhz}")
        return self.send_command(payload)

    def request_config_read(self):
        print("Requesting configuration from board...")
        return self.send_command([0x50])

    def set_cam_switch_config(self, active_camera, cam_rc_channel, manual_override):
        payload = [
            0x60,
            1 if active_camera else 0,
            cam_rc_channel,
            1 if manual_override else 0
        ]
        print(f"Sending Cam Switch Config: ActiveCam={active_camera}, RC_Chan={cam_rc_channel}, ManualPot={manual_override}")
        return self.send_command(payload)

    def _read_loop(self):
        buffer = bytearray()
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)

                        # Process bytes one by one to isolate Config packets and route MAVLink
                        i = 0
                        while i < len(data):
                            # Look for 43-byte robust Config response [0xCF][0xFC]
                            if i <= len(data) - 43 and data[i] == 0xCF and data[i+1] == 0xFC:
                                packet = data[i:i+43]
                                self._parse_config_packet(packet)
                                i += 43
                            else:
                                # Standard telemetry byte -> Route to MAVLink UDP Client (Mission Planner)
                                if self.udp_sock and self.udp_client_addr:
                                    try:
                                        self.udp_sock.sendto(bytes([data[i]]), self.udp_client_addr)
                                    except Exception:
                                        pass
                                i += 1
                except Exception as e:
                    print(f"Error in serial reading thread: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _udp_loop(self):
        # Listens for MAVLink packets from Mission Planner on port 14550 and forwards to YD-RP2040
        while self.running:
            if self.udp_sock:
                try:
                    data, addr = self.udp_sock.recvfrom(1024)
                    if data:
                        self.udp_client_addr = addr # Remember client address
                        if self.ser and self.ser.is_open:
                            self.ser.write(data)
                            self.ser.flush()
                except socket.timeout:
                    pass
                except Exception as e:
                    print(f"Error in UDP proxy thread: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _parse_config_packet(self, packet):
        if len(packet) < 43 or packet[0] != 0xCF or packet[1] != 0xFC:
            return

        calc_cksum = sum(packet[2:42]) & 0xFF
        parsed_cksum = packet[42]
        if calc_cksum != parsed_cksum:
            print("Config packet checksum validation failed, discarding packet.")
            return

        system_mode = packet[2]
        az_min = (packet[3] << 8) | packet[4]
        az_max = (packet[5] << 8) | packet[6]
        az_trim = (packet[7] << 8) | packet[8]
        az_rev = packet[9]

        el_min = (packet[10] << 8) | packet[11]
        el_max = (packet[12] << 8) | packet[13]
        el_trim = (packet[14] << 8) | packet[15]
        el_rev = packet[16]

        home_lat = struct.unpack('<f', packet[17:21])[0]
        home_lon = struct.unpack('<f', packet[21:25])[0]
        home_alt = struct.unpack('<f', packet[25:29])[0]
        home_set = packet[29] == 1

        vrx_rc_chan = packet[30]
        vrx_band = packet[31]
        vrx_chan = packet[32]
        vrx_mhz = (packet[33] << 8) | packet[34]

        manual_override = packet[35]
        active_camera = packet[36]
        cam_rc_chan = packet[37]
        live_az = (packet[38] << 8) | packet[39]
        live_el = (packet[40] << 8) | packet[41]

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
            'live_el': live_el
        }

        if self.on_config_received_cb:
            self.on_config_received_cb(config_dict)
ZOOM = 1.0
