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
    chunks = []
    i = 0
    while i < len(payload):
        chunk = payload[i:i+255]
        out = bytearray([SYNC1, SYNC2, chan_id, len(chunk)])
        out.extend(chunk)
        cksum = (chan_id + len(chunk)) & 0xFF
        for b in chunk:
            cksum = (cksum + b) & 0xFF
        out.append(cksum)
        chunks.append(bytes(out))
        i += 255
    return b"".join(chunks)


class PythonMavlinkParser:
    def __init__(self, on_gps_cb=None):
        self.state = 0
        self.length = 0
        self.msg_id = 0
        self.payload = bytearray()
        self.is_v2 = False
        self.payload_idx = 0
        self.on_gps_cb = on_gps_cb

    def parse_byte(self, b):
        if self.state == 0:
            if b == 0xFE:
                self.is_v2 = False
                self.state = 1
            elif b == 0xFD:
                self.is_v2 = True
                self.state = 1
        elif self.state == 1:
            self.length = b
            self.state = 2 if self.is_v2 else 4
        elif self.state == 2 or self.state == 3:
            self.state += 1
        elif 4 <= self.state <= 6:
            self.state += 1
        elif self.state == 7:
            if self.is_v2:
                self.msg_id = b
                self.state = 8
            else:
                self.msg_id = b
                self.payload = bytearray()
                self.payload_idx = 0
                self.state = 10
        elif self.state == 8:
            self.msg_id |= (b << 8)
            self.state = 9
        elif self.state == 9:
            self.msg_id |= (b << 16)
            self.payload = bytearray()
            self.payload_idx = 0
            self.state = 10
        elif self.state == 10:
            self.payload.append(b)
            self.payload_idx += 1
            if self.payload_idx >= self.length:
                self.state = 11
        elif self.state == 11:
            self.state = 12
        elif self.state == 12:
            self.state = 0
            self.handle_message()

    def handle_message(self):
        if self.msg_id == 33: # GLOBAL_POSITION_INT
            if len(self.payload) < 28: return
            lat_int = struct.unpack('<i', self.payload[4:8])[0]
            lon_int = struct.unpack('<i', self.payload[8:12])[0]
            alt_int = struct.unpack('<i', self.payload[16:20])[0]
            lat = lat_int / 1e7
            lon = lon_int / 1e7
            alt = alt_int / 1000.0
            if self.on_gps_cb:
                self.on_gps_cb(lat, lon, alt)


class SerialConnection:
    def __init__(self, on_config_received_cb=None, on_telemetry_received_cb=None, log_message_cb=None):
        self.ser = None
        self.read_thread = None
        self.running = False
        self.on_config_received_cb = on_config_received_cb
        self.on_telemetry_received_cb = on_telemetry_received_cb
        self.log_message_cb = log_message_cb

        # MAVLink Virtual COM Port Redirector
        self.mav_redirect_port = None
        self.mav_redirect_ser = None
        self.mav_redirect_thread = None

        # MAVLink UDP Proxy Settings
        self.udp_sock = None
        self.udp_thread = None
        self.udp_client_addr = None
        self.udp_port = 14550 # Standard Mission Planner / QGC UDP Port

        # Secondary MAVLink UDP Proxy Settings (Aligned with MAVP2P bat script)
        self.udp_sock_sec = None
        self.udp_thread_sec = None
        self.udp_client_addr_sec = None
        self.udp_port_sec = 14445  # Receives UDP from MAVP2P udps:127.0.0.1:14445
        self.udp_tx_port_sec = 14446  # Transmits UDP to MAVP2P udpc:127.0.0.1:14446

        # Third MAVLink UDP Proxy Settings (for custom program)
        self.udp_sock_custom = None
        self.udp_thread_custom = None
        self.udp_client_addr_custom = None
        self.udp_port_custom = 14555  # Default custom program port

        # Dedicated MAVLink UDP 14556 Port Settings
        self.udp_sock_14556 = None
        self.udp_thread_14556 = None
        self.udp_client_addr_14556 = None
        self.udp_port_14556 = 14556

        # Parse state
        self.usb_mux_parser = MuxParser()
        self.local_mav_parser = PythonMavlinkParser(on_gps_cb=self._on_drone_gps_parsed)

    def _on_drone_gps_parsed(self, lat, lon, alt):
        if self.on_telemetry_received_cb:
            self.on_telemetry_received_cb({'lat': lat, 'lon': lon, 'alt': alt})

    @staticmethod
    def list_ports():
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def log(self, msg):
        if self.log_message_cb:
            self.log_message_cb(msg)
        else:
            print(msg)

    def connect(self, port, baudrate=115200, redirect_port=None):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.running = True

            # Start Background Read Thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            # Start MAVLink Virtual COM Port Redirector if specified
            if redirect_port:
                self.mav_redirect_port = redirect_port
                try:
                    self.mav_redirect_ser = serial.Serial(redirect_port, baudrate=115200, timeout=0.1)
                    self.mav_redirect_thread = threading.Thread(target=self._redirect_loop, daemon=True)
                    self.mav_redirect_thread.start()
                    self.log(f"MAVLink COM Redirector successfully started on {redirect_port}.")
                except Exception as e:
                    self.mav_redirect_ser = None
                    self.log(f"Warning: Could not open MAVLink Redirector port {redirect_port} ({e}).")

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

            # Try to bind the secondary MAVLink UDP Port 14556
            try:
                self.udp_sock_sec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock_sec.bind(('127.0.0.1', self.udp_port_sec))
                self.udp_sock_sec.settimeout(0.1)

                # Start background thread to read from 14556
                self.udp_thread_sec = threading.Thread(target=self._udp_loop_sec, daemon=True)
                self.udp_thread_sec.start()
                self.log(f"Secondary MAVLink UDP proxy server successfully started on port {self.udp_port_sec}, transmitting to port {self.udp_tx_port_sec}.")
            except OSError as e:
                self.udp_sock_sec = None
                self.udp_thread_sec = None
                self.log(f"Warning: Could not bind secondary MAVLink UDP port {self.udp_port_sec} ({e}).")

            # Try to bind the custom program MAVLink UDP Port
            try:
                self.udp_sock_custom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock_custom.bind(('127.0.0.1', self.udp_port_custom))
                self.udp_sock_custom.settimeout(0.1)

                # Start background thread to read from custom port
                self.udp_thread_custom = threading.Thread(target=self._udp_loop_custom, daemon=True)
                self.udp_thread_custom.start()
                self.log(f"Custom Program MAVLink UDP proxy server successfully started on port {self.udp_port_custom}.")
            except OSError as e:
                self.udp_sock_custom = None
                self.udp_thread_custom = None
                self.log(f"Warning: Could not bind custom program MAVLink UDP port {self.udp_port_custom} ({e}).")

            # Try to bind the dedicated MAVLink UDP Port 14556
            try:
                self.udp_sock_14556 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock_14556.bind(('127.0.0.1', self.udp_port_14556))
                self.udp_sock_14556.settimeout(0.1)

                # Start background thread to read from 14556
                self.udp_thread_14556 = threading.Thread(target=self._udp_loop_14556, daemon=True)
                self.udp_thread_14556.start()
                self.log(f"Dedicated MAVLink UDP 14556 proxy server successfully started on port {self.udp_port_14556}.")
            except OSError as e:
                self.udp_sock_14556 = None
                self.udp_thread_14556 = None
                self.log(f"Warning: Could not bind dedicated MAVLink UDP 14556 port {self.udp_port_14556} ({e}).")

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
        if self.udp_thread_sec:
            self.udp_thread_sec.join(timeout=1.0)
        if self.udp_thread_custom:
            self.udp_thread_custom.join(timeout=1.0)
        if self.udp_thread_14556:
            self.udp_thread_14556.join(timeout=1.0)
        if self.mav_redirect_thread:
            self.mav_redirect_thread.join(timeout=1.0)
        self.mav_redirect_thread = None

        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

        if self.mav_redirect_ser and self.mav_redirect_ser.is_open:
            try:
                self.mav_redirect_ser.close()
            except Exception:
                pass
        self.mav_redirect_ser = None

        if self.udp_sock:
            self.udp_sock.close()
        self.udp_sock = None
        self.udp_client_addr = None

        if self.udp_sock_sec:
            self.udp_sock_sec.close()
        self.udp_sock_sec = None
        self.udp_client_addr_sec = None

        if self.udp_sock_custom:
            self.udp_sock_custom.close()
        self.udp_sock_custom = None
        self.udp_client_addr_custom = None

        if self.udp_sock_14556:
            self.udp_sock_14556.close()
        self.udp_sock_14556 = None
        self.udp_client_addr_14556 = None

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

    def set_vrx_advanced_config(self, control_mode, s2_rc_channel, s2_switch_type, p6_rc_channel, p6_switch_type):
        payload = [
            0x40, # This can reuse or map to the 0x40 extended configuration command depending on how we handle it
            # To avoid collision or simplify, let's pass all values to the config.
            # In Board 1, process_pc_command checks command 0x40. We can format it to pass all 24 bytes of the vrx config layout:
            # [0x40, rc_channel, positions_count, control_mode, s2_rc, s2_type, p6_rc, p6_type, mappings (16 bytes)]
            # Let's write a comprehensive update helper instead, or handle it inside the GUI.
        ]
        pass

    def calibrate_azimuth_zero(self, ref_deg=0):
        self.log(f"Sending Calibrate Azimuth command to target reference: {ref_deg}° (0x80)...")
        payload = [0x80, (ref_deg >> 8) & 0xFF, ref_deg & 0xFF]
        return self.send_command(payload)

    def _redirect_loop(self):
        while self.running and self.mav_redirect_ser and self.mav_redirect_ser.is_open:
            try:
                if self.mav_redirect_ser.in_waiting > 0:
                    data = self.mav_redirect_ser.read(self.mav_redirect_ser.in_waiting)
                    if data:
                        if self.ser and self.ser.is_open:
                            framed = mux_encode(CHAN_MAVLINK, data)
                            self.ser.write(framed)
                            self.ser.flush()
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.log(f"Error in MAVLink Redirector loop: {e}")
                time.sleep(0.1)

    def _forward_mavlink_bytes(self, payload):
        # Forward MAVLink packet bytes to the local visual map parser!
        for byte in payload:
            self.local_mav_parser.parse_byte(byte)

        # Forward to MAVLink COM Redirector (written to GCS)
        if self.mav_redirect_ser and self.mav_redirect_ser.is_open:
            try:
                self.mav_redirect_ser.write(payload)
                self.mav_redirect_ser.flush()
            except Exception:
                pass

        # 1. Forward to primary UDP port client (Standard 14550)
        if self.udp_sock and self.udp_client_addr:
            try:
                self.udp_sock.sendto(payload, self.udp_client_addr)
            except Exception:
                pass

        # 2. Forward to secondary UDP transmit port (Port 2228)
        if self.udp_sock_sec:
            try:
                self.udp_sock_sec.sendto(payload, ('127.0.0.1', self.udp_tx_port_sec))
            except Exception:
                pass

        # 3. Forward to custom program UDP client
        if self.udp_sock_custom and self.udp_client_addr_custom:
            try:
                self.udp_sock_custom.sendto(payload, self.udp_client_addr_custom)
            except Exception:
                pass

        # 4. Forward to dedicated UDP port 14556
        if self.udp_sock_14556:
            try:
                target = self.udp_client_addr_14556 or ('127.0.0.1', 14556)
                self.udp_sock_14556.sendto(payload, target)
            except Exception:
                pass

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
                                    self._forward_mavlink_bytes(payload)
                            else:
                                # Adaptive parsing: if we are not in the middle of a multiplexed packet
                                # (state == 0) or we explicitly see a MAVLink header (0xFE, 0xFD),
                                # treat it as raw MAVLink and parse/forward directly!
                                if self.usb_mux_parser.state == 0 or b in [0xFE, 0xFD]:
                                    self._forward_mavlink_bytes(bytes([b]))
                except Exception as e:
                    self.log(f"Error in serial reading thread: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _udp_loop_14556(self):
        while self.running:
            if self.udp_sock_14556:
                try:
                    data, addr = self.udp_sock_14556.recvfrom(2048)
                    if data:
                        self.udp_client_addr_14556 = addr
                        if self.ser and self.ser.is_open:
                            framed = mux_encode(CHAN_MAVLINK, data)
                            self.ser.write(framed)
                            self.ser.flush()
                except (socket.timeout, TimeoutError):
                    pass
                except ConnectionResetError:
                    pass
                except OSError as e:
                    if getattr(e, 'winerror', 0) == 10054 or "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in UDP 14556 proxy thread: {e}")
                        time.sleep(0.1)
                except Exception as e:
                    if "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in UDP 14556 proxy thread: {e}")
                        time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _udp_loop_custom(self):
        while self.running:
            if self.udp_sock_custom:
                try:
                    data, addr = self.udp_sock_custom.recvfrom(2048)
                    if data:
                        self.udp_client_addr_custom = addr
                        if self.ser and self.ser.is_open:
                            framed = mux_encode(CHAN_MAVLINK, data)
                            self.ser.write(framed)
                            self.ser.flush()
                except (socket.timeout, TimeoutError):
                    pass
                except ConnectionResetError:
                    # Windows specific: UDP port unreachable ICMP response, safe to ignore
                    pass
                except OSError as e:
                    if getattr(e, 'winerror', 0) == 10054 or "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in custom program UDP proxy thread: {e}")
                        time.sleep(0.1)
                except Exception as e:
                    if "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in custom program UDP proxy thread: {e}")
                        time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _udp_loop_sec(self):
        while self.running:
            if self.udp_sock_sec:
                try:
                    data, addr = self.udp_sock_sec.recvfrom(2048)
                    if data:
                        self.udp_client_addr_sec = addr
                        if self.ser and self.ser.is_open:
                            framed = mux_encode(CHAN_MAVLINK, data)
                            self.ser.write(framed)
                            self.ser.flush()
                except (socket.timeout, TimeoutError):
                    pass
                except ConnectionResetError:
                    # Windows specific: UDP port unreachable ICMP response, safe to ignore
                    pass
                except OSError as e:
                    if getattr(e, 'winerror', 0) == 10054 or "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in secondary UDP proxy thread: {e}")
                        time.sleep(0.1)
                except Exception as e:
                    if "timed out" in str(e).lower() or "timeout" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in secondary UDP proxy thread: {e}")
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
                except (socket.timeout, TimeoutError):
                    pass
                except ConnectionResetError:
                    # Windows specific: UDP port unreachable ICMP response, safe to ignore
                    pass
                except OSError as e:
                    if getattr(e, 'winerror', 0) == 10054 or "timed out" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in UDP proxy thread: {e}")
                        time.sleep(0.1)
                except Exception as e:
                    if "timed out" in str(e).lower():
                        pass
                    else:
                        self.log(f"Error in UDP proxy thread: {e}")
                        time.sleep(0.1)
            else:
                time.sleep(0.1)

    def _parse_config_packet(self, packet):
        # Packed config packet is now 57 bytes (basic mappings) or 62 bytes (including S2 / 6POS and offsets) long inside CHAN_CONFIG
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

        # Parse extra S2 & 6POS parameters if available
        vrx_control_mode = 3      # Default: S2 + 6POS
        vrx_s2_rc_channel = 8
        vrx_s2_switch_type = 8
        vrx_6pos_rc_channel = 9
        vrx_6pos_switch_type = 6

        jr1_crsf_baud = 4200
        jr1_mav_baud = 1152
        jr2_crsf_baud = 4200
        rf_board_online = 0
        mavlink_active = 0

        if len(packet) >= 62:
            vrx_control_mode = packet[57]
            vrx_s2_rc_channel = packet[58]
            vrx_s2_switch_type = packet[59]
            vrx_6pos_rc_channel = packet[60]
            vrx_6pos_switch_type = packet[61]

        if len(packet) >= 68:
            jr1_crsf_baud = (packet[62] << 8) | packet[63]
            jr1_mav_baud = (packet[64] << 8) | packet[65]
            jr2_crsf_baud = (packet[66] << 8) | packet[67]

        if len(packet) >= 70:
            rf_board_online = packet[68]
            mavlink_active = packet[69]

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
            'vrx_mapped_channels': vrx_mapped_channels,
            'vrx_control_mode': vrx_control_mode,
            'vrx_s2_rc_channel': vrx_s2_rc_channel,
            'vrx_s2_switch_type': vrx_s2_switch_type,
            'vrx_6pos_rc_channel': vrx_6pos_rc_channel,
            'vrx_6pos_switch_type': vrx_6pos_switch_type,
            'jr1_crsf_baud': jr1_crsf_baud,
            'jr1_mav_baud': jr1_mav_baud,
            'jr2_crsf_baud': jr2_crsf_baud,
            'rf_board_online': rf_board_online,
            'mavlink_active': mavlink_active
        }

        if self.on_config_received_cb:
            self.on_config_received_cb(config_dict)
ZOOM = 1.0
