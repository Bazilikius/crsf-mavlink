# board1_ground/main.py
import machine
import sys
import math
import struct
import uselect as select
import time
import micropython

# Disable REPL keyboard interrupts to allow 100% binary-safe USB serial data streaming!
micropython.kbd_intr(-1)

# --- Shared Multiplexer Protocol (Embedded for Self-Containment) ---
SYNC1 = 0xAA
SYNC2 = 0x55

CHAN_CRSF = 0x01
CHAN_MAVLINK = 0x02
CHAN_CONFIG = 0x03

class MuxParser:
    def __init__(self):
        self.state = 0 # 0: SYNC1, 1: SYNC2, 2: CHAN_ID, 3: LEN, 4: PAYLOAD, 5: CHECKSUM
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
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    out = bytearray([SYNC1, SYNC2, chan_id, len(payload)])
    out.extend(payload)
    cksum = (chan_id + len(payload)) & 0xFF
    for b in payload:
        cksum = (cksum + b) & 0xFF
    out.append(cksum)
    return bytes(out)

# Adaptive, 100% Binary-Safe VCP Stream Reader and Writer Helpers
_has_stdout_buffer = hasattr(sys.stdout, 'buffer')
def write_stdout_bytes(data):
    if _has_stdout_buffer:
        sys.stdout.buffer.write(data)
    else:
        sys.stdout.write(data.decode('latin-1'))

_has_stdin_buffer = hasattr(sys.stdin, 'buffer')
def read_stdin_byte():
    if _has_stdin_buffer:
        b = sys.stdin.buffer.read(1)
        return b[0] if b else None
    else:
        char = sys.stdin.read(1)
        return ord(char) if char else None

# --- Hardware Configuration Pin Mappings ---
PIN_I2C_SDA = 16
PIN_I2C_SCL = 17
PIN_UART_TX = 4
PIN_UART_RX = 5
PIN_CAM_SWITCH = 18
PIN_ADC_POT_AZ = 26
PIN_ADC_POT_EL = 27

# --- Global System Configuration & State ---
class SystemConfig:
    def __init__(self):
        self.system_mode = 3 # 1: JR1, 2: JR2, 3: Simultaneous

        # Servo calibration (us)
        self.azimuth_min_us = 1000
        self.azimuth_max_us = 2000
        self.azimuth_trim_us = 1500
        self.azimuth_reversed = 0

        self.elevation_min_us = 1000
        self.elevation_max_us = 2000
        self.elevation_trim_us = 1500
        self.elevation_reversed = 0

        # GPS/Home coordinates
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.home_set = False

        # VRX & Camera
        self.vrx_rc_channel = 8
        self.vrx_band = 3
        self.vrx_channel = 0
        self.vrx_frequency_mhz = 5740

        self.manual_override = 0
        self.active_camera = 1 # 1: VRX, 0: Analog
        self.cam_rc_channel = 7

        # Real-time state
        self.live_azimuth_deg = 0
        self.live_elevation_deg = 0

        # Customizable toggle switch positions mapping for video receiver (FT System 5.8G)
        self.vrx_positions_count = 3 # Default to a 3-position toggle switch
        self.vrx_mapped_channels = [
            [0, 0], # Pos 1 (0): Band A, Channel 1 (5865 MHz)
            [3, 0], # Pos 2 (1): Band F, Channel 1 (5740 MHz)
            [4, 0], # Pos 3 (2): Band R, Channel 1 (5658 MHz)
            [0, 0], # Pos 4 (3): Default A1
            [0, 0], # Pos 5 (4): Default A1
            [0, 0], # Pos 6 (5): Default A1
            [0, 0], # Pos 7 (6): Default A1
            [0, 0]  # Pos 8 (7): Default A1
        ]

        # S2 & 6POS VRX Control parameters
        self.vrx_control_mode = 3      # 1: Only S2, 2: Only 6POS, 3: S2 + 6POS, 4: Mapping Table
        self.vrx_s2_rc_channel = 8     # S2 default channel 8
        self.vrx_s2_switch_type = 8    # S2 default switch type: 8pos
        self.vrx_6pos_rc_channel = 9   # 6POS default channel 9
        self.vrx_6pos_switch_type = 6  # 6POS default switch type: 6pos

config = SystemConfig()
mux_parser = MuxParser()
pc_mux_parser = MuxParser()

# --- Hardware Initializations ---
uart1 = machine.UART(1, baudrate=460800, tx=machine.Pin(PIN_UART_TX), rx=machine.Pin(PIN_UART_RX), rxbuf=4096)
i2c0 = machine.I2C(0, sda=machine.Pin(PIN_I2C_SDA), scl=machine.Pin(PIN_I2C_SCL), freq=400000)
adc_pot_az = machine.ADC(machine.Pin(PIN_ADC_POT_AZ))
adc_pot_el = machine.ADC(machine.Pin(PIN_ADC_POT_EL))
cam_switch_pin = machine.Pin(PIN_CAM_SWITCH, machine.Pin.OUT)

# --- Helper Functions ---
def update_servos(az_us, el_us):
    az_us = max(config.azimuth_min_us, min(az_us, config.azimuth_max_us))
    el_us = max(config.elevation_min_us, min(el_us, config.elevation_max_us))

    cmd = bytearray([0x70, (az_us >> 8) & 0xFF, az_us & 0xFF, (el_us >> 8) & 0xFF, el_us & 0xFF])
    enc = mux_encode(CHAN_CONFIG, cmd)
    uart1.write(enc)

# FT System 5.8G Frequencies Matrix (6 Bands x 8 Channels = 48 selectable frequencies)
VRX_I2C_ADDR = 0x35
VRX_FREQ_TABLE = [
    [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], # Band A
    [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866], # Band B
    [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], # Band E
    [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880], # Band F (Fatshark)
    [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], # Band R (Raceband)
    [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621]  # Band L (Lowband / Low frequency)
]

def vrx_set_frequency(mhz):
    config.vrx_frequency_mhz = mhz
    f_val = mhz - 479
    N = f_val // 2
    A = (f_val % 2) * 16
    reg_val = (A & 0x1F) | ((N & 0x1FF) << 5)

    cmd = bytearray([0x0F, reg_val & 0xFF, (reg_val >> 8) & 0xFF])
    try:
        i2c0.writeto(VRX_I2C_ADDR, cmd)
    except Exception:
        pass

def vrx_set_band_channel(band, channel):
    band = max(0, min(band, 5)) # 6 Bands: 0 to 5
    channel = max(0, min(channel, 7))
    config.vrx_band = band
    config.vrx_channel = channel
    mhz = VRX_FREQ_TABLE[band][channel]
    vrx_set_frequency(mhz)

def vrx_set_cam_switch(active_cam):
    config.active_camera = active_cam
    cam_switch_pin.value(1 if active_cam == 1 else 0)

def vrx_init():
    cam_switch_pin.value(1 if config.active_camera == 1 else 0)
    vrx_set_band_channel(config.vrx_band, config.vrx_channel)

# SSD1306 128x64 OLED Minimal Text Driver
OLED_ADDR = 0x3C
OLED_INIT_CMDS = [
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
    0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
    0xDB, 0x40, 0xA4, 0xA6, 0xAF
]

_oled_cmd_buf = bytearray(2)
def oled_send_cmd(cmd):
    _oled_cmd_buf[0] = 0x00
    _oled_cmd_buf[1] = cmd
    try:
        i2c0.writeto(OLED_ADDR, _oled_cmd_buf)
    except Exception:
        pass

_oled_data_hdr = bytearray([0x40])
def oled_send_data(data):
    try:
        i2c0.writeto(OLED_ADDR, _oled_data_hdr + data)
    except Exception:
        pass

def oled_init():
    for cmd in OLED_INIT_CMDS:
        oled_send_cmd(cmd)
    oled_clear()

def oled_clear():
    zero_buf = bytearray(128)
    for page in range(8):
        oled_send_cmd(0xB0 + page)
        oled_send_cmd(0x00)
        oled_send_cmd(0x10)
        oled_send_data(zero_buf)

FONT_5X7 = {
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
    'A': [0x7e, 0x11, 0x11, 0x11, 0x7e],
    'B': [0x7f, 0x49, 0x49, 0x49, 0x36],
    'C': [0x3e, 0x41, 0x41, 0x41, 0x22],
    'D': [0x7f, 0x41, 0x41, 0x22, 0x1c],
    'E': [0x7f, 0x49, 0x49, 0x49, 0x41],
    'F': [0x7f, 0x09, 0x09, 0x09, 0x01],
    'G': [0x3e, 0x41, 0x49, 0x49, 0x7a],
    'H': [0x7f, 0x08, 0x08, 0x08, 0x7f],
    'I': [0x00, 0x41, 0x7f, 0x41, 0x00],
    'J': [0x20, 0x40, 0x41, 0x3f, 0x01],
    'K': [0x7f, 0x08, 0x14, 0x22, 0x41],
    'L': [0x7f, 0x40, 0x40, 0x40, 0x40],
    'M': [0x7f, 0x02, 0x0c, 0x02, 0x7f],
    'N': [0x7f, 0x04, 0x08, 0x10, 0x7f],
    'O': [0x3e, 0x41, 0x41, 0x41, 0x3e],
    'P': [0x7f, 0x09, 0x09, 0x09, 0x06],
    'Q': [0x3e, 0x41, 0x51, 0x21, 0x5e],
    'R': [0x7f, 0x09, 0x19, 0x29, 0x46],
    'S': [0x46, 0x49, 0x49, 0x49, 0x31],
    'T': [0x01, 0x01, 0x7f, 0x01, 0x01],
    'U': [0x3f, 0x40, 0x40, 0x40, 0x3f],
    'V': [0x1f, 0x20, 0x40, 0x20, 0x1f],
    'W': [0x3f, 0x40, 0x38, 0x40, 0x3f],
    'X': [0x63, 0x14, 0x08, 0x14, 0x63],
    'Y': [0x07, 0x08, 0x70, 0x08, 0x07],
    'Z': [0x61, 0x51, 0x49, 0x45, 0x43],
    '0': [0x3e, 0x51, 0x49, 0x45, 0x3e],
    '1': [0x00, 0x42, 0x7f, 0x40, 0x00],
    '2': [0x42, 0x61, 0x51, 0x49, 0x46],
    '3': [0x21, 0x41, 0x45, 0x4b, 0x31],
    '4': [0x18, 0x14, 0x12, 0x7f, 0x10],
    '5': [0x27, 0x45, 0x45, 0x45, 0x39],
    '6': [0x3c, 0x4a, 0x49, 0x49, 0x30],
    '7': [0x01, 0x71, 0x09, 0x05, 0x03],
    '8': [0x36, 0x49, 0x49, 0x49, 0x36],
    '9': [0x06, 0x49, 0x49, 0x29, 0x1e],
    ':': [0x00, 0x36, 0x36, 0x00, 0x00],
    '=': [0x14, 0x14, 0x14, 0x14, 0x14],
    '(': [0x00, 0x1c, 0x22, 0x41, 0x00],
    ')': [0x00, 0x41, 0x22, 0x1c, 0x00],
}

def oled_write_string(col, page, text):
    if page > 7:
         return
    oled_send_cmd(0xB0 + page)
    oled_send_cmd(col & 0x0F)
    oled_send_cmd(0x10 | ((col >> 4) & 0x0F))

    for c in text:
        upper_c = c.upper()
        char_pattern = FONT_5X7.get(upper_c, FONT_5X7[' '])
        out_buf = bytearray(char_pattern) + b'\x00'
        oled_send_data(out_buf)

def oled_update_display():
    oled_write_string(0, 0, "=== GS TRACKER ===")

    modes = {1: "JR1 Only", 2: "JR2 Only", 3: "Simultaneous"}
    mode_str = modes.get(config.system_mode, "Unknown")
    oled_write_string(0, 1, "MODE: {:<13}".format(mode_str))

    over_str = "POT" if config.manual_override == 1 else "AUTO"
    oled_write_string(0, 2, "AZ: {:<3} DEG ({})".format(config.live_azimuth_deg, over_str))
    oled_write_string(0, 3, "EL: {:<3} DEG".format(config.live_elevation_deg))

    cam_str = "VRX" if config.active_camera == 1 else "ANALOG"
    oled_write_string(0, 4, "CAM: {:<13}".format(cam_str))

    band_char = chr(ord('A') + config.vrx_band)
    if config.vrx_band == 3: band_char = 'F'
    elif config.vrx_band == 4: band_char = 'R'
    elif config.vrx_band == 5: band_char = 'L'
    oled_write_string(0, 5, "FRQ: {} (B{} C{})".format(config.vrx_frequency_mhz, band_char, config.vrx_channel + 1))

# --- ADC Potentiometer Processing ---
def tracker_read_potentiometers():
    if config.manual_override != 1:
        return
    raw_az = adc_pot_az.read_u16()
    raw_el = adc_pot_el.read_u16()

    config.live_azimuth_deg = int((raw_az * 360) / 65535)
    config.live_elevation_deg = int((raw_el * 180) / 65535)

    az_pct = raw_az / 65535.0
    if config.azimuth_reversed:
        az_pct = 1.0 - az_pct
    az_range = config.azimuth_max_us - config.azimuth_min_us
    az_us = config.azimuth_min_us + int(az_pct * az_range)

    el_pct = raw_el / 65535.0
    if config.elevation_reversed:
        el_pct = 1.0 - el_pct
    el_range = config.elevation_max_us - config.elevation_min_us
    el_us = config.elevation_min_us + int(el_pct * el_range)

    update_servos(az_us, el_us)

# --- MAVLink Tracking Parser with X.25 CRC ---
class MavlinkParser:
    def __init__(self):
        self.state = 0
        self.length = 0
        self.msg_id = 0
        self.payload = bytearray()
        self.crc = 0xFFFF
        self.is_v2 = False
        self.payload_idx = 0

    def crc_accumulate(self, byte):
        tmp = byte ^ (self.crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        self.crc = ((self.crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

    def parse_byte(self, b):
        if self.state == 0:
            if b == 0xFE:
                self.is_v2 = False
                self.crc = 0xFFFF
                self.state = 1
            elif b == 0xFD:
                self.is_v2 = True
                self.crc = 0xFFFF
                self.state = 1
        elif self.state == 1:
            self.length = b
            self.crc_accumulate(b)
            self.state = 2 if self.is_v2 else 4
        elif self.state == 2:
            self.crc_accumulate(b)
            self.state = 3
        elif self.state == 3:
            self.crc_accumulate(b)
            self.state = 4
        elif self.state == 4:
            self.crc_accumulate(b)
            self.state = 5
        elif self.state == 5:
            self.crc_accumulate(b)
            self.state = 6
        elif self.state == 6:
            self.crc_accumulate(b)
            self.state = 7
        elif self.state == 7:
            self.crc_accumulate(b)
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
            self.crc_accumulate(b)
            self.state = 9
        elif self.state == 9:
            self.msg_id |= (b << 16)
            self.crc_accumulate(b)
            self.payload = bytearray()
            self.payload_idx = 0
            self.state = 10
        elif self.state == 10:
            self.payload.append(b)
            self.crc_accumulate(b)
            self.payload_idx += 1
            if self.payload_idx >= self.length:
                self.state = 11
        elif self.state == 11:
            self.parsed_crc = b
            self.state = 12
        elif self.state == 12:
            self.parsed_crc |= (b << 8)
            self.state = 0

            extra = 104 if self.msg_id == 33 else 0
            tmp_crc = self.crc
            tmp_val = extra ^ (tmp_crc & 0xFF)
            tmp_val ^= (tmp_val << 4) & 0xFF
            final_crc = ((tmp_crc >> 8) ^ (tmp_val << 8) ^ (tmp_val << 3) ^ (tmp_val >> 4)) & 0xFFFF

            if final_crc == self.parsed_crc:
                self.handle_message()

    def handle_message(self):
        if config.manual_override == 1:
            return
        if self.msg_id == 33: # GLOBAL_POSITION_INT
            if len(self.payload) < 28: return
            lat_int = struct.unpack('<i', self.payload[4:8])[0]
            lon_int = struct.unpack('<i', self.payload[8:12])[0]
            alt_int = struct.unpack('<i', self.payload[16:20])[0]

            lat = lat_int / 1e7
            lon = lon_int / 1e7
            rel_alt = alt_int / 1000.0

            if not config.home_set:
                config.home_lat = lat
                config.home_lon = lon
                config.home_alt = 0.0
                config.home_set = True

            lat_rad = config.home_lat * (math.pi / 180.0)
            d_lat = lat - config.home_lat
            d_lon = lon - config.home_lon
            y = d_lat * 111139.0
            x = d_lon * 111139.0 * math.cos(lat_rad)
            z = rel_alt - config.home_alt

            az_deg = math.atan2(x, y) * (180.0 / math.pi)
            if az_deg < 0: az_deg += 360.0
            config.live_azimuth_deg = int(az_deg)

            dist = math.sqrt(x*x + y*y)
            el_deg = 0.0
            if dist > 0.1:
                el_deg = math.atan2(z, dist) * (180.0 / math.pi)
            el_deg = max(0.0, min(el_deg, 180.0))
            config.live_elevation_deg = int(el_deg)

            az_pct = az_deg / 360.0
            if config.azimuth_reversed: az_pct = 1.0 - az_pct
            az_us = config.azimuth_min_us + int(az_pct * (config.azimuth_max_us - config.azimuth_min_us))

            el_pct = el_deg / 180.0
            if config.elevation_reversed: el_pct = 1.0 - el_pct
            el_us = config.elevation_min_us + int(el_pct * (config.elevation_max_us - config.elevation_min_us))

            update_servos(az_us, el_us)

mav_parser = MavlinkParser()

# --- PC Commands and Configuration Serialization ---
def send_config_to_pc():
    # Build the 62-byte payload
    payload = bytearray([
        config.system_mode,
        (config.azimuth_min_us >> 8) & 0xFF, config.azimuth_min_us & 0xFF,
        (config.azimuth_max_us >> 8) & 0xFF, config.azimuth_max_us & 0xFF,
        (config.azimuth_trim_us >> 8) & 0xFF, config.azimuth_trim_us & 0xFF,
        config.azimuth_reversed,
        (config.elevation_min_us >> 8) & 0xFF, config.elevation_min_us & 0xFF,
        (config.elevation_max_us >> 8) & 0xFF, config.elevation_max_us & 0xFF,
        (config.elevation_trim_us >> 8) & 0xFF, config.elevation_trim_us & 0xFF,
        config.elevation_reversed,
    ])
    payload.extend(struct.pack('<f', config.home_lat))
    payload.extend(struct.pack('<f', config.home_lon))
    payload.extend(struct.pack('<f', config.home_alt))
    payload.append(1 if config.home_set else 0)

    payload.append(config.vrx_rc_channel)
    payload.append(config.vrx_band)
    payload.append(config.vrx_channel)
    payload.append((config.vrx_frequency_mhz >> 8) & 0xFF)
    payload.append(config.vrx_frequency_mhz & 0xFF)

    payload.append(config.manual_override)
    payload.append(config.active_camera)
    payload.append(config.cam_rc_channel)
    payload.append((config.live_azimuth_deg >> 8) & 0xFF)
    payload.append(config.live_azimuth_deg & 0xFF)
    payload.append((config.live_elevation_deg >> 8) & 0xFF)
    payload.append(config.live_elevation_deg & 0xFF)

    # Pack the position switch table mapping
    payload.append(config.vrx_positions_count)
    for i in range(8):
        payload.append(config.vrx_mapped_channels[i][0])
        payload.append(config.vrx_mapped_channels[i][1])

    # Append the 5 new S2 & 6POS VRX parameters
    payload.append(config.vrx_control_mode)
    payload.append(config.vrx_s2_rc_channel)
    payload.append(config.vrx_s2_switch_type)
    payload.append(config.vrx_6pos_rc_channel)
    payload.append(config.vrx_6pos_switch_type)

    packet = mux_encode(CHAN_CONFIG, payload)
    write_stdout_bytes(packet)

def process_pc_command(payload):
    if not payload: return
    cmd = payload[0]

    if cmd == 0x10:
        config.system_mode = payload[1]
        enc = mux_encode(CHAN_CONFIG, bytearray([0x10, config.system_mode]))
        uart1.write(enc)
    elif cmd == 0x20:
        config.azimuth_min_us = (payload[1] << 8) | payload[2]
        config.azimuth_max_us = (payload[3] << 8) | payload[4]
        config.azimuth_trim_us = (payload[5] << 8) | payload[6]
        config.azimuth_reversed = payload[7]
        config.elevation_min_us = (payload[8] << 8) | payload[9]
        config.elevation_max_us = (payload[10] << 8) | payload[11]
        config.elevation_trim_us = (payload[12] << 8) | payload[13]
        config.elevation_reversed = payload[14]
    elif cmd == 0x30:
        config.home_lat = struct.unpack('<f', bytes(payload[1:5]))[0]
        config.home_lon = struct.unpack('<f', bytes(payload[5:9]))[0]
        config.home_alt = struct.unpack('<f', bytes(payload[9:13]))[0]
        config.home_set = True
    elif cmd == 0x40: # Extended VRX configuration command
        if len(payload) >= 24:
            config.vrx_rc_channel = payload[1]
            config.vrx_positions_count = payload[2]

            config.vrx_control_mode = payload[3]
            config.vrx_s2_rc_channel = payload[4]
            config.vrx_s2_switch_type = payload[5]
            config.vrx_6pos_rc_channel = payload[6]
            config.vrx_6pos_switch_type = payload[7]

            # Unpack 8 mappings
            idx = 8
            for i in range(8):
                config.vrx_mapped_channels[i][0] = payload[idx]
                config.vrx_mapped_channels[i][1] = payload[idx+1]
                idx += 2

            # Trigger setup update based on control modes
            # If mapping table mode is selected, default to first mapping slot
            if config.vrx_control_mode == 4:
                b, ch = config.vrx_mapped_channels[0]
                vrx_set_band_channel(b, ch)
    elif cmd == 0x50:
        send_config_to_pc()
    elif cmd == 0x60:
        config.active_camera = payload[1]
        config.cam_rc_channel = payload[2]
        config.manual_override = payload[3]
        vrx_set_cam_switch(config.active_camera)

# CRSF RC Channel Decoder for VRX/Cam Toggles
crsf_state = 0
crsf_len = 0
crsf_type = 0
crsf_payload = bytearray()

def resolve_switch_position(val, switch_type):
    # Map raw CRSF 172..1811 range cleanly into switch positions (0 to switch_type - 1)
    if val < 172: val = 172
    if val > 1811: val = 1811
    pos = int(((val - 172) * switch_type) / 1640)
    return max(0, min(pos, switch_type - 1))

def process_crsf_byte(b):
    global crsf_state, crsf_len, crsf_type, crsf_payload
    if crsf_state == 0:
        if b == 0xC8:
            crsf_state = 1
    elif crsf_state == 1:
        if 2 <= b <= 62:
            crsf_len = b
            crsf_state = 2
        else:
            crsf_state = 0
    elif crsf_state == 2:
        crsf_type = b
        crsf_payload = bytearray()
        if crsf_len > 2:
            crsf_state = 3
        else:
            crsf_state = 4
    elif crsf_state == 3:
        crsf_payload.append(b)
        if len(crsf_payload) >= crsf_len - 2:
            crsf_state = 4
    elif crsf_state == 4:
        if crsf_type == 0x16 and len(crsf_payload) >= 22:
            channels = [0]*16
            channels[0]  = (crsf_payload[0]       | crsf_payload[1]  << 8) & 0x07FF
            channels[1]  = (crsf_payload[1]  >> 3 | crsf_payload[2]  << 5) & 0x07FF
            channels[2]  = (crsf_payload[2]  >> 6 | crsf_payload[3]  << 2 | crsf_payload[4] << 10) & 0x07FF
            channels[3]  = (crsf_payload[4]  >> 1 | crsf_payload[5]  << 7) & 0x07FF
            channels[4]  = (crsf_payload[5]  >> 4 | crsf_payload[6]  << 4) & 0x07FF
            channels[5]  = (crsf_payload[6]  >> 7 | crsf_payload[7]  << 1 | crsf_payload[8] << 9) & 0x07FF
            channels[6]  = (crsf_payload[8]  >> 2 | crsf_payload[9]  << 6) & 0x07FF
            channels[7]  = (crsf_payload[9]  >> 5 | crsf_payload[10] << 3) & 0x07FF
            channels[8]  = (crsf_payload[11]      | crsf_payload[12] << 8) & 0x07FF
            channels[9]  = (crsf_payload[12] >> 3 | crsf_payload[13] << 5) & 0x07FF
            channels[10] = (crsf_payload[13] >> 6 | crsf_payload[14] << 2 | crsf_payload[15] << 10) & 0x07FF
            channels[11] = (crsf_payload[15] >> 1 | crsf_payload[16] << 7) & 0x07FF
            channels[12] = (crsf_payload[16] >> 4 | crsf_payload[17] << 4) & 0x07FF
            channels[13] = (crsf_payload[17] >> 7 | crsf_payload[18] << 1 | crsf_payload[19] << 9) & 0x07FF
            channels[14] = (crsf_payload[19] >> 2 | crsf_payload[20] << 6) & 0x07FF
            channels[15] = (crsf_payload[20] >> 5 | crsf_payload[21] << 3) & 0x07FF

            # PROCESS VRX CONTROL MODE
            if config.vrx_control_mode == 1:
                # 1. ONLY S2 Controls Video Channel (0 to 7)
                s2_val = channels[config.vrx_s2_rc_channel - 1]
                if 172 <= s2_val <= 1811:
                    s2_pos = resolve_switch_position(s2_val, config.vrx_s2_switch_type)
                    # Video channel mapped directly: s2_pos clamped to 0..7
                    target_chan = min(s2_pos, 7)
                    if target_chan != config.vrx_channel:
                        vrx_set_band_channel(config.vrx_band, target_chan)
                        send_config_to_pc()

            elif config.vrx_control_mode == 2:
                # 2. ONLY 6POS Controls Video Band (0 to 5)
                p6_val = channels[config.vrx_6pos_rc_channel - 1]
                if 172 <= p6_val <= 1811:
                    p6_pos = resolve_switch_position(p6_val, config.vrx_6pos_switch_type)
                    # Video band mapped directly: p6_pos clamped to 0..5 (Band A, B, E, F, R, L)
                    target_band = min(p6_pos, 5)
                    if target_band != config.vrx_band:
                        vrx_set_band_channel(target_band, config.vrx_channel)
                        send_config_to_pc()

            elif config.vrx_control_mode == 3:
                # 3. S2 + 6POS Simultaneous operation: S2 sets Channel (0..7), 6POS sets Band (0..5)
                s2_val = channels[config.vrx_s2_rc_channel - 1]
                p6_val = channels[config.vrx_6pos_rc_channel - 1]
                if (172 <= s2_val <= 1811) and (172 <= p6_val <= 1811):
                    s2_pos = resolve_switch_position(s2_val, config.vrx_s2_switch_type)
                    p6_pos = resolve_switch_position(p6_val, config.vrx_6pos_switch_type)

                    target_chan = min(s2_pos, 7)
                    target_band = min(p6_pos, 5)

                    if target_chan != config.vrx_channel or target_band != config.vrx_band:
                        vrx_set_band_channel(target_band, target_chan)
                        send_config_to_pc()

            elif config.vrx_control_mode == 4:
                # 4. Standard Customizable Mapping Table lookup
                vrx_ch = channels[config.vrx_rc_channel - 1]
                if 172 <= vrx_ch <= 1811:
                    pos = int(((vrx_ch - 172) * config.vrx_positions_count) / 1640)
                    pos = max(0, min(pos, config.vrx_positions_count - 1))

                    target_band, target_chan = config.vrx_mapped_channels[pos]
                    if target_band != config.vrx_band or target_chan != config.vrx_channel:
                        vrx_set_band_channel(target_band, target_chan)
                        send_config_to_pc()

            # Cam Switch toggling
            cam_ch = channels[config.cam_rc_channel - 1]
            if 172 <= cam_ch <= 1811:
                target_cam = 1 if cam_ch > 992 else 0
                if target_cam != config.active_camera:
                    vrx_set_cam_switch(target_cam)
                    send_config_to_pc()
        crsf_state = 0

# --- Main Polling Engine ---
def main():
    vrx_init()
    oled_init()

    last_pot_update_ms = 0
    last_oled_update_ms = 0

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    poll.register(uart1, select.POLLIN)

    while True:
        now = time.ticks_ms()

        # 1. Read manual potentiometers (50ms interval)
        if time.ticks_diff(now, last_pot_update_ms) >= 50:
            last_pot_update_ms = now
            tracker_read_potentiometers()

        # 2. Update SSD1306 Display (500ms interval to completely avoid UART starve bottlenecks)
        if time.ticks_diff(now, last_oled_update_ms) >= 500:
            last_oled_update_ms = now
            oled_update_display()

        # 3. Read incoming bytes from USB stdin (VCP) non-blocking using select.select()
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            b = read_stdin_byte()
            if b is not None:
                success, chan, payload = pc_mux_parser.parse_byte(b)
                if success:
                    if chan == CHAN_CONFIG:
                        process_pc_command(payload)
                    elif chan == CHAN_MAVLINK:
                        uart1.write(mux_encode(CHAN_MAVLINK, payload))

        # 4. Read incoming bytes from UART1 (Board 2 inter-board link)
        if uart1.any():
            b_buf = uart1.read()
            if b_buf:
                for b in b_buf:
                    success, chan, payload = mux_parser.parse_byte(b)
                    if success:
                        if chan == CHAN_MAVLINK:
                            enc_val = mux_encode(CHAN_MAVLINK, payload)
                            write_stdout_bytes(enc_val)

                            for byte in payload:
                                mav_parser.parse_byte(byte)
                        elif chan == CHAN_CRSF:
                            enc_val = mux_encode(CHAN_CRSF, payload)
                            write_stdout_bytes(enc_val)

                            for byte in payload:
                                process_crsf_byte(byte)
                        elif chan == CHAN_CONFIG:
                            process_pc_command(payload)

if __name__ == '__main__':
    main()
