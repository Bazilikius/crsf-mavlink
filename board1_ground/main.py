# board1_ground/main.py
import machine
import sys
import math
import struct
import uselect as select
import time
import micropython

# 2-second safety delay to allow IDE connection and interruption (prevents "board busy" lockups on auto launch)
time.sleep(2)

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

import rp2

# --- PIO Soft-UART Drivers for CH340 Adapter ---
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH, out_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT, sideset_opt=True)
def pio_uart_tx():
    pull()
    set(x, 7)            .side(0) [7] # Start bit (low) for 8 cycles (1 set + 7 delay)
    label("bit_loop")
    out(pins, 1)                  [6] # Out 1 bit (1 out + 6 delay = 7 cycles)
    jmp(x_dec, "bit_loop")            # JMP instruction (1 cycle) -> Loop body = exactly 8 cycles!
    nop()                .side(1) [7] # Stop bit (high) for 8 cycles (1 nop + 7 delay)

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_uart_rx():
    label("start")
    wait(0, pin, 0)
    set(x, 7)            [10]         # 1 set + 10 delay = 11 cycles. Sampling at cycle 11 aligns near middle of first bit.
    label("bit_loop")
    in_(pins, 1)         [6]          # In 1 bit (1 in + 6 delay = 7 cycles)
    jmp(x_dec, "bit_loop")            # JMP instruction (1 cycle) -> Loop body = exactly 8 cycles per bit!
    push()
    jmp("start")

sm_ch340_tx = None
sm_ch340_rx = None

def pio_write_ch340(data):
    if sm_ch340_tx is not None:
        for b in data:
            sm_ch340_tx.put(b)

def pio_read_ch340():
    res = bytearray()
    if sm_ch340_rx is not None:
        while sm_ch340_rx.rx_fifo():
            val = (sm_ch340_rx.get() >> 24) & 0xFF
            res.append(val)
    return bytes(res) if len(res) > 0 else None

# Adaptive, 100% Binary-Safe VCP Stream Reader and Writer Helpers
try:
    _usb = machine.USB_VCP()
except Exception:
    _usb = None

def write_stdout_vcp_only(data):
    if _usb is not None:
        try:
            _usb.write(data)
        except Exception:
            pass
    if hasattr(sys.stdout, 'buffer'):
        try:
            sys.stdout.buffer.write(data)
        except Exception:
            pass

def write_stdout_bytes(data):
    write_stdout_vcp_only(data)
    try:
        pio_write_ch340(data)
    except Exception:
        pass

def read_stdin_byte():
    if _usb is not None:
        try:
            b = _usb.read(1)
            return b[0] if b else None
        except Exception:
            pass
    if hasattr(sys.stdin, 'buffer'):
        try:
            b = sys.stdin.buffer.read(1)
            return b[0] if b else None
        except Exception:
            pass
    try:
        char = sys.stdin.read(1)
        return ord(char) if char else None
    except Exception:
        return None

# --- Hardware Configuration Pin Mappings ---
PIN_I2C_SDA = 16
PIN_I2C_SCL = 17
PIN_UART_TX = 4
PIN_UART_RX = 5
PIN_CAM_SWITCH = 18
PIN_ADC_POT_AZ = 26
PIN_ADC_POT_EL = 27
PIN_TX16S_TX = 0
PIN_TX16S_RX = 1
PIN_CH340_TX = 12
PIN_CH340_RX = 13

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

        # JR Modules Baudrates Configuration (Baudrate / 100 for single byte fit)
        # e.g., 1152 for 115200, 4200 for 420000
        self.jr1_crsf_baud = 4200
        self.jr1_mav_baud = 1152
        self.jr2_crsf_baud = 4200

        # Calibration offsets
        self.azimuth_offset_deg = 0

config = SystemConfig()
mux_parser = MuxParser()
pc_mux_parser = MuxParser()
pc_mux_parser_ch340 = MuxParser()

# Connection status tracking
last_rf_board_msg_ms = 0
last_mav_msg_ms = 0
last_pc_mux_vcp_ms = 0
last_pc_mux_ch340_ms = 0

# --- Hardware Initializations ---
# Initialize CH340 Soft-UART State Machines using PIO
try:
    sm_ch340_tx = rp2.StateMachine(0, pio_uart_tx, freq=115200 * 8, sideset_base=machine.Pin(PIN_CH340_TX), out_base=machine.Pin(PIN_CH340_TX))
    sm_ch340_rx = rp2.StateMachine(1, pio_uart_rx, freq=115200 * 8, in_base=machine.Pin(PIN_CH340_RX, machine.Pin.IN, machine.Pin.PULL_UP))
    sm_ch340_tx.active(1)
    sm_ch340_rx.active(1)
except Exception:
    sm_ch340_tx = None
    sm_ch340_rx = None

uart1 = machine.UART(1, baudrate=115200, tx=machine.Pin(PIN_UART_TX), rx=machine.Pin(PIN_UART_RX), rxbuf=4096)
uart0 = machine.UART(0, baudrate=config.jr1_crsf_baud * 100, tx=machine.Pin(PIN_TX16S_TX), rx=machine.Pin(PIN_TX16S_RX))
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

# FT System 5.8G Frequencies Matrix (11 Bands x 8 Channels = 88 selectable frequencies)
VRX_I2C_ADDR = 0x35
VRX_FREQ_TABLE = [
    [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], # Band A
    [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866], # Band B
    [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], # Band E
    [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880], # Band F
    [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], # Band R
    [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621], # Band D
    [4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200], # Band X
    [5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613], # Band L
    [4867, 4884, 4921, 4958, 4995, 5032, 5069, 5099], # Band J
    [5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456], # Band U
    [5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600]  # Band O
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
    band = max(0, min(band, 10)) # 11 Bands: 0 to 10
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

    band_names = ["A", "B", "E", "F", "R", "D", "X", "L", "J", "U", "O"]
    band_char = band_names[config.vrx_band] if config.vrx_band < len(band_names) else "?"
    oled_write_string(0, 5, "FRQ: {} (B{} C{})".format(config.vrx_frequency_mhz, band_char, config.vrx_channel + 1))

# --- ADC Potentiometer Processing ---
def tracker_read_potentiometers():
    if config.manual_override != 1:
        return
    raw_az = adc_pot_az.read_u16()
    raw_el = adc_pot_el.read_u16()

    # Calculate initial raw azimuth degrees
    raw_az_deg = int((raw_az * 360) / 65535)

    # Apply calibrated zero reference offset
    config.live_azimuth_deg = (raw_az_deg - config.azimuth_offset_deg) % 360
    config.live_elevation_deg = int((raw_el * 180) / 65535)

    az_pct = (config.live_azimuth_deg / 360.0)
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

            # Apply Azimuth Zero Reference Offset!
            config.live_azimuth_deg = int((az_deg - config.azimuth_offset_deg) % 360)

            dist = math.sqrt(x*x + y*y)
            el_deg = 0.0
            if dist > 0.1:
                el_deg = math.atan2(z, dist) * (180.0 / math.pi)
            el_deg = max(0.0, min(el_deg, 180.0))
            config.live_elevation_deg = int(el_deg)

            # Map to servos
            az_pct = config.live_azimuth_deg / 360.0
            if config.azimuth_reversed: az_pct = 1.0 - az_pct
            az_us = config.azimuth_min_us + int(az_pct * (config.azimuth_max_us - config.azimuth_min_us))

            el_pct = el_deg / 180.0
            if config.elevation_reversed: el_pct = 1.0 - el_pct
            el_us = config.elevation_min_us + int(el_pct * (config.elevation_max_us - config.elevation_min_us))

            update_servos(az_us, el_us)

mav_parser = MavlinkParser()

# --- PC Commands and Configuration Serialization ---
def send_config_to_pc():
    # Only transmit config status if at least one PC interface is active
    global last_pc_mux_vcp_ms, last_pc_mux_ch340_ms
    now = time.ticks_ms()
    vcp_active = time.ticks_diff(now, last_pc_mux_vcp_ms) < 5000
    ch340_active = time.ticks_diff(now, last_pc_mux_ch340_ms) < 5000

    if not vcp_active and not ch340_active:
        return

    # Build the 68-byte payload
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

    # Append the 5 S2 & 6POS VRX parameters
    payload.append(config.vrx_control_mode)
    payload.append(config.vrx_s2_rc_channel)
    payload.append(config.vrx_s2_switch_type)
    payload.append(config.vrx_6pos_rc_channel)
    payload.append(config.vrx_6pos_switch_type)

    # Append the JR modules baudrate configurations (stored as 2-byte values, e.g. 115200 -> 1152)
    payload.append((config.jr1_crsf_baud >> 8) & 0xFF)
    payload.append(config.jr1_crsf_baud & 0xFF)
    payload.append((config.jr1_mav_baud >> 8) & 0xFF)
    payload.append(config.jr1_mav_baud & 0xFF)
    payload.append((config.jr2_crsf_baud >> 8) & 0xFF)
    payload.append(config.jr2_crsf_baud & 0xFF)

    # Append rf_board_online and mavlink_active connection indicators (Bytes 68 and 69)
    global last_rf_board_msg_ms, last_mav_msg_ms
    now = time.ticks_ms()
    rf_board_online = 1 if time.ticks_diff(now, last_rf_board_msg_ms) < 2500 else 0
    mavlink_active = 1 if time.ticks_diff(now, last_mav_msg_ms) < 3000 else 0

    payload.append(rf_board_online)
    payload.append(mavlink_active)

    packet = mux_encode(CHAN_CONFIG, payload)
    if vcp_active:
        write_stdout_vcp_only(packet)
    if ch340_active:
        pio_write_ch340(packet)

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

            if len(payload) >= 30:
                # Payload contains the 3 baudrate fields as well! (e.g. 6 bytes for 3 baudrates)
                config.jr1_crsf_baud = (payload[idx] << 8) | payload[idx+1]
                config.jr1_mav_baud = (payload[idx+2] << 8) | payload[idx+3]
                config.jr2_crsf_baud = (payload[idx+4] << 8) | payload[idx+5]
                # Dynamically update the TX16S CRSF UART0 baudrate on Board 1
                try:
                    uart0.init(baudrate=config.jr1_crsf_baud * 100, tx=machine.Pin(PIN_TX16S_TX), rx=machine.Pin(PIN_TX16S_RX))
                except Exception:
                    pass

            # Forward the exact configuration to board 2 RF switcher over UART1
            uart1.write(mux_encode(CHAN_CONFIG, payload))

            if config.vrx_control_mode == 4:
                b, ch = config.vrx_mapped_channels[0]
                vrx_set_band_channel(b, ch)
    elif cmd == 0x45: # Direct I2C Video Receiver Band/Channel Change Command
        band = max(0, min(payload[1], 10))
        channel = max(0, min(payload[2], 7))
        vrx_set_band_channel(band, channel)
        send_config_to_pc()
    elif cmd == 0x50:
        send_config_to_pc()
    elif cmd == 0x60:
        config.active_camera = payload[1]
        config.cam_rc_channel = payload[2]
        config.manual_override = payload[3]
        vrx_set_cam_switch(config.active_camera)
    elif cmd == 0x80: # Set Current Azimuth as Zero Point Calibration!
        # Set current potentiometer read heading as the zero azimuth heading calibration offset!
        raw_az = adc_pot_az.read_u16()
        raw_az_deg = int((raw_az * 360) / 65535)

        # Read the target reference degrees from the payload if provided
        ref_deg = 0
        if len(payload) >= 3:
            ref_deg = (payload[1] << 8) | payload[2]

        config.azimuth_offset_deg = (raw_az_deg - ref_deg) % 360
        config.live_azimuth_deg = ref_deg # Calibrated immediately to matching reference degrees
        send_config_to_pc()

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
                    target_chan = min(s2_pos, 7)
                    if target_chan != config.vrx_channel:
                        vrx_set_band_channel(config.vrx_band, target_chan)
                        send_config_to_pc()

            elif config.vrx_control_mode == 2:
                # 2. ONLY 6POS Controls Video Band (0 to 5)
                p6_val = channels[config.vrx_6pos_rc_channel - 1]
                if 172 <= p6_val <= 1811:
                    p6_pos = resolve_switch_position(p6_val, config.vrx_6pos_switch_type)
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

    # === Safe Physical Servo Homing Sequence ===
    # Drive Elevation servo to -10 degrees (888us) on boot to home mechanical structure safely!
    update_servos(config.azimuth_trim_us, 888)
    time.sleep_ms(1200) # Wait 1.2 seconds for safe homing
    # Move smoothly back to standard 0-degree point (1000us)
    update_servos(config.azimuth_trim_us, config.elevation_min_us)
    time.sleep_ms(300)

    last_pot_update_ms = 0
    last_oled_update_ms = 0
    last_pc_status_ms = 0

    vcp_mux_buf = bytearray()
    last_vcp_mux_byte_ms = 0

    ch340_mux_buf = bytearray()
    last_ch340_mux_byte_ms = 0

    poll = select.poll()
    if _usb is not None:
        poll.register(_usb, select.POLLIN)
    else:
        poll.register(sys.stdin, select.POLLIN)

    global last_rf_board_msg_ms, last_mav_msg_ms, last_pc_mux_vcp_ms, last_pc_mux_ch340_ms

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

        # 2a. Periodically send system config and connection/MAVLink status to PC (1000ms interval)
        if time.ticks_diff(now, last_pc_status_ms) >= 1000:
            last_pc_status_ms = now
            send_config_to_pc()

        # 3. Direct hardware UART polling (safe, robust, and bypasses select.poll() compatibility limits)
        if uart0.any():
            b_buf = uart0.read()
            if b_buf:
                # Forward raw CRSF from TX16S directly to Board 2 as multiplexed CHAN_CRSF packets
                uart1.write(mux_encode(CHAN_CRSF, b_buf))
                # Also parse locally for VRX/Cam switching logic
                for b in b_buf:
                    process_crsf_byte(b)

        if uart1.any():
            b_buf = uart1.read()
            if b_buf:
                last_rf_board_msg_ms = time.ticks_ms() # We received valid UART bytes from Board 2!
                for b in b_buf:
                    success, chan, payload = mux_parser.parse_byte(b)
                    if success:
                        if chan == CHAN_MAVLINK:
                            last_mav_msg_ms = time.ticks_ms() # MAVLink telemetry is actively transferring!

                            vcp_is_pc_mode = (time.ticks_diff(time.ticks_ms(), last_pc_mux_vcp_ms) < 5000)
                            if vcp_is_pc_mode:
                                # Send multiplexed MAVLink to PC Configurator VCP
                                write_stdout_vcp_only(mux_encode(CHAN_MAVLINK, payload))
                            else:
                                # Send RAW MAVLink to direct GCS VCP (Mission Planner/QGC)
                                write_stdout_vcp_only(payload)

                            ch340_is_pc_mode = (time.ticks_diff(time.ticks_ms(), last_pc_mux_ch340_ms) < 5000)
                            if ch340_is_pc_mode:
                                # Send multiplexed MAVLink to CH340
                                pio_write_ch340(mux_encode(CHAN_MAVLINK, payload))
                            else:
                                # Send RAW MAVLink to CH340
                                pio_write_ch340(payload)

                            for byte in payload:
                                mav_parser.parse_byte(byte)
                        elif chan == CHAN_CRSF:
                            vcp_is_pc_mode = (time.ticks_diff(time.ticks_ms(), last_pc_mux_vcp_ms) < 5000)
                            if vcp_is_pc_mode:
                                write_stdout_vcp_only(mux_encode(CHAN_CRSF, payload))

                            ch340_is_pc_mode = (time.ticks_diff(time.ticks_ms(), last_pc_mux_ch340_ms) < 5000)
                            if ch340_is_pc_mode:
                                pio_write_ch340(mux_encode(CHAN_CRSF, payload))

                            # Write received CRSF back to TX16S
                            try:
                                uart0.write(payload)
                            except Exception:
                                pass

                            for byte in payload:
                                process_crsf_byte(byte)
                        elif chan == CHAN_CONFIG:
                            # 0x99 is the periodic RF Switcher ping. If received, simply register connection.
                            if len(payload) > 0 and payload[0] == 0x99:
                                pass
                            else:
                                process_pc_command(payload)

        # VCP multiplexer parser timeout flush (100ms)
        if len(vcp_mux_buf) > 0 and time.ticks_diff(now, last_vcp_mux_byte_ms) > 100:
            if not (time.ticks_diff(now, last_pc_mux_vcp_ms) < 5000):
                i = 0
                while i < len(vcp_mux_buf):
                    chunk = vcp_mux_buf[i:i+255]
                    uart1.write(mux_encode(CHAN_MAVLINK, chunk))
                    i += 255
            vcp_mux_buf.clear()
            pc_mux_parser.state = 0

        # CH340 multiplexer parser timeout flush (100ms)
        if len(ch340_mux_buf) > 0 and time.ticks_diff(now, last_ch340_mux_byte_ms) > 100:
            if not (time.ticks_diff(now, last_pc_mux_ch340_ms) < 5000):
                i = 0
                while i < len(ch340_mux_buf):
                    chunk = ch340_mux_buf[i:i+255]
                    uart1.write(mux_encode(CHAN_MAVLINK, chunk))
                    i += 255
            ch340_mux_buf.clear()
            pc_mux_parser_ch340.state = 0

        # 4. Non-blocking high-speed VCP polling (Auto-detecting dual-mode PC Configurator / raw GCS COM connection)
        vcp_data = None
        if _usb is not None and _usb.any():
            vcp_data = _usb.read()
        else:
            # Fallback stdin non-blocking loop to read all available bytes
            stdin_bytes = bytearray()
            while True:
                events = poll.poll(0)
                has_input = False
                for obj, event in events:
                    if obj == sys.stdin and (event & select.POLLIN):
                        has_input = True
                        break
                if has_input:
                    if hasattr(sys.stdin, 'buffer'):
                        b = sys.stdin.buffer.read(1)
                    else:
                        b = sys.stdin.read(1).encode('latin-1')
                    if b:
                        stdin_bytes.extend(b)
                    else:
                        break
                else:
                    break
            if len(stdin_bytes) > 0:
                vcp_data = stdin_bytes

        if vcp_data:
            last_vcp_mux_byte_ms = now
            vcp_is_pc_mode = (time.ticks_diff(now, last_pc_mux_vcp_ms) < 5000)
            raw_vcp_in_buf = bytearray()

            for b in vcp_data:
                if vcp_is_pc_mode:
                    success, chan, payload = pc_mux_parser.parse_byte(b)
                    if success:
                        last_pc_mux_vcp_ms = time.ticks_ms()
                        if chan == CHAN_CONFIG:
                            process_pc_command(payload)
                        elif chan == CHAN_MAVLINK:
                            uart1.write(mux_encode(CHAN_MAVLINK, payload))
                else:
                    vcp_mux_buf.append(b)
                    success, chan, payload = pc_mux_parser.parse_byte(b)
                    if success:
                        last_pc_mux_vcp_ms = time.ticks_ms()
                        vcp_is_pc_mode = True
                        vcp_mux_buf.clear()
                        if chan == CHAN_CONFIG:
                            process_pc_command(payload)
                        elif chan == CHAN_MAVLINK:
                            uart1.write(mux_encode(CHAN_MAVLINK, payload))
                    elif pc_mux_parser.state == 0:
                        raw_vcp_in_buf.extend(vcp_mux_buf)
                        vcp_mux_buf.clear()

            if len(raw_vcp_in_buf) > 0:
                # Forward raw GCS MAVLink bytes to Board 2 inside CHAN_MAVLINK chunks
                i = 0
                while i < len(raw_vcp_in_buf):
                    chunk = raw_vcp_in_buf[i:i+255]
                    uart1.write(mux_encode(CHAN_MAVLINK, chunk))
                    i += 255

        # 5. Non-blocking high-speed CH340 Soft-UART polling
        ch340_data = pio_read_ch340()
        if ch340_data:
            last_ch340_mux_byte_ms = now
            ch340_is_pc_mode = (time.ticks_diff(now, last_pc_mux_ch340_ms) < 5000)
            raw_ch340_in_buf = bytearray()

            for b in ch340_data:
                if ch340_is_pc_mode:
                    success, chan, payload = pc_mux_parser_ch340.parse_byte(b)
                    if success:
                        last_pc_mux_ch340_ms = time.ticks_ms()
                        if chan == CHAN_CONFIG:
                            process_pc_command(payload)
                        elif chan == CHAN_MAVLINK:
                            uart1.write(mux_encode(CHAN_MAVLINK, payload))
                else:
                    ch340_mux_buf.append(b)
                    success, chan, payload = pc_mux_parser_ch340.parse_byte(b)
                    if success:
                        last_pc_mux_ch340_ms = time.ticks_ms()
                        ch340_is_pc_mode = True
                        ch340_mux_buf.clear()
                        if chan == CHAN_CONFIG:
                            process_pc_command(payload)
                        elif chan == CHAN_MAVLINK:
                            uart1.write(mux_encode(CHAN_MAVLINK, payload))
                    elif pc_mux_parser_ch340.state == 0:
                        raw_ch340_in_buf.extend(ch340_mux_buf)
                        ch340_mux_buf.clear()

            if len(raw_ch340_in_buf) > 0:
                # Forward raw GCS MAVLink bytes to Board 2 inside CHAN_MAVLINK chunks
                i = 0
                while i < len(raw_ch340_in_buf):
                    chunk = raw_ch340_in_buf[i:i+255]
                    uart1.write(mux_encode(CHAN_MAVLINK, chunk))
                    i += 255

        # Yield CPU slightly to keep the board running cool and prevent tight-loop starvation
        time.sleep_ms(1)

if __name__ == '__main__':
    main()
