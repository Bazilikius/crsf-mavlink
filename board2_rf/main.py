# board2_rf/main.py
import machine
import sys
import rp2
import select

# Import shared protocol
sys.path.append('')
sys.path.append('shared')
try:
    from shared.mux_protocol import MuxParser, mux_encode, CHAN_MAVLINK, CHAN_CRSF, CHAN_CONFIG
except ImportError:
    from mux_protocol import MuxParser, mux_encode, CHAN_MAVLINK, CHAN_CRSF, CHAN_CONFIG

# --- Hardware Configuration Pin Mappings ---
PIN_UART1_TX = 4
PIN_UART1_RX = 5
PIN_JR1_TX = 0
PIN_JR1_RX = 1
PIN_JR2_TX = 8
PIN_JR2_RX = 9
PIN_JR1_PWR = 12
PIN_JR2_PWR = 13

# --- Global Operational Mode ---
MODE_JR1_ALL = 1
MODE_JR2_CRSF = 2
MODE_SIMULTANEOUS = 3

active_mode = MODE_SIMULTANEOUS
mux_parser = MuxParser()

# --- Hardware Initializations ---
# 1. UART1 for Board 1 link (Baud 460800)
uart1 = machine.UART(1, baudrate=460800, tx=machine.Pin(PIN_UART1_TX), rx=machine.Pin(PIN_UART1_RX))

# 2. UART0 for JR Module 1 (Baud 115200 for CRSF/MAVLink)
uart0 = machine.UART(0, baudrate=115200, tx=machine.Pin(PIN_JR1_TX), rx=machine.Pin(PIN_JR1_RX))

# 3. Power Enable Pins
jr1_pwr_pin = machine.Pin(PIN_JR1_PWR, machine.Pin.OUT)
jr2_pwr_pin = machine.Pin(PIN_JR2_PWR, machine.Pin.OUT)

# --- PIO Soft-UART Driver for JR Module 2 (CRSF @ 420000 bps) ---
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH, out_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_uart_tx():
    # 8 cycles per bit at 8 * 420000Hz clock frequency
    pull()
    set(pins, 0)         .side(0) [7] # Start bit
    label("bit_loop")
    out(pins, 1)                  [6] # Shift out data bits
    jmp(not_osre, "bit_loop")
    nop()                .side(1) [7] # Stop bit

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_uart_rx():
    label("start")
    wait(0, pin, 0)               # Wait for start bit
    set(x, 7)            [10]     # Pre-delay to center sample
    label("bit_loop")
    in_(pins, 1)         [6]      # Sample bits
    jmp(x_dec, "bit_loop")
    push()
    jmp("start")

# Instantiate PIO state machines on pio0 (Frequency: 8 * 420000 = 3360000 Hz)
sm_tx = rp2.StateMachine(0, pio_uart_tx, freq=3360000, sideset_base=machine.Pin(PIN_JR2_TX), out_base=machine.Pin(PIN_JR2_TX))
sm_rx = rp2.StateMachine(1, pio_uart_rx, freq=3360000, in_base=machine.Pin(PIN_JR2_RX))

sm_tx.active(1)
sm_rx.active(1)

def pio_write(data):
    for b in data:
        # MicroPython StateMachine.put accepts 32-bit values. Align left for RIGHT shifting.
        sm_tx.put(b << 24)

def pio_read():
    buf = bytearray()
    while sm_rx.rx_fifo():
        # Get 32-bit value and extract the shifted 8-bit byte
        val = sm_rx.get() >> 24
        buf.append(val)
    return bytes(buf)

# --- Switcher Logic and Module Powering ---
def switcher_set_mode(mode):
    global active_mode
    active_mode = mode
    if mode == MODE_JR1_ALL:
        jr1_pwr_pin.value(1)
        jr2_pwr_pin.value(0)
    elif mode == MODE_JR2_CRSF:
        jr1_pwr_pin.value(0)
        jr2_pwr_pin.value(1)
    elif mode == MODE_SIMULTANEOUS:
        jr1_pwr_pin.value(1)
        jr2_pwr_pin.value(1)

switcher_set_mode(active_mode)

# --- Checksum Validated Stream Splitters ---
# 1. CRSF CRC-8
def crsf_crc8(ptr):
    crc = 0
    for b in ptr:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# 2. MAVLink X.25 CRC
def mavlink_crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

def get_mavlink_crc_extra(msg_id):
    extras = {0: 50, 1: 103, 24: 30, 30: 39, 33: 104, 74: 20}
    return extras.get(msg_id, 0)

# Parsing filters for JR1 Mixed Mode (Mode 1)
jr1_crsf_state = 0
jr1_crsf_buf = bytearray()
jr1_crsf_len = 0

jr1_mav_state = 0
jr1_mav_buf = bytearray()
jr1_mav_len = 0
jr1_mav_crc = 0xFFFF
jr1_mav_is_v2 = False

def parse_and_forward_jr1_mixed(b):
    global jr1_crsf_state, jr1_crsf_buf, jr1_crsf_len
    global jr1_mav_state, jr1_mav_buf, jr1_mav_len, jr1_mav_crc, jr1_mav_is_v2

    # --- CRSF Checksum Validated Parser ---
    if jr1_crsf_state == 0:
        if b in [0xC8, 0xEE, 0xEA]:
            jr1_crsf_buf = bytearray([b])
            jr1_crsf_state = 1
    elif jr1_crsf_state == 1:
        if 2 <= b <= 62:
            jr1_crsf_buf.append(b)
            jr1_crsf_len = b
            jr1_crsf_state = 2
        else:
            jr1_crsf_state = 0
    elif jr1_crsf_state == 2:
        jr1_crsf_buf.append(b)
        # Total CRSF packet length is length + 2 (addr and length bytes)
        if len(jr1_crsf_buf) >= jr1_crsf_len + 2:
            # Validate CRC-8 on payload (from type byte at index 2 to end of payload)
            calc = crsf_crc8(jr1_crsf_buf[2:-1])
            parsed = jr1_crsf_buf[-1]
            if calc == parsed:
                uart1.write(mux_encode(CHAN_CRSF, jr1_crsf_buf))
            jr1_crsf_state = 0

    # --- MAVLink Checksum Validated Parser ---
    if jr1_mav_state == 0:
        if b == 0xFE: # v1
            jr1_mav_is_v2 = False
            jr1_mav_crc = 0xFFFF
            jr1_mav_buf = bytearray([b])
            jr1_mav_state = 1
        elif b == 0xFD: # v2
            jr1_mav_is_v2 = True
            jr1_mav_crc = 0xFFFF
            jr1_mav_buf = bytearray([b])
            jr1_mav_state = 1
    elif jr1_mav_state == 1:
        jr1_mav_len = b
        jr1_mav_crc = mavlink_crc_accumulate(b, jr1_mav_crc)
        jr1_mav_buf.append(b)
        jr1_mav_state = 2
    elif jr1_mav_state == 2:
        jr1_mav_buf.append(b)
        target_len = (jr1_mav_len + 12) if jr1_mav_is_v2 else (jr1_mav_len + 8)

        # Accumulate CRC on all bytes excluding STX and the last two CRC bytes
        if len(jr1_mav_buf) <= target_len - 2:
            jr1_mav_crc = mavlink_crc_accumulate(b, jr1_mav_crc)

        if len(jr1_mav_buf) >= target_len:
            # Parse MSG ID
            if jr1_mav_is_v2:
                msg_id = jr1_mav_buf[7] | (jr1_mav_buf[8] << 8) | (jr1_mav_buf[9] << 16)
            else:
                msg_id = jr1_mav_buf[5]

            parsed_crc = jr1_mav_buf[-2] | (jr1_mav_buf[-1] << 8)
            extra = get_mavlink_crc_extra(msg_id)
            final_crc = mavlink_crc_accumulate(extra, jr1_mav_crc)

            if final_crc == parsed_crc:
                uart1.write(mux_encode(CHAN_MAVLINK, jr1_mav_buf))
            jr1_mav_state = 0

# --- Command Parser ---
def switcher_process_command(payload):
    if not payload: return
    cmd = payload[0]
    if cmd == 0x10:
        switcher_set_mode(payload[1])

# --- Main Polling Engine ---
def main():
    poll = select.poll()
    poll.register(uart1, select.POLLIN)
    poll.register(uart0, select.POLLIN)

    while True:
        # 1. Inputs from Board 1 Ground Station (UART1)
        events = poll.poll(1)
        if events:
            if uart1.any():
                data = uart1.read()
                for b in data:
                    success, chan, payload = mux_parser.parse_byte(b)
                    if success:
                        if chan == CHAN_MAVLINK:
                            if active_mode in [MODE_JR1_ALL, MODE_SIMULTANEOUS]:
                                uart0.write(payload)
                        elif chan == CHAN_CRSF:
                            if active_mode == MODE_JR1_ALL:
                                uart0.write(payload)
                            elif active_mode in [MODE_JR2_CRSF, MODE_SIMULTANEOUS]:
                                pio_write(payload)
                        elif chan == CHAN_CONFIG:
                            switcher_process_command(payload)

        # 2. Inputs from active JR Modules
        if active_mode == MODE_JR1_ALL:
            if uart0.any():
                b_buf = uart0.read()
                for b in b_buf:
                    parse_and_forward_jr1_mixed(b)

        elif active_mode == MODE_JR2_CRSF:
            p_data = pio_read()
            if p_data:
                uart1.write(mux_encode(CHAN_CRSF, p_data))

        elif active_mode == MODE_SIMULTANEOUS:
            # JR1 handles MAVLink exclusively
            if uart0.any():
                m_data = uart0.read()
                if m_data:
                    uart1.write(mux_encode(CHAN_MAVLINK, m_data))
            # JR2 handles CRSF exclusively
            p_data = pio_read()
            if p_data:
                uart1.write(mux_encode(CHAN_CRSF, p_data))

if __name__ == '__main__':
    main()
