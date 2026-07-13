# board2_rf/main.py
import machine
import sys
import rp2
import select

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
    # Frequency is 8 * 420000 = 3360000 Hz
    # 8 cycles per bit
    pull()
    set(x, 7)            .side(0) [7] # Start bit (low) for 8 cycles
    label("bit_loop")
    out(pins, 1)                  [6] # Out 1 bit, wait 7 cycles total
    jmp(x_dec, "bit_loop")
    nop()                .side(1) [7] # Stop bit (high) for 8 cycles

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_uart_rx():
    label("start")
    wait(0, pin, 0)
    set(x, 7)            [10]
    label("bit_loop")
    in_(pins, 1)         [6]
    jmp(x_dec, "bit_loop")
    push()
    jmp("start")

# Instantiate PIO state machines on pio0 (Frequency: 3360000 Hz)
sm_tx = rp2.StateMachine(0, pio_uart_tx, freq=3360000, sideset_base=machine.Pin(PIN_JR2_TX), out_base=machine.Pin(PIN_JR2_TX))
sm_rx = rp2.StateMachine(1, pio_uart_rx, freq=3360000, in_base=machine.Pin(PIN_JR2_RX))

sm_tx.active(1)
sm_rx.active(1)

def pio_write(data):
    for b in data:
        sm_tx.put(b)

def pio_read():
    buf = bytearray()
    while sm_rx.rx_fifo():
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

# Parsing filters for JR1 Mixed Mode (Mode 1)
jr1_crsf_state = 0
jr1_crsf_buf = bytearray()
jr1_crsf_len = 0

jr1_mav_state = 0
jr1_mav_buf = bytearray()
jr1_mav_len = 0
jr1_mav_is_v2 = False

def parse_and_forward_jr1_mixed(b):
    global jr1_crsf_state, jr1_crsf_buf, jr1_crsf_len
    global jr1_mav_state, jr1_mav_buf, jr1_mav_len, jr1_mav_is_v2

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
        if len(jr1_crsf_buf) >= jr1_crsf_len + 2:
            calc = crsf_crc8(jr1_crsf_buf[2:-1])
            parsed = jr1_crsf_buf[-1]
            if calc == parsed:
                uart1.write(mux_encode(CHAN_CRSF, jr1_crsf_buf))
            jr1_crsf_state = 0

    # --- Permissive MAVLink Parser ---
    if jr1_mav_state == 0:
        if b == 0xFE: # v1
            jr1_mav_is_v2 = False
            jr1_mav_buf = bytearray([b])
            jr1_mav_state = 1
        elif b == 0xFD: # v2
            jr1_mav_is_v2 = True
            jr1_mav_buf = bytearray([b])
            jr1_mav_state = 1
    elif jr1_mav_state == 1:
        jr1_mav_len = b
        jr1_mav_buf.append(b)
        jr1_mav_state = 2
    elif jr1_mav_state == 2:
        jr1_mav_buf.append(b)
        target_len = (jr1_mav_len + 12) if jr1_mav_is_v2 else (jr1_mav_len + 8)
        if len(jr1_mav_buf) >= target_len:
            # Package and forward frame to Board 1
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
            if uart0.any():
                m_data = uart0.read()
                if m_data:
                    uart1.write(mux_encode(CHAN_MAVLINK, m_data))
            p_data = pio_read()
            if p_data:
                uart1.write(mux_encode(CHAN_CRSF, p_data))

if __name__ == '__main__':
    main()
