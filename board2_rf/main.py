# board2_rf/main.py
import machine
import sys
import rp2
import select
import micropython
import time

# 2-second safety delay to allow IDE connection and interruption (prevents "board busy" lockups on auto launch)
time.sleep(2)

# --- Shared Multiplexer Protocol ---
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
        self.header_buf = bytearray()

    def parse_byte(self, b):
        # Returns (success, chan_id, payload, failed_raw_bytes)
        if self.state == 0:
            if b == SYNC1:
                self.header_buf = bytearray([b])
                self.state = 1
                return False, 0, b"", b""
            else:
                return False, 0, b"", bytes([b])
        elif self.state == 1:
            if b == SYNC2:
                self.header_buf.append(b)
                self.state = 2
                return False, 0, b"", b""
            elif b == SYNC1:
                failed = bytes(self.header_buf)
                self.header_buf = bytearray([b])
                self.state = 1
                return False, 0, b"", failed
            else:
                self.header_buf.append(b)
                failed = bytes(self.header_buf)
                self.header_buf = bytearray()
                self.state = 0
                return False, 0, b"", failed
        elif self.state == 2:
            if b in [CHAN_CRSF, CHAN_MAVLINK, CHAN_CONFIG]:
                self.header_buf.append(b)
                self.chan_id = b
                self.state = 3
                return False, 0, b"", b""
            elif b == SYNC1:
                failed = bytes(self.header_buf)
                self.header_buf = bytearray([b])
                self.state = 1
                return False, 0, b"", failed
            else:
                self.header_buf.append(b)
                failed = bytes(self.header_buf)
                self.header_buf = bytearray()
                self.state = 0
                return False, 0, b"", failed
        elif self.state == 3:
            self.header_buf.append(b)
            self.length = b
            self.payload = bytearray()
            if b == 0:
                self.state = 5
            else:
                self.state = 4
            return False, 0, b"", b""
        elif self.state == 4:
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.state = 5
            return False, 0, b"", b""
        elif self.state == 5:
            self.checksum = b
            self.state = 0
            calc = (self.chan_id + len(self.payload)) & 0xFF
            for x in self.payload:
                calc = (calc + x) & 0xFF
            if calc == self.checksum:
                self.header_buf = bytearray()
                return True, self.chan_id, bytes(self.payload), b""
            else:
                failed = bytes(self.header_buf) + bytes(self.payload) + bytes([b])
                self.header_buf = bytearray()
                self.payload = bytearray()
                return False, 0, b"", failed
        return False, 0, b"", b""

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

PIN_UART1_TX = 4
PIN_UART1_RX = 5

PIN_JR1_TX = 0
PIN_JR1_RX = 1

PIN_JR1_CRSF_TX = 10
PIN_JR1_CRSF_RX = 11

PIN_JR2_TX = 8
PIN_JR2_RX = 9

PIN_JR1_PWR = 12
PIN_JR2_PWR = 13
PIN_SERVO_AZ = 14
PIN_SERVO_EL = 15

MODE_JR1_ALL = 1
MODE_JR2_CRSF = 2
MODE_SIMULTANEOUS = 3

active_mode = MODE_SIMULTANEOUS
mux_parser = MuxParser()

uart1 = machine.UART(1, baudrate=400000, tx=machine.Pin(PIN_UART1_TX), rx=machine.Pin(PIN_UART1_RX), rxbuf=8192, txbuf=2048)

pwm_pwr1 = machine.PWM(machine.Pin(PIN_JR1_PWR))
pwm_pwr1.freq(50)

pwm_pwr2 = machine.PWM(machine.Pin(PIN_JR2_PWR))
pwm_pwr2.freq(50)

pwm_az = machine.PWM(machine.Pin(PIN_SERVO_AZ))
pwm_az.freq(50)

pwm_el = machine.PWM(machine.Pin(PIN_SERVO_EL))
pwm_el.freq(50)

def set_servo_pwm(pwm_obj, pulse_us):
    duty = int((pulse_us * 65535) / 20000)
    pwm_obj.duty_u16(duty)

set_servo_pwm(pwm_az, 1500)
set_servo_pwm(pwm_el, 1500)

@rp2.asm_pio(out_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT, set_init=rp2.PIO.OUT_HIGH)
def pio_uart_tx():
    pull()
    set(pins, 0)         [6]
    set(x, 7)
    label("bit_loop")
    out(pins, 1)         [6]
    jmp(x_dec, "bit_loop")
    set(pins, 1)         [7]

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, fifo_join=rp2.PIO.JOIN_RX)
def pio_uart_rx():
    label("start")
    wait(0, pin, 0)
    set(x, 7)            [10]
    label("bit_loop")
    in_(pins, 1)         [6]
    jmp(x_dec, "bit_loop")
    push()
    jmp("start")

current_jr1_crsf_baud = 400000
current_jr1_mav_baud = 115200
current_jr2_crsf_baud = 400000

uart0 = None
sm_jr1_tx = None
sm_jr1_rx = None
sm_jr2_tx = None
sm_jr2_rx = None

def init_jr_uarts(jr1_crsf, jr1_mav, jr2_crsf):
    global sm_jr1_tx, sm_jr1_rx, sm_jr2_tx, sm_jr2_rx, uart0

    if sm_jr1_tx is not None: sm_jr1_tx.active(0)
    if sm_jr1_rx is not None: sm_jr1_rx.active(0)
    if sm_jr2_tx is not None: sm_jr2_tx.active(0)
    if sm_jr2_rx is not None: sm_jr2_rx.active(0)

    uart0 = machine.UART(0, baudrate=jr1_mav, tx=machine.Pin(PIN_JR1_TX), rx=machine.Pin(PIN_JR1_RX), rxbuf=8192, txbuf=2048)

    sm_jr1_tx = rp2.StateMachine(0, pio_uart_tx, freq=jr1_crsf * 8, set_base=machine.Pin(PIN_JR1_CRSF_TX), out_base=machine.Pin(PIN_JR1_CRSF_TX))
    sm_jr1_rx = rp2.StateMachine(1, pio_uart_rx, freq=jr1_crsf * 8, in_base=machine.Pin(PIN_JR1_CRSF_RX, machine.Pin.IN, machine.Pin.PULL_UP))
    sm_jr1_tx.active(1)
    sm_jr1_rx.active(1)

    sm_jr2_tx = rp2.StateMachine(2, pio_uart_tx, freq=jr2_crsf * 8, set_base=machine.Pin(PIN_JR2_TX), out_base=machine.Pin(PIN_JR2_TX))
    sm_jr2_rx = rp2.StateMachine(3, pio_uart_rx, freq=jr2_crsf * 8, in_base=machine.Pin(PIN_JR2_RX, machine.Pin.IN, machine.Pin.PULL_UP))
    sm_jr2_tx.active(1)
    sm_jr2_rx.active(1)

init_jr_uarts(current_jr1_crsf_baud, current_jr1_mav_baud, current_jr2_crsf_baud)

def pio_write_jr1(data):
    for b in data:
        sm_jr1_tx.put(b)

def pio_read_jr1():
    buf = bytearray()
    while sm_jr1_rx.rx_fifo():
        val = sm_jr1_rx.get() >> 24
        buf.append(val)
    return bytes(buf) if buf else b""

def pio_write_jr2(data):
    for b in data:
        sm_jr2_tx.put(b)

def pio_read_jr2():
    buf = bytearray()
    while sm_jr2_rx.rx_fifo():
        val = sm_jr2_rx.get() >> 24
        buf.append(val)
    return bytes(buf) if buf else b""

def set_pwm_switch(pwm_obj, on):
    pulse_us = 2000 if on else 1000
    duty = int((pulse_us * 65535) / 20000)
    pwm_obj.duty_u16(duty)

def switcher_set_mode(mode):
    global active_mode
    active_mode = mode
    if mode == MODE_JR1_ALL:
        set_pwm_switch(pwm_pwr1, True)
        set_pwm_switch(pwm_pwr2, False)
    elif mode == MODE_JR2_CRSF:
        set_pwm_switch(pwm_pwr1, False)
        set_pwm_switch(pwm_pwr2, True)
    elif mode == MODE_SIMULTANEOUS:
        set_pwm_switch(pwm_pwr1, True)
        set_pwm_switch(pwm_pwr2, True)

switcher_set_mode(active_mode)

def switcher_process_command(payload):
    global current_jr1_crsf_baud, current_jr1_mav_baud, current_jr2_crsf_baud
    if not payload: return
    cmd = payload[0]
    if cmd == 0x10:
        switcher_set_mode(payload[1])
    elif cmd == 0x70:
        if len(payload) >= 5:
            az_us = (payload[1] << 8) | payload[2]
            el_us = (payload[3] << 8) | payload[4]
            set_servo_pwm(pwm_az, az_us)
            set_servo_pwm(pwm_el, el_us)
    elif cmd == 0x40:
        if len(payload) >= 30:
            b1 = ((payload[24] << 8) | payload[25]) * 100
            b2 = ((payload[26] << 8) | payload[27]) * 100
            b3 = ((payload[28] << 8) | payload[29]) * 100

            if 9600 <= b1 <= 921600 and 9600 <= b2 <= 921600 and 9600 <= b3 <= 921600:
                if b1 != current_jr1_crsf_baud or b2 != current_jr1_mav_baud or b3 != current_jr2_crsf_baud:
                    current_jr1_crsf_baud = b1
                    current_jr1_mav_baud = b2
                    current_jr2_crsf_baud = b3
                    init_jr_uarts(b1, b2, b3)

CHUNK = 256
uart1_rx_buf = bytearray(CHUNK)
uart0_rx_buf = bytearray(CHUNK)

def main():
    last_ping_ms = 0
    while True:
        now = time.ticks_ms()
        activity = False

        if time.ticks_diff(now, last_ping_ms) >= 1000:
            last_ping_ms = now
            try:
                uart1.write(mux_encode(CHAN_CONFIG, bytearray([0x99])))
            except Exception:
                pass

        avail1 = uart1.any()
        if avail1:
            n1 = uart1.readinto(uart1_rx_buf, min(avail1, CHUNK))
            if n1:
                activity = True
                for i in range(n1):
                    b = uart1_rx_buf[i]
                    res = mux_parser.parse_byte(b)
                    if len(res) == 4:
                        success, chan, payload, failed_raw = res
                    else:
                        success, chan, payload = res
                        failed_raw = b""

                    if success:
                        if chan == CHAN_MAVLINK:
                            if active_mode in [MODE_JR1_ALL, MODE_SIMULTANEOUS]:
                                uart0.write(payload)
                        elif chan == CHAN_CRSF:
                            if active_mode == MODE_JR1_ALL:
                                pio_write_jr1(payload)
                            elif active_mode in [MODE_JR2_CRSF, MODE_SIMULTANEOUS]:
                                pio_write_jr2(payload)
                        elif chan == CHAN_CONFIG:
                            switcher_process_command(payload)
                    else:
                        if failed_raw:
                            if active_mode in [MODE_JR1_ALL, MODE_SIMULTANEOUS]:
                                uart0.write(failed_raw)

        if active_mode in [MODE_JR1_ALL, MODE_SIMULTANEOUS]:
            avail0 = uart0.any()
            if avail0:
                n0 = uart0.readinto(uart0_rx_buf, min(avail0, CHUNK))
                if n0:
                    activity = True
                    m_data = memoryview(uart0_rx_buf)[:n0]
                    uart1.write(mux_encode(CHAN_MAVLINK, m_data))

        if active_mode == MODE_JR1_ALL:
            p_data = pio_read_jr1()
            if p_data:
                activity = True
                uart1.write(mux_encode(CHAN_CRSF, p_data))
        elif active_mode in [MODE_JR2_CRSF, MODE_SIMULTANEOUS]:
            p_data = pio_read_jr2()
            if p_data:
                activity = True
                uart1.write(mux_encode(CHAN_CRSF, p_data))

        if not activity:
            time.sleep_ms(1)

if __name__ == '__main__':
    main()
