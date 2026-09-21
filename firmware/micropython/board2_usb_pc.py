"""
Board 2 MicroPython Firmware: Inter-board Link <-> USB CDC (PC)
----------------------------------------------------------------
Hardware Connections:
- Inter-board Link (RP2040 Board 2 <-> RP2040 Board 1):
    - RP2040 Board 2 GPIO 4 (UART1 TX) -> RP2040 Board 1 GPIO 5 (UART1 RX)
    - RP2040 Board 2 GPIO 5 (UART1 RX) -> RP2040 Board 1 GPIO 4 (UART1 TX)
    - GND -> GND
- USB Connection:
    - Micro-USB / USB-C to PC

FIX NOTES
---------
The previous version did `sys.stdin.buffer.read(256)` after poll() reported data.
On MicroPython that call BLOCKS until 256 bytes have arrived from the PC (a GCS
only sends a few dozen bytes per second), which froze the UART -> USB direction.
While frozen, the UART1 RX buffer overflowed and bytes were dropped in the middle
of MAVLink frames, which mavp2p reports as "N errors in the last 5s"
(wrong checksum / invalid magic byte).

Now every stdin read is exactly 1 byte and only happens when poll() says a byte
is ready, so the loop never blocks.
"""

import sys
import select
import micropython
from machine import UART, Pin

# Disable Ctrl+C (KeyboardInterrupt 0x03) on stdin USB CDC so binary MAVLink bytes won't stop the code
micropython.kbd_intr(-1)

# Configuration
BAUDRATE = 115200  # Must match Board 1 and the MAVLink device
BUFFER_SIZE = 4096
CHUNK = 256

uart_interboard = UART(
    1,
    baudrate=BAUDRATE,
    tx=Pin(4),
    rx=Pin(5),
    txbuf=BUFFER_SIZE,
    rxbuf=BUFFER_SIZE,
)


def run():
    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer

    poll = select.poll()
    poll.register(stdin, select.POLLIN)

    rx_buf = bytearray(CHUNK)   # UART -> USB
    tx_buf = bytearray(64)      # USB  -> UART

    while True:
        # UART1 -> PC USB. Read only what is already buffered: never waits.
        avail = uart_interboard.any()
        if avail:
            n = uart_interboard.readinto(rx_buf, min(avail, CHUNK))
            if n:
                stdout.write(memoryview(rx_buf)[:n])

        # PC USB -> UART1. Byte-wise, and only while poll() says data is ready.
        i = 0
        while i < len(tx_buf) and poll.poll(0):
            b = stdin.read(1)
            if not b:
                break
            tx_buf[i] = b[0]
            i += 1
        if i:
            uart_interboard.write(memoryview(tx_buf)[:i])


if __name__ == "__main__":
    run()
