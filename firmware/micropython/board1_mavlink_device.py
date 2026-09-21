"""
Board 1 MicroPython Firmware: MAVLink Device <-> Inter-board Link
------------------------------------------------------------------
Hardware Connections:
- MAVLink Device (e.g., Flight Controller / Radio):
    - RP2040 GPIO 0 (UART0 TX) -> Device RX
    - RP2040 GPIO 1 (UART0 RX) -> Device TX
    - GND -> Device GND
- Inter-board Link (RP2040 Board 1 <-> RP2040 Board 2):
    - RP2040 Board 1 GPIO 4 (UART1 TX) -> RP2040 Board 2 GPIO 5 (UART1 RX)
    - RP2040 Board 1 GPIO 5 (UART1 RX) -> RP2040 Board 2 GPIO 4 (UART1 TX)
    - GND -> GND

FIX NOTES
---------
`UART.readinto(buf)` waits for the whole buffer to fill or for an inter-character
timeout, so a continuous telemetry stream could keep this loop parked in one
direction while the other direction's buffer filled up. We now ask for exactly the
number of bytes that are already buffered (`any()`), so reads return immediately.
"""

from machine import UART, Pin

# Configuration - BAUDRATE must match the MAVLink device (57600 / 115200 / 921600 ...)
BAUDRATE = 115200
BUFFER_SIZE = 4096
CHUNK = 256

uart_device = UART(
    0,
    baudrate=BAUDRATE,
    tx=Pin(0),
    rx=Pin(1),
    txbuf=BUFFER_SIZE,
    rxbuf=BUFFER_SIZE,
)

uart_interboard = UART(
    1,
    baudrate=BAUDRATE,
    tx=Pin(4),
    rx=Pin(5),
    txbuf=BUFFER_SIZE,
    rxbuf=BUFFER_SIZE,
)


def run():
    buf = bytearray(CHUNK)
    mv = memoryview(buf)

    while True:
        # Device (UART0) -> Inter-board (UART1)
        avail = uart_device.any()
        if avail:
            n = uart_device.readinto(buf, min(avail, CHUNK))
            if n:
                uart_interboard.write(mv[:n])

        # Inter-board (UART1) -> Device (UART0)
        avail = uart_interboard.any()
        if avail:
            n = uart_interboard.readinto(buf, min(avail, CHUNK))
            if n:
                uart_device.write(mv[:n])


if __name__ == "__main__":
    run()
