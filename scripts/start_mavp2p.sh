#!/usr/bin/env bash
# Script to launch mavp2p in background for RP2040 MAVLink bridge

BAUD=${BAUD:-115200}
PORT1=${PORT1:-19415}
PORT2=${PORT2:-14556}

# Auto-detect serial port if not provided
if [ -z "$SERIAL_PORT" ]; then
    SERIAL_PORT=$(ls /dev/ttyACM* /dev/ttyUSB* /dev/tty.usbmodem* 2>/dev/null | head -n 1)
fi

if [ -z "$SERIAL_PORT" ]; then
    echo "Error: Could not automatically detect RP2040 serial port."
    echo "Usage: SERIAL_PORT=/dev/ttyACM0 ./start_mavp2p.sh"
    exit 1
fi

echo "Connecting mavp2p to $SERIAL_PORT at $BAUD baud..."
echo "Forwarding MAVLink to UDP 127.0.0.1:$PORT1 & 127.0.0.1:$PORT2 in background..."

# Optional diagnostics:
#   PRINT_ERRORS=1  -> print every parse error instead of just a count every 5s
#   SERIAL_ONLY=1   -> only the serial endpoint (isolates firmware/serial issues from UDP issues)
ARGS=()
[ -n "$PRINT_ERRORS" ] && ARGS+=(--print-errors)
if [ -n "$SERIAL_ONLY" ]; then
    ARGS+=("serial:${SERIAL_PORT}:${BAUD}")
else
    ARGS+=("udps:127.0.0.1:${PORT1}" "udpc:127.0.0.1:${PORT2}" "serial:${SERIAL_PORT}:${BAUD}")
fi

mavp2p "${ARGS[@]}" &
MAVP2P_PID=$!

echo "mavp2p started in background with PID $MAVP2P_PID."
echo "To stop: kill $MAVP2P_PID"
