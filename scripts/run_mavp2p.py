#!/usr/bin/env python3
"""
MAVLink Bridge & mavp2p Background Runner (Windows & Linux / macOS)
------------------------------------------------------------------
This script launches mavp2p in the background to forward MAVLink messages between RP2040 (USB CDC)
and UDP ports 19415 & 14556.

Features:
- Supports Windows (COM ports) and Linux/macOS (/dev/ttyACM* /dev/ttyUSB*).
- Uses udps:127.0.0.1:19415 and udpc:127.0.0.1:14556 endpoints to prevent Windows socket bind errors.
- Auto-detects serial ports or allows interactive manual port and baud rate selection.
- Auto-detects mavp2p executable (mavp2p or mavp2p.exe).
"""

import os
import sys
import glob
import shutil
import argparse
import subprocess

def list_available_ports():
    """Returns a list of candidate serial ports on Windows and Linux/Mac."""
    ports = []

    if sys.platform.startswith('win'):
        try:
            import serial.tools.list_ports
            ports = [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            for i in range(1, 65):
                ports.append(f"COM{i}")
    else:
        patterns = [
            "/dev/ttyACM*",
            "/dev/ttyUSB*",
            "/dev/tty.usbmodem*",
            "/dev/tty.usbserial*"
        ]
        for pattern in patterns:
            ports.extend(glob.glob(pattern))

    return ports

def get_mavp2p_executable(custom_path=None):
    """Finds mavp2p or mavp2p.exe binary path."""
    if custom_path and os.path.exists(custom_path):
        return custom_path

    exec_names = ["mavp2p.exe", "mavp2p"] if sys.platform.startswith('win') else ["mavp2p", "mavp2p.exe"]

    for name in exec_names:
        found = shutil.which(name)
        if found:
            return found
        if os.path.exists(name):
            return os.path.abspath(name)

    return exec_names[0]

def main():
    parser = argparse.ArgumentParser(description="Launch mavp2p background forwarder for RP2040 USB MAVLink bridge (Windows/Linux/Mac).")
    parser.add_argument("--port", help="Serial port of RP2040 (e.g., COM80 on Windows, /dev/ttyACM0 on Linux).")
    parser.add_argument("--baud", help="Serial baud rate (default: 115200).")
    parser.add_argument("--udp-port1", default="19415", help="UDP server port (default: 19415).")
    parser.add_argument("--udp-port2", default="14556", help="UDP client port (default: 14556).")
    parser.add_argument("--mavp2p-bin", help="Path to mavp2p binary (default: auto-detected mavp2p/mavp2p.exe).")
    parser.add_argument("--print-errors", action="store_true",
                        help="Print each parse error (checksum, bad magic byte, ...) instead of only a count every 5s.")
    parser.add_argument("--serial-only", action="store_true",
                        help="Diagnostics: run with ONLY the serial endpoint (no UDP). If errors persist, the problem "
                             "is the serial/firmware path; if they vanish, it is on the UDP side.")

    args = parser.parse_args()

    port = args.port
    if not port:
        available_ports = list_available_ports()
        if available_ports and not sys.platform.startswith('win'):
            port = available_ports[0]
            print(f"Auto-detected serial port: {port}")
        else:
            if available_ports:
                print(f"Found available serial ports: {', '.join(available_ports)}")
            print("\n--- Manual Serial Port Selection ---")
            user_input_port = input("Enter RP2040 COM port (e.g. COM80 or /dev/ttyACM0): ").strip()
            if user_input_port:
                port = user_input_port
            else:
                print("Error: Port is required!")
                sys.exit(1)

    baud = args.baud
    if not baud:
        user_input_baud = input("Enter baud rate [default 115200]: ").strip()
        baud = user_input_baud if user_input_baud else "115200"

    mavp2p_bin = get_mavp2p_executable(args.mavp2p_bin)

    print("\n--------------------------------------------------")
    print(f"  RP2040 Port : {port}")
    print(f"  Baud Rate   : {baud}")
    print(f"  UDP Server  : udps:127.0.0.1:{args.udp_port1}")
    print(f"  UDP Client  : udpc:127.0.0.1:{args.udp_port2}")
    print(f"  mavp2p Bin  : {mavp2p_bin}")
    print("--------------------------------------------------\n")

    serial_endpoint = f"serial:{port}:{baud}"
    udp1_endpoint = f"udps:127.0.0.1:{args.udp_port1}"
    udp2_endpoint = f"udpc:127.0.0.1:{args.udp_port2}"

    cmd = [mavp2p_bin]
    if args.print_errors:
        cmd.append("--print-errors")
    if args.serial_only:
        cmd.append(serial_endpoint)
    else:
        cmd.extend([udp1_endpoint, udp2_endpoint, serial_endpoint])

    print(f"Executing: {' '.join(cmd)}\n")

    try:
        proc = subprocess.Popen(cmd)
        print(f"mavp2p started with PID {proc.pid}. Press Ctrl+C to stop.\n")
        try:
            sys.exit(proc.wait())
        except KeyboardInterrupt:
            proc.terminate()
            proc.wait()
    except FileNotFoundError:
        print(f"Error: '{mavp2p_bin}' binary not found.")
        print("Please download mavp2p from https://github.com/bluenviron/mavp2p/releases")
        sys.exit(1)

if __name__ == "__main__":
    main()
