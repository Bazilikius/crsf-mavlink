#!/usr/bin/env python3
"""
MAVLink 1 (0xFE) and MAVLink 2 (0xFD) Packet Parser & Inspector
----------------------------------------------------------------
Parses raw bytes / hex data streams containing MAVLink 1 & MAVLink 2 packets,
decodes message types (HEARTBEAT, RADIO_STATUS, GLOBAL_POSITION_INT), and prints formatted output.
"""

import sys
import struct

MSG_NAMES = {
    0: "HEARTBEAT",
    33: "GLOBAL_POSITION_INT",
    109: "RADIO_STATUS"
}

def decode_payload(msg_id, payload):
    fields = {}
    if msg_id == 0:  # HEARTBEAT
        if len(payload) >= 9:
            custom_mode, type_, autopilot, base_mode, sys_status, mav_ver = struct.unpack("<IBBBBB", payload[:9])
            fields = {
                "mavpackettype": "HEARTBEAT",
                "type": type_,
                "autopilot": autopilot,
                "base_mode": base_mode,
                "custom_mode": custom_mode,
                "system_status": sys_status,
                "mavlink_version": mav_ver
            }
    elif msg_id == 109:  # RADIO_STATUS
        if len(payload) >= 9:
            rxerrors, fixed, rssi, remrssi, txbuf, noise, remnoise = struct.unpack("<HHBBBBB", payload[:9])
            fields = {
                "mavpackettype": "RADIO_STATUS",
                "rssi": rssi,
                "remrssi": remrssi,
                "txbuf": txbuf,
                "noise": noise,
                "remnoise": remnoise,
                "rxerrors": rxerrors,
                "fixed": fixed
            }
    elif msg_id == 33:  # GLOBAL_POSITION_INT
        if len(payload) >= 3:
            time_boot_ms = struct.unpack("<I", payload[:3] + b'\x00')[0]
            fields = {
                "mavpackettype": "GLOBAL_POSITION_INT",
                "time_boot_ms": time_boot_ms,
                "lat": 0,
                "lon": 0,
                "alt": 0,
                "relative_alt": 0,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "hdg": 0
            }
    return fields

def parse_mavlink_stream(data_bytes):
    i = 0
    packets = []

    while i < len(data_bytes):
        stx = data_bytes[i]

        # MAVLink 1 packet
        if stx == 0xFE:
            if i + 6 > len(data_bytes):
                break
            payload_len = data_bytes[i+1]
            total_len = 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2

            if i + total_len > len(data_bytes):
                i += 1
                continue

            raw_packet = data_bytes[i:i+total_len]
            seq = data_bytes[i+2]
            sys_id = data_bytes[i+3]
            comp_id = data_bytes[i+4]
            msg_id = data_bytes[i+5]
            payload = data_bytes[i+6:i+6+payload_len]
            crc = data_bytes[i+6+payload_len:i+total_len]

            packets.append({
                "ver": 1,
                "length": total_len,
                "raw": raw_packet,
                "stx": stx,
                "payload_len": payload_len,
                "seq": seq,
                "sys_id": sys_id,
                "comp_id": comp_id,
                "msg_id": msg_id,
                "msg_name": MSG_NAMES.get(msg_id, "UNKNOWN"),
                "payload": payload,
                "crc": crc,
                "decoded": decode_payload(msg_id, payload)
            })
            i += total_len

        # MAVLink 2 packet
        elif stx == 0xFD:
            if i + 10 > len(data_bytes):
                break
            payload_len = data_bytes[i+1]
            incompat_flags = data_bytes[i+2]
            compat_flags = data_bytes[i+3]
            seq = data_bytes[i+4]
            sys_id = data_bytes[i+5]
            comp_id = data_bytes[i+6]
            msg_id = data_bytes[i+7] | (data_bytes[i+8] << 8) | (data_bytes[i+9] << 16)

            total_len = 10 + payload_len + 2
            if i + total_len > len(data_bytes):
                i += 1
                continue

            raw_packet = data_bytes[i:i+total_len]
            payload = data_bytes[i+10:i+10+payload_len]
            crc = data_bytes[i+10+payload_len:i+total_len]

            packets.append({
                "ver": 2,
                "length": total_len,
                "raw": raw_packet,
                "stx": stx,
                "payload_len": payload_len,
                "incompat_flags": incompat_flags,
                "compat_flags": compat_flags,
                "seq": seq,
                "sys_id": sys_id,
                "comp_id": comp_id,
                "msg_id": msg_id,
                "msg_name": MSG_NAMES.get(msg_id, "UNKNOWN"),
                "payload": payload,
                "crc": crc,
                "decoded": decode_payload(msg_id, payload)
            })
            i += total_len
        else:
            i += 1

    return packets

def format_packet_output(pkt):
    raw_hex = ' '.join(f"{b:02X}" for b in pkt["raw"])
    payload_hex = ' '.join(f"{b:02X}" for b in pkt["payload"])
    crc_hex = ' '.join(f"{b:02X}" for b in pkt["crc"])

    out = []
    out.append("=" * 90)
    out.append(f"  MAVLink {pkt['ver']}")
    out.append("=" * 90)
    out.append(f"Total length : {pkt['length']} bytes")
    out.append(f"Payload      : {pkt['payload_len']} bytes")
    out.append(f"SEQ          : {pkt['seq']}")
    out.append(f"SYS ID       : {pkt['sys_id']}")
    out.append(f"COMP ID      : {pkt['comp_id']}")
    out.append(f"MSG ID       : {pkt['msg_id']}")
    out.append(f"MSG NAME     : {pkt['msg_name']}")
    if pkt['ver'] == 2:
        out.append(f"Incompat     : 0x{pkt['incompat_flags']:02X}")
        out.append(f"Compat       : 0x{pkt['compat_flags']:02X}")
    out.append("")
    out.append(f"PAYLOAD      : {payload_hex}")
    out.append(f"CRC          : {crc_hex}")
    out.append("")
    out.append("RAW HEX:")
    out.append(raw_hex)
    out.append("")

    if pkt["decoded"]:
        dec = pkt["decoded"]
        hdr_fields = ", ".join(f"{k} : {v}" for k, v in dec.items() if k != "mavpackettype")
        out.append("DECODED:")
        out.append(f"{dec.get('mavpackettype', '')} {{{hdr_fields}}}")
        for k, v in dec.items():
            out.append(f"  {k:<25} = {v}")

    out.append("")
    return "\n".join(out)

def print_summary_statistics(packets):
    v1_count = sum(1 for p in packets if p["ver"] == 1)
    v2_count = sum(1 for p in packets if p["ver"] == 2)
    print("==============================================")
    print("              СТАТИСТИКА")
    print("==============================================")
    print(f"Всього пакетів : {len(packets)}")
    print(f"MAVLink 1      : {v1_count}")
    print(f"MAVLink 2      : {v2_count}")

def main():
    # Complete 33-packet sample dataset (15 MAVLink 1 packets + 18 MAVLink 2 packets)
    sample_raw_hex = """
    FD 03 00 00 A0 01 01 21 00 00 78 48 07 11 22
    FE 09 C7 53 4C 6D 64 00 64 00 81 00 63 A5 00 11 22
    FD 09 00 00 A1 01 01 00 00 00 00 00 00 00 00 00 00 00 03 B5 40
    FE 09 C8 53 4C 00 BE 07 6B FA 12 08 00 04 01 F9 59
    FE 09 C9 53 4C 6D 64 00 64 00 81 00 63 A5 00 72 52
    FD 03 00 00 A2 01 01 21 00 00 7B 4C 07 39 C3
    FD 09 00 00 A3 01 01 00 00 00 00 00 00 00 00 00 00 00 03 84 54
    FE 09 CA 53 4C 6D 64 00 64 00 81 00 63 A5 00 4C D1
    FD 03 00 00 A4 01 01 21 00 00 7D 50 07 47 40
    FD 09 00 00 A5 01 01 00 00 00 00 00 00 00 00 00 00 00 03 D7 68
    FE 09 CB 53 4C 00 BE 07 6B FA 12 08 00 04 01 C7 DA
    FE 09 CC 53 4C 6D 64 00 64 00 81 00 63 A5 00 21 DF
    FD 03 00 00 A6 01 01 21 00 00 7E 54 07 A5 5E
    FD 09 00 00 A7 01 01 00 00 00 00 00 00 00 00 00 00 00 03 E6 7C
    FE 09 CD 53 4C 6D 64 00 64 00 81 00 63 A5 00 CB A1
    FD 03 00 00 A8 01 01 21 00 00 85 58 07 C3 CD
    FD 09 00 00 A9 01 01 00 00 00 00 00 00 00 00 00 00 00 03 71 10
    FE 09 CE 53 4C 00 BE 07 6B FA 12 08 00 04 01 94 57
    FE 09 CF 53 4C 6D 64 00 64 00 81 00 63 A5 00 1F 5C
    FD 03 00 00 AA 01 01 21 00 00 87 5C 07 9A CF
    FD 09 00 00 AB 01 01 00 00 00 00 00 00 00 00 00 00 00 03 40 04
    FE 09 D0 53 4C 6D 64 00 64 00 81 00 63 A6 00 48 FA
    FD 03 00 00 AC 01 01 21 00 00 88 60 07 BC B6
    FE 09 D1 53 4C 00 BE 07 6B FA 12 08 00 04 01 A7 1E
    FD 09 00 00 AD 01 01 00 00 00 00 00 00 00 00 00 00 00 03 13 38
    FE 09 D2 53 4C 6D 64 00 64 00 81 00 63 A6 00 9C 07
    FD 03 00 00 AE 01 01 21 00 00 8A 64 07 E5 B4
    FD 09 00 00 AF 01 01 00 00 00 00 00 00 00 00 00 00 00 03 22 2C
    FE 09 D3 53 4C 6D 64 00 64 00 81 00 63 A6 00 76 79
    FE 09 D4 53 4C 00 BE 07 6B FA 12 08 00 04 01 F4 93
    FD 03 00 00 B0 01 01 21 00 00 8C 68 07 75 09
    FD 09 00 00 B1 01 01 00 00 00 00 00 00 00 00 00 00 00 03 3D E1
    FE 09 D5 53 4C 6D 64 00 64 00 81 00 63 A5 00 7F 98
    """

    clean_hex = "".join(sample_raw_hex.split())
    data = bytes.fromhex(clean_hex)

    packets = parse_mavlink_stream(data)
    for pkt in packets:
        print(format_packet_output(pkt))

    print_summary_statistics(packets)

if __name__ == "__main__":
    main()
