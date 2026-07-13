import unittest
import struct

def calculate_checksum(chan_id, payload):
    cksum = (chan_id + len(payload)) & 0xFF
    for b in payload:
        cksum = (cksum + b) & 0xFF
    return cksum

def encode_mux_frame(chan_id, payload):
    out = bytearray([0xAA, 0x55, chan_id, len(payload)])
    out.extend(payload)
    out.append(calculate_checksum(chan_id, payload))
    return out

class MuxParserPython:
    def __init__(self):
        self.state = 0
        self.chan_id = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0

    def parse_byte(self, b):
        # states: 0=SYNC1, 1=SYNC2, 2=CHAN_ID, 3=LEN, 4=PAYLOAD, 5=CHECKSUM
        if self.state == 0:
            if b == 0xAA:
                self.state = 1
        elif self.state == 1:
            if b == 0x55:
                self.state = 2
            elif b == 0xAA:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 2:
            if b in [0x01, 0x02, 0x03]:
                self.chan_id = b
                self.state = 3
            elif b == 0xAA:
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
            calc = calculate_checksum(self.chan_id, self.payload)
            if calc == self.checksum:
                return True, self.chan_id, bytes(self.payload)
        return False, 0, b""

class TestMuxProtocol(unittest.TestCase):
    def test_encoding_decoding(self):
        parser = MuxParserPython()
        payload = b"MAVLink_Data_123"
        frame = encode_mux_frame(0x02, payload)

        parsed = False
        for b in frame:
            success, chan, out_pay = parser.parse_byte(b)
            if success:
                self.assertEqual(chan, 0x02)
                self.assertEqual(out_pay, payload)
                parsed = True
                break
        self.assertTrue(parsed)

    def test_checksum_failure(self):
        parser = MuxParserPython()
        payload = b"CRSF_Data"
        frame = encode_mux_frame(0x01, payload)
        # Corrupt the payload
        frame[5] ^= 0xFF

        parsed = False
        for b in frame:
            success, chan, out_pay = parser.parse_byte(b)
            if success:
                parsed = True
        self.assertFalse(parsed)

    def test_recovery_from_garbage(self):
        parser = MuxParserPython()
        payload = b"Hello_Mux"
        frame = encode_mux_frame(0x03, payload)

        # Prepend garbage
        stream = bytearray([0x11, 0xAA, 0x22, 0x99, 0xAA, 0x55]) + frame

        results = []
        for b in stream:
            success, chan, out_pay = parser.parse_byte(b)
            if success:
                results.append((chan, out_pay))

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0], (0x03, payload))

if __name__ == '__main__':
    unittest.main()
