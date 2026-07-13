# shared/mux_protocol.py

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
        # Returns (success, chan_id, payload)
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
            calc = calculate_checksum(self.chan_id, self.payload)
            if calc == self.checksum:
                return True, self.chan_id, bytes(self.payload)
        return False, 0, b""

def calculate_checksum(chan_id, payload):
    cksum = (chan_id + len(payload)) & 0xFF
    for b in payload:
        cksum = (cksum + b) & 0xFF
    return cksum

def mux_encode(chan_id, payload):
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    out = bytearray([SYNC1, SYNC2, chan_id, len(payload)])
    out.extend(payload)
    out.append(calculate_checksum(chan_id, payload))
    return bytes(out)
