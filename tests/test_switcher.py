import unittest

class RFSwitcherMock:
    def __init__(self):
        self.mode = 3 # Default mode: Simultaneous
        self.jr1_pwr = True
        self.jr2_pwr = True
        self.forwarded = []

    def set_mode(self, mode):
        self.mode = mode
        if mode == 1: # JR1 only (CRSF + MAVLink)
            self.jr1_pwr = True
            self.jr2_pwr = False
        elif mode == 2: # JR2 only (CRSF)
            self.jr1_pwr = False
            self.jr2_pwr = True
        elif mode == 3: # Simultaneous (JR1 MAVLink, JR2 CRSF)
            self.jr1_pwr = True
            self.jr2_pwr = True

    def process_command(self, payload):
        if not payload:
            return
        cmd_type = payload[0]
        if cmd_type == 0x10:
            self.set_mode(payload[1])

    def route_incoming_packet(self, chan_id, payload):
        # Data from Board 1 (Ground) to JR Modules
        if chan_id == 0x02: # MAVLink
            if self.mode in [1, 3]:
                self.forwarded.append(("JR1_UART0", payload))
        elif chan_id == 0x01: # CRSF
            if self.mode == 1:
                self.forwarded.append(("JR1_UART0", payload))
            elif self.mode in [2, 3]:
                self.forwarded.append(("JR2_PIO", payload))

class TestSwitcherStateMachine(unittest.TestCase):
    def test_mode_transitions(self):
        switcher = RFSwitcherMock()

        # Test Mode 3 (Simultaneous)
        self.assertEqual(switcher.mode, 3)
        self.assertTrue(switcher.jr1_pwr)
        self.assertTrue(switcher.jr2_pwr)

        # Switch to Mode 1 (JR1 only)
        switcher.process_command([0x10, 1])
        self.assertEqual(switcher.mode, 1)
        self.assertTrue(switcher.jr1_pwr)
        self.assertFalse(switcher.jr2_pwr)

        # Switch to Mode 2 (JR2 only)
        switcher.process_command([0x10, 2])
        self.assertEqual(switcher.mode, 2)
        self.assertFalse(switcher.jr1_pwr)
        self.assertTrue(switcher.jr2_pwr)

    def test_routing_mode1(self):
        switcher = RFSwitcherMock()
        switcher.set_mode(1) # JR1 All

        # Both MAVLink and CRSF should go to JR1 (UART0)
        switcher.route_incoming_packet(0x02, b"mavlink_msg")
        switcher.route_incoming_packet(0x01, b"crsf_msg")

        self.assertEqual(len(switcher.forwarded), 2)
        self.assertEqual(switcher.forwarded[0], ("JR1_UART0", b"mavlink_msg"))
        self.assertEqual(switcher.forwarded[1], ("JR1_UART0", b"crsf_msg"))

    def test_routing_mode3(self):
        switcher = RFSwitcherMock()
        switcher.set_mode(3) # Simultaneous

        # MAVLink to JR1, CRSF to JR2
        switcher.route_incoming_packet(0x02, b"mavlink_msg")
        switcher.route_incoming_packet(0x01, b"crsf_msg")

        self.assertEqual(len(switcher.forwarded), 2)
        self.assertEqual(switcher.forwarded[0], ("JR1_UART0", b"mavlink_msg"))
        self.assertEqual(switcher.forwarded[1], ("JR2_PIO", b"crsf_msg"))

if __name__ == '__main__':
    unittest.main()
