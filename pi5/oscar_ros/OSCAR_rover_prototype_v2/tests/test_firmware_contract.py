import unittest
from pathlib import Path
class Firmware(unittest.TestCase):
    def test_motor_output_default_disabled(self):
        src=(Path(__file__).resolve().parents[1]/'firmware/oscar_autonomous.c').read_text()
        self.assertIn('#define OSCAR_ENABLE_MOTOR_OUTPUT 0',src)
        self.assertIn('CMD_TIMEOUT_MS 250u',src)
        self.assertIn('HELLO OSCAR_V1',src)
        self.assertIn('ENC %ld %ld',src)
        self.assertIn('#define MAX_VY 0.00f',src)
    def test_bridge_manual_arm(self):
        src=(Path(__file__).resolve().parents[1]/'oscar_preflight/oscar_preflight/serial_bridge.py').read_text()
        self.assertIn("'/pico/arm'",src)
        self.assertIn("'/pico/estop'",src)
        self.assertIn("'enable_hardware': False",src)
if __name__ == '__main__': unittest.main()
