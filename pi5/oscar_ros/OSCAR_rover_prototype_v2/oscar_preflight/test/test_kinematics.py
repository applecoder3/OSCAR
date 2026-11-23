import unittest
import math
from oscar_preflight.kinematics import meters_per_tick, twist_to_wheels, parse_encoder_line
from oscar_preflight.target_math import camera_bearing, laser_range_at_bearing, approach_point

class KinematicTests(unittest.TestCase):
    def test_tick(self): self.assertAlmostEqual(meters_per_tick(), math.pi*0.08/2464)
    def test_forward(self): self.assertEqual(twist_to_wheels(.1,0,0,.2), (.1,.1,.1,.1))
    def test_strafe(self): self.assertEqual(twist_to_wheels(0,.1,0,.2), (-.1,.1,.1,-.1))
    def test_parse(self):
        self.assertEqual(parse_encoder_line('ENC -123 456'),(-123,456))
        self.assertIsNone(parse_encoder_line('RL +50 mm/s'))
    def test_bearing(self): self.assertAlmostEqual(camera_bearing(.5,math.radians(62)),0)
    def test_scan(self):
        self.assertEqual(laser_range_at_bearing([float('nan'),.4,.5],-.1,.1,0,0,.05,1),.4)
    def test_standoff(self): self.assertAlmostEqual(approach_point(0,0,0,0,.6,.4)[0],.2)

if __name__ == '__main__': unittest.main()
