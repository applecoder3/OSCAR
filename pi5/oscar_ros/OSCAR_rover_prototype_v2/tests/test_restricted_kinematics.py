import math
import unittest
from oscar_preflight.kinematics import meters_per_tick, twist_to_wheels, parse_encoder_line

class TwoRearEncoders(unittest.TestCase):
    def test_one_rev(self):
        self.assertAlmostEqual(meters_per_tick()*2464, math.pi*0.08, places=9)

    def test_straight_wheels(self):
        self.assertEqual(twist_to_wheels(0.1,0.0,0.0,0.2), (0.1,0.1,0.1,0.1))

    def test_rotation_wheels(self):
        w=twist_to_wheels(0,0,0.3,0.2)
        self.assertAlmostEqual(w[2], -0.06)
        self.assertAlmostEqual(w[3], 0.06)

    def test_two_rear_sensors_cannot_separate_yaw_and_strafe(self):
        # Rear left = vx+vy - k*wz, rear right = vx-vy + k*wz.
        # Pure turn at rate 0.2 (k=.2) has same rear readings as lateral
        # strafe vy=-0.04 m/s at zero yaw. Hence unobservable without vy=0.
        turning=twist_to_wheels(0.0,0.0,0.2,0.2)[2:4]
        strafing=twist_to_wheels(0.0,-0.04,0.0,0.2)[2:4]
        for a,b in zip(turning,strafing): self.assertAlmostEqual(a,b)

    def test_encoder_line(self):
        self.assertEqual(parse_encoder_line('ENC -123 456'),(-123,456))
        self.assertIsNone(parse_encoder_line('RL: count=123 speed=14'))

if __name__ == '__main__': unittest.main()
