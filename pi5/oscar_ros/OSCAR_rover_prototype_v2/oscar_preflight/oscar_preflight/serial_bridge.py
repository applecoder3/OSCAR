"""Versioned Pico serial bridge: heartbeat, encoder telemetry, manual arming.

Never auto-arms from /cmd_vel. Does not speak Ethan's old keystroke protocol.
Safety: physical serial requires enable_hardware=true AND /pico/arm true, with
fresh firmware HELLO, heartbeat, scan + odometry (if enabled), no E-stop.
"""
import math
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class SerialBridge(Node):
    def __init__(self):
        super().__init__('oscar_serial_bridge')
        for key, default in {
            'port': '/dev/serial0', 'baud': 115200, 'enable_hardware': False,
            'enable_scan_guard': True, 'enable_odom_guard': True,
            'max_linear_x': 0.15, 'max_angular_z': 0.4,
            'max_command_age_s': 0.35, 'max_sensor_age_s': 0.75,
            'serial_timeout_s': 0.03, 'firmware_hello_timeout_s': 1.5,
            'allow_strafe': False,
        }.items(): self.declare_parameter(key, default)
        self.ser = None
        self.last_cmd = None
        self.last_scan = self.last_odom = None
        self.last_hello = None
        self.cmd = Twist()
        self.armed = False
        self.estop = False
        self.rx_buf = b''
        self.encoder_pub = self.create_publisher(String, '/pico/telemetry_raw', 100)
        self.status_pub = self.create_publisher(String, '/pico/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/pico/arm', self.on_arm, 10)
        self.create_subscription(Bool, '/pico/estop', self.on_estop, 10)
        self.create_subscription(LaserScan, '/scan', lambda _: self.on_sensor('scan'), 10)
        self.create_subscription(Odometry, '/odom', lambda _: self.on_sensor('odom'), 10)
        self.create_timer(0.05, self.update)
        if self.get_parameter('enable_hardware').value:
            try:
                import serial
                self.ser = serial.Serial(self.get_parameter('port').value,
                                         int(self.get_parameter('baud').value), timeout=0,
                                         write_timeout=0.03)
                self.get_logger().warn('Physical serial opened but DISARMED: requires firmware OSCAR_V1 + explicit approval')
            except Exception as exc:
                self.get_logger().error(f'Cannot open serial port: {exc}')
        else:
            self.get_logger().warn('Hardware serial disabled; no movement commands can be sent')

    def on_sensor(self, name):
        setattr(self, 'last_' + name, time.monotonic())

    def on_cmd(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(v) for v in values):
            self.last_cmd = None
            return
        self.cmd = msg
        self.last_cmd = time.monotonic()

    def on_arm(self, msg):
        if not msg.data:
            self.armed = False
            self.send('DISARM')
        elif not self.estop and self.ser is not None and self.firmware_ready():
            self.armed = True
            self.send('ARM')
        else:
            self.get_logger().warn('Arm rejected: firmware handshake not ready, hardware disabled, or E-stop active')

    def on_estop(self, msg):
        if msg.data:
            self.estop = True
            self.armed = False
            self.send('DISARM')
        # E-stop is latched for process lifetime, requires a restart to reset.

    def send(self, line):
        if self.ser is None: return
        try:
            self.ser.write((line + '\n').encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self.armed = False

    def read_serial(self):
        if self.ser is None: return
        try:
            chunk = self.ser.read(min(256, max(0, self.ser.in_waiting)))
        except Exception as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            self.armed = False
            return
        self.rx_buf += chunk
        if len(self.rx_buf) > 4096:
            self.rx_buf = b''
        for _ in range(20):
            if b'\n' not in self.rx_buf: break
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            line = raw.decode('ascii', errors='ignore').strip()
            if line == 'HELLO OSCAR_V1':
                self.last_hello = time.monotonic()
            if line.startswith('ENC '):
                m = String(); m.data = line; self.encoder_pub.publish(m)

    def firmware_ready(self):
        return self.last_hello is not None and (time.monotonic()-self.last_hello) < float(self.get_parameter('firmware_hello_timeout_s').value)

    def update(self):
        self.read_serial()
        now = time.monotonic()
        ready = self.firmware_ready()
        if self.ser is not None and not ready:
            self.armed = False
            self.send('DISARM')
        cmd_ok = self.last_cmd is not None and now - self.last_cmd < float(self.get_parameter('max_command_age_s').value)
        scan_ok = not self.get_parameter('enable_scan_guard').value or (self.last_scan is not None and now-self.last_scan < float(self.get_parameter('max_sensor_age_s').value))
        odom_ok = not self.get_parameter('enable_odom_guard').value or (self.last_odom is not None and now-self.last_odom < float(self.get_parameter('max_sensor_age_s').value))
        drive_ok = bool(self.armed and not self.estop and ready and cmd_ok and scan_ok and odom_ok)
        if drive_ok:
            vx = max(-float(self.get_parameter('max_linear_x').value), min(float(self.get_parameter('max_linear_x').value), self.cmd.linear.x))
            wz = max(-float(self.get_parameter('max_angular_z').value), min(float(self.get_parameter('max_angular_z').value), self.cmd.angular.z))
            vy = 0.0  # Intentionally restricted 2-encoder navigation
            if self.get_parameter('allow_strafe').value:
                self.get_logger().error('Strafe blocked until full-state estimator is implemented')
            self.send(f'VEL {vx:+.4f} {vy:+.4f} {wz:+.4f}')
        else:
            if self.armed:
                self.armed = False
                self.send('DISARM')
        m = String(); m.data = f'firmware_ready={ready};armed={self.armed};drive_ok={drive_ok};scan_ok={scan_ok};odom_ok={odom_ok};estop={self.estop}'
        self.status_pub.publish(m)

    def destroy_node(self):
        self.send('DISARM')
        if self.ser is not None: self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
