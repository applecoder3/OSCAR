
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelUartBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_uart_bridge')

        #Set up virtual serial communication
        self.declare_parameter('serial_port', 'loop://')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baudRate = self.get_parameter('baud_rate').value

        #Physical communication stays disabled for testing
        if port != 'loop://':
            raise RuntimeError('Physical UART is disabled')

        self.serialPort = serial.serial_for_url(
            port,
            baudrate=baudRate,
            timeout=0.2,
            write_timeout=0.2
        )

        #Set up movement limits and timeout
        self.maxVx = 0.25
        self.maxVy = 0.25
        self.maxWz = 0.70
        self.timeout = 0.5
        self.lastCommand = None
        self.velocity = (0.0, 0.0, 0.0)
        self.armed = False

        #Subscribe to ROS 2 movement commands
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmdVelCallback, 10
        )

        #Transmit commands every 100 ms
        self.timer = self.create_timer(0.1, self.sendCycle)

        self.get_logger().info('UART bridge ready - virtual serial mode')

    def sendCommand(self, command):
        #Transmit a command through virtual UART
        self.serialPort.write((command + '\n').encode('ascii'))
        self.get_logger().info(f'UART SENT: {command}')

        #Read virtual loopback response
        received = self.serialPort.readline().decode('ascii').strip()

        if received == command:
            self.get_logger().info(f'UART RECEIVED: {received}')
        else:
            self.get_logger().warn('Virtual UART response mismatch')

    def stopMotors(self):
        #Stop and disarm the controller
        if self.armed:
            self.sendCommand('STOP')

        self.armed = False
        self.velocity = (0.0, 0.0, 0.0)
        self.lastCommand = None

    def cmdVelCallback(self, msg):
        #Read requested velocities
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        #Reject invalid values
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.stopMotors()
            self.get_logger().warn('Invalid velocity - STOP')
            return

        #Reject velocities exceeding Pico limits
        if (abs(vx) > self.maxVx or
            abs(vy) > self.maxVy or
            abs(wz) > self.maxWz):
            self.stopMotors()
            self.get_logger().warn('Velocity limit exceeded - STOP')
            return

        self.velocity = (vx, vy, wz)
        self.lastCommand = time.monotonic()

        #Arm when movement is requested
        if not self.armed:
            if all(abs(v) < 0.0001 for v in self.velocity):
                return

            self.sendCommand('ARM')
            self.armed = True

        #Send the first velocity command immediately
        self.sendVelocity()

    def sendVelocity(self):
        #Format the Pico velocity command
        vx, vy, wz = self.velocity
        command = f'VEL {vx:.3f} {vy:.3f} {wz:.3f}'
        self.sendCommand(command)

    def sendCycle(self):
        #Only transmit while armed
        if not self.armed or self.lastCommand is None:
            return

        elapsed = time.monotonic() - self.lastCommand

        #Stop if ROS 2 commands have expired
        if elapsed >= self.timeout:
            self.stopMotors()
            self.get_logger().warn('Command timeout - STOP')
            return

        #Keep Pico watchdog refreshed
        self.sendVelocity()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CmdVelUartBridge()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stopMotors()
            node.serialPort.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
