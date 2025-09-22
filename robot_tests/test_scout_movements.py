#!/usr/bin/env python3
"""
Comprehensive Scout Mini Movement Test Script
Tests all movement capabilities with safety features
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class ScoutMovementTester(Node):
    def __init__(self):
        super().__init__('scout_movement_tester')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Scout Movement Tester Initialized')

        # Robot speed limits
        self.MAX_LINEAR_SPEED = 1.5  # m/s (Scout Mini max)
        self.MAX_ANGULAR_SPEED = 2.0  # rad/s

        # Test speeds (conservative for safety)
        self.TEST_LINEAR_SPEED = 0.3   # m/s
        self.TEST_ANGULAR_SPEED = 0.5  # rad/s
        self.TEST_DURATION = 2.0       # seconds per test

    def send_cmd(self, linear_x=0.0, angular_z=0.0, duration=None):
        """Send velocity command to robot"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

        if duration:
            time.sleep(duration)
            self.stop()

    def stop(self):
        """Emergency stop - send zero velocities"""
        self.send_cmd(0.0, 0.0)
        self.get_logger().info('STOP')
        time.sleep(0.5)

    def test_forward(self):
        """Test forward movement"""
        self.get_logger().info('Testing FORWARD movement...')
        self.send_cmd(self.TEST_LINEAR_SPEED, 0.0, self.TEST_DURATION)

    def test_backward(self):
        """Test backward movement"""
        self.get_logger().info('Testing BACKWARD movement...')
        self.send_cmd(-self.TEST_LINEAR_SPEED, 0.0, self.TEST_DURATION)

    def test_rotate_left(self):
        """Test counter-clockwise rotation"""
        self.get_logger().info('Testing LEFT ROTATION (CCW)...')
        self.send_cmd(0.0, self.TEST_ANGULAR_SPEED, self.TEST_DURATION)

    def test_rotate_right(self):
        """Test clockwise rotation"""
        self.get_logger().info('Testing RIGHT ROTATION (CW)...')
        self.send_cmd(0.0, -self.TEST_ANGULAR_SPEED, self.TEST_DURATION)

    def test_arc_left(self):
        """Test forward motion with left turn"""
        self.get_logger().info('Testing ARC LEFT (forward + left turn)...')
        self.send_cmd(self.TEST_LINEAR_SPEED, self.TEST_ANGULAR_SPEED/2, self.TEST_DURATION)

    def test_arc_right(self):
        """Test forward motion with right turn"""
        self.get_logger().info('Testing ARC RIGHT (forward + right turn)...')
        self.send_cmd(self.TEST_LINEAR_SPEED, -self.TEST_ANGULAR_SPEED/2, self.TEST_DURATION)

    def test_speed_variations(self):
        """Test different speed levels"""
        self.get_logger().info('Testing SPEED VARIATIONS...')

        speeds = [0.1, 0.2, 0.3, 0.4, 0.5]  # Progressive speeds
        for speed in speeds:
            self.get_logger().info(f'  Speed: {speed} m/s')
            self.send_cmd(speed, 0.0, 1.0)

        self.stop()

    def test_acceleration(self):
        """Test smooth acceleration and deceleration"""
        self.get_logger().info('Testing ACCELERATION ramp...')

        # Ramp up
        for i in range(10):
            speed = (i / 10.0) * self.TEST_LINEAR_SPEED
            self.send_cmd(speed, 0.0)
            time.sleep(0.1)

        # Hold max speed
        time.sleep(1.0)

        # Ramp down
        for i in range(10, 0, -1):
            speed = (i / 10.0) * self.TEST_LINEAR_SPEED
            self.send_cmd(speed, 0.0)
            time.sleep(0.1)

        self.stop()

    def run_all_tests(self):
        """Execute all movement tests in sequence"""
        self.get_logger().info('='*50)
        self.get_logger().info('SCOUT MINI MOVEMENT TEST SUITE')
        self.get_logger().info('='*50)
        self.get_logger().info('⚠️  WARNING: Robot will move! Ensure clear space!')
        self.get_logger().info('Press Ctrl+C anytime for emergency stop')
        self.get_logger().info('='*50)

        # Wait for user confirmation
        input('Press Enter to start tests...')

        tests = [
            ('Forward Movement', self.test_forward),
            ('Backward Movement', self.test_backward),
            ('Left Rotation', self.test_rotate_left),
            ('Right Rotation', self.test_rotate_right),
            ('Arc Left', self.test_arc_left),
            ('Arc Right', self.test_arc_right),
            ('Speed Variations', self.test_speed_variations),
            ('Acceleration', self.test_acceleration)
        ]

        for i, (name, test_func) in enumerate(tests, 1):
            self.get_logger().info(f'\n[{i}/{len(tests)}] {name}')
            self.get_logger().info('-' * 30)

            try:
                test_func()
                self.get_logger().info('✅ Test completed')
            except Exception as e:
                self.get_logger().error(f'❌ Test failed: {e}')
                self.stop()

            # Pause between tests
            if i < len(tests):
                self.get_logger().info('Pausing 2 seconds before next test...')
                time.sleep(2.0)

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('ALL TESTS COMPLETED!')
        self.get_logger().info('='*50)
        self.stop()

    def run_interactive(self):
        """Interactive control mode"""
        self.get_logger().info('INTERACTIVE CONTROL MODE')
        self.get_logger().info('Commands:')
        self.get_logger().info('  w/s - forward/backward')
        self.get_logger().info('  a/d - rotate left/right')
        self.get_logger().info('  q/e - arc left/right')
        self.get_logger().info('  space - stop')
        self.get_logger().info('  x - exit')

        while True:
            cmd = input('Command: ').lower().strip()

            if cmd == 'w':
                self.send_cmd(self.TEST_LINEAR_SPEED, 0.0, 1.0)
            elif cmd == 's':
                self.send_cmd(-self.TEST_LINEAR_SPEED, 0.0, 1.0)
            elif cmd == 'a':
                self.send_cmd(0.0, self.TEST_ANGULAR_SPEED, 1.0)
            elif cmd == 'd':
                self.send_cmd(0.0, -self.TEST_ANGULAR_SPEED, 1.0)
            elif cmd == 'q':
                self.send_cmd(self.TEST_LINEAR_SPEED, self.TEST_ANGULAR_SPEED/2, 1.0)
            elif cmd == 'e':
                self.send_cmd(self.TEST_LINEAR_SPEED, -self.TEST_ANGULAR_SPEED/2, 1.0)
            elif cmd == ' ':
                self.stop()
            elif cmd == 'x':
                break
            else:
                self.get_logger().info('Unknown command')

def main(args=None):
    rclpy.init(args=args)

    tester = ScoutMovementTester()

    try:
        # Check command line arguments
        if len(sys.argv) > 1 and sys.argv[1] == '--interactive':
            tester.run_interactive()
        else:
            tester.run_all_tests()

    except KeyboardInterrupt:
        tester.get_logger().info('\n⚠️  EMERGENCY STOP - User interrupted')
        tester.stop()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()