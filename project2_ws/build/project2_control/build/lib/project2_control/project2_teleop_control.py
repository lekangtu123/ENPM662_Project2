#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios

LIN_VEL_STEP_SIZE = 2.0

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """
        self.get_logger().info(self.msg)

        key_left_vel = 0.0
        key_right_vel = 0.0

        while True:
            key = self.getKey()
            if key == '\x1b':  # Escape key
                break
            elif key == 'q':  # Quit
                key_left_vel = 0.0
                key_right_vel = 0.0
            elif key == 'w':  # Forward 
                key_left_vel += LIN_VEL_STEP_SIZE
                key_right_vel += LIN_VEL_STEP_SIZE
            elif key == 's':   # Reverse
                key_left_vel -= LIN_VEL_STEP_SIZE
                key_right_vel -= LIN_VEL_STEP_SIZE
            elif key == 'd':  # Right
                key_left_vel += LIN_VEL_STEP_SIZE * 5000000  # Increase left wheel speed
                key_right_vel -= LIN_VEL_STEP_SIZE * 5000000 # Decrease right wheel speed
            elif key == 'a':  # Left
                key_left_vel -= LIN_VEL_STEP_SIZE * 5000000  # Decrease left wheel speed
                key_right_vel += LIN_VEL_STEP_SIZE * 5000000 # Increase right wheel speed

            # Cap velocities to a certain range
            key_left_vel = max(-5.0, min(key_left_vel, 5.0))
            key_right_vel = max(-5.0, min(key_right_vel, 5.0))

            wheel_velocities = Float64MultiArray()
            wheel_velocities.data = [key_left_vel, key_right_vel, key_left_vel, key_right_vel]
            self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
