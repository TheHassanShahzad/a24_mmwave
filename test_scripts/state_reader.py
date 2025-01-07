#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StateFeedbackPrinter(Node):
    def __init__(self):
        super().__init__('state_feedback_printer')

        # Subscriber to /state_feedback
        self.sub_feedback = self.create_subscription(
            Float64MultiArray,
            'state_feedback',
            self.feedback_callback,
            10
        )

    def feedback_callback(self, msg):
        """
        Callback function to handle received state feedback messages.
        Prints elements 1 (right wheel velocity) and 3 (left wheel velocity).
        """
        if len(msg.data) >= 4:
            right_wheel_velocity = msg.data[1]
            left_wheel_velocity = msg.data[3]

            self.get_logger().info(
                f"Right Wheel Velocity: {right_wheel_velocity:.3f}, "
                f"Left Wheel Velocity: {left_wheel_velocity:.3f}"
            )
        else:
            self.get_logger().warn("Received /state_feedback message with insufficient data!")


def main(args=None):
    rclpy.init(args=args)

    node = StateFeedbackPrinter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
