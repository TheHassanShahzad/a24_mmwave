import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class PWMCalculatorNode(Node):
    def __init__(self):
        super().__init__('pwm_calculator_node')

        # Subscription to ang_vel topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/topic_commands_float', 
            self.ang_vel_callback,
            10
        )

        # Publisher to pwm topic
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/pwm_topic',  # Replace with your actual topic name
            10
        )

        self.r_reverse_cof = [0.00040997, 0.17173445, 0.13462213]
        self.r_forward_cof = [-4.32870853e-04,  1.67879421e-01,  2.35361022e+00]
        self.r_reverse_clip = -38
        self.r_forward_clip = 52

        self.l_reverse_cof = [ 5.29843271e-04,  1.85604262e-01, -1.66710749e+00]
        self.l_forward_cof = [-4.25785323e-04,  1.57314982e-01,  4.38386866e+00]
        self.l_reverse_clip = -38
        self.l_forward_clip = 52

    def solve_poly(self, value, basic_cofs, offset):
        a, b, c = basic_cofs
        cofs = [a, (2*a*-offset)+b, a * -offset**2 + b * -offset + c - value] 

        roots = np.roots(cofs)
        closest_root = min(roots, key=abs)
        return closest_root

    def get_pwm(self, ang_vel_r, ang_vel_l):
        if ang_vel_r < 0:
            r_pwm = self.solve_poly(ang_vel_r, self.r_reverse_cof, self.r_reverse_clip)
        else:
            r_pwm = self.solve_poly(ang_vel_r, self.r_forward_cof, self.r_forward_clip)
        if ang_vel_l < 0:
            l_pwm = self.solve_poly(ang_vel_l, self.l_reverse_cof, self.l_reverse_clip)
        else:
            l_pwm = self.solve_poly(ang_vel_l, self.l_forward_cof, self.l_forward_clip)

        if r_pwm <= self.r_forward_clip and r_pwm >= self.r_reverse_clip:
            r_pwm = 0

        if l_pwm <= self.l_forward_clip and l_pwm >= self.l_reverse_clip:
            l_pwm = 0

        return [float(r_pwm), float(l_pwm)]
   
    def ang_vel_callback(self, msg):

        if len(msg.data) < 2:
            self.get_logger().warn("Received insufficient data. Expected 2 values: [ang_vel_r, ang_vel_l].")
            return

        # Extract angular velocities
        ang_vel_r = msg.data[0]
        ang_vel_l = msg.data[1]

        # self.get_logger().info(f"Received ang_vel_r: {ang_vel_r}, ang_vel_l: {ang_vel_l}")

        # Prepare output message
        output_msg = Float64MultiArray()
        output_msg.data = self.get_pwm(ang_vel_r, ang_vel_l)

        self.publisher.publish(output_msg)
        self.get_logger().info(f"Published pwm_r: {output_msg.data[0]}, pwm_l: {output_msg.data[1]}")


def main(args=None):
    rclpy.init(args=args)

    node = PWMCalculatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
