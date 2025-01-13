import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Float64MultiArrayProcessor(Node):
    def __init__(self):
        super().__init__('float64_multiarray_processor')

        # Subscriber to input topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/topic_commands_float',  # Change this to your input topic name
            self.listener_callback,
            10
        )

        # Publisher to output topic
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'pwm_topic',  # Change this to your output topic name
            10
        )


        self.max_r_rev, self.max_l_rev, self.max_r_forward, self.max_l_forward = [18.78, 18.41, 19.55, 19.80]

        self.get_logger().info('Node initialized. Listening to input_topic and publishing to output_topic.')

    def calculate_pwm(self, ang_vel, max_rev, max_forward):
        if ang_vel <= 0:
            pwm_uncon = ang_vel / (max_rev / 250)
        else:
            pwm_uncon = ang_vel / (max_forward / 250)
        return float(max(-255, min(255, pwm_uncon)))
    

    def listener_callback(self, msg):

        r_ang_vel = msg.data[0]
        l_ang_vel = msg.data[1]
        
        r_pwm = self.calculate_pwm(r_ang_vel, self.max_r_rev, self.max_r_forward)
        l_pwm = self.calculate_pwm(l_ang_vel, self.max_l_rev, self.max_l_forward)
        
        pwm_data = [r_pwm, l_pwm]

        self.get_logger().info(f'Calculated PWM - Right: {r_pwm}, Left: {l_pwm}')

        new_msg = Float64MultiArray()
        new_msg.data = pwm_data

        # Publish the processed data
        self.publisher.publish(new_msg)
        self.get_logger().info(f'Published: {new_msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = Float64MultiArrayProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
