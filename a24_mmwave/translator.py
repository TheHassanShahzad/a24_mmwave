import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class WheelStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_state_publisher')
        
        # Subscriber to /topic_commands_js
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/topic_commands_js',
            self.joint_states_callback,
            10
        )

        # Subscriber to /topic_states_float
        self.float_states_subscriber = self.create_subscription(
            Float64MultiArray,
            '/topic_states_float',
            self.float_states_callback,
            10
        )

        # Publisher for /topic_commands_float
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/topic_commands_float', 10)

        # Publisher for /topic_states_js
        self.states_publisher = self.create_publisher(JointState, '/topic_states_js', 10)

        self.get_logger().info('Wheel State Publisher node has started.')

    def joint_states_callback(self, msg):
        # Initialize velocities for right and left wheels
        right_wheel_velocity = 0.0
        left_wheel_velocity = 0.0

        try:
            # Find indices of the desired joints in the name list
            right_wheel_index = msg.name.index('right_wheel_joint')
            left_wheel_index = msg.name.index('left_wheel_joint')

            # Extract velocities using the indices
            right_wheel_velocity = msg.velocity[right_wheel_index]
            left_wheel_velocity = msg.velocity[left_wheel_index]

        except ValueError as e:
            self.get_logger().warn(f"Wheel names not found in joint_states message: {e}")
        except IndexError as e:
            self.get_logger().warn(f"Velocity data not available for the wheels: {e}")

        # Publish the velocities as Float64MultiArray
        velocity_array = Float64MultiArray()
        velocity_array.data = [right_wheel_velocity, left_wheel_velocity]
        
        self.velocity_publisher.publish(velocity_array)
        self.get_logger().debug(f"Published wheel velocities: {velocity_array.data}")

    def float_states_callback(self, msg):
        # Check if the message has the required number of elements
        if len(msg.data) < 4:
            self.get_logger().warn("Received Float64MultiArray with insufficient data.")
            return

        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['right_wheel_joint', 'left_wheel_joint']

        # Extract positions and velocities
        right_wheel_position = msg.data[0]
        right_wheel_velocity = msg.data[1]
        left_wheel_position = msg.data[2]
        left_wheel_velocity = msg.data[3]

        joint_state_msg.position = [right_wheel_position, left_wheel_position]
        joint_state_msg.velocity = [right_wheel_velocity, left_wheel_velocity]

        # Publish the JointState message
        self.states_publisher.publish(joint_state_msg)
        self.get_logger().debug(f"Published joint states: {joint_state_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
