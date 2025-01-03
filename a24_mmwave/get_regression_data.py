import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import matplotlib.pyplot as plt


class PWMToVelocityMapper(Node):
    def __init__(self):
        super().__init__('pwm_to_velocity_mapper')
        
        # Publisher and Subscriber
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)
        self.sub = self.create_subscription(Float64MultiArray, '/state_feedback', self.feedback_callback, 10)
        
        # Variables for PWM and velocity mapping
        self.current_pwm = -250
        self.max_pwm = 250
        self.step = 10
        self.duration = 5  # seconds to wait per PWM
        self.angular_velocities = []  # Store angular velocity readings
        self.pwm_values = []  # Store corresponding PWM values
        
        # Temporary storage for feedback
        self.latest_feedback = [0.0, 0.0]
        self.feedback_received = False

        # Start the process
        self.start_pwm_test()

    def feedback_callback(self, msg):
        # Extract angular velocities for the two wheels
        self.latest_feedback = msg.data[1::2]  # Assuming [pos_r, vel_r, pos_l, vel_l]
        self.feedback_received = True

    def start_pwm_test(self):
        while self.current_pwm <= self.max_pwm:
            self.send_pwm_command(self.current_pwm)
            self.get_logger().info(f"Testing PWM: {self.current_pwm}")
            
            # Wait for the system to stabilize
            time.sleep(self.duration)
            
            # Record feedback
            if self.feedback_received:
                self.pwm_values.append(self.current_pwm)
                self.angular_velocities.append(self.latest_feedback.copy())
                self.get_logger().info(f"Recorded Velocities: {self.latest_feedback}")
            
            # Increment PWM
            self.current_pwm += self.step

        # Stop the robot and plot the results
        self.send_pwm_command(0)
        self.plot_results()

    def send_pwm_command(self, pwm_value):
        msg = Float64MultiArray()
        msg.data = [pwm_value, pwm_value]
        self.pub.publish(msg)

    def plot_results(self):
        # Extract individual wheel velocities
        vel_r = [vel[0] for vel in self.angular_velocities]
        vel_l = [vel[1] for vel in self.angular_velocities]
        
        # Plot data
        plt.figure()
        plt.plot(self.pwm_values, vel_r, label='Right Wheel')
        plt.plot(self.pwm_values, vel_l, label='Left Wheel')
        plt.title('PWM vs Angular Velocity')
        plt.xlabel('PWM Value')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid()
        plt.show()

        # Print values for regression
        print("PWM to Velocity Data:")
        for pwm, vel in zip(self.pwm_values, self.angular_velocities):
            print(f"PWM: {pwm}, Velocities: {vel}")


def main(args=None):
    rclpy.init(args=args)
    node = PWMToVelocityMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
