import rclpy
from rclpy.node import Node
from scipy.interpolate import interp1d
from std_msgs.msg import Float64MultiArray
import numpy as np

class VelocityToPWMNode(Node):
    def __init__(self):
        super().__init__('velocity_to_pwm_node')

        # Interpolated mappings for right and left wheels
        self.right_wheel_interp = None
        self.left_wheel_interp = None

        # Initialize interpolation with known data
        self.setup_interpolation()

        # ROS 2 subscriber and publisher
        self.subscriber = self.create_subscription(
            Float64MultiArray, '/topic_based_blah', self.velocity_callback, 10
        )
        self.publisher = self.create_publisher(Float64MultiArray, '/wheel_pwms', 10)

        self.get_logger().info("Velocity to PWM node is ready.")

    def setup_interpolation(self):
        # Replace these with your actual recorded data
        data = [[-18.786088964468103, -18.405290218100397], [-17.643692725364986, -17.643692725364986], [-17.389825646096423, -17.89755793209841], [-17.389825646096423, -17.77062439246413], [-17.009026899728717, -17.516759185730706], [-17.135960439363, -17.26289210646214], [-16.120495867359026, -17.009026899728717], [-15.612763581357038, -16.755161692995294], [-15.105031295355051, -16.374361074092448], [-14.597299009353064, -15.86662878809046], [-13.835700580350082, -15.612763581357038], [-13.201035690981383, -15.105031295355051], [-12.439437261978405, -14.470366405986352], [-11.677839769242993, -13.835700580350082], [-10.408509054238026, -12.693303404979398], [-9.26611187886734, -11.931704975976418], [-7.86984856049566, -10.662375197239019], [-6.473584773990196, -9.393044482234052], [-4.823455312617522, -7.615982417494666], [-3.300258922745346, -5.458120670120007], [-1.650129461372673, -3.1733258512448494], [0.0, -0.7615981949360882], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [1.9039956043736666, 0.0], [3.4271919942458426, 4.823455312617522], [5.077321455618516, 6.854383988491685], [6.600517845490692, 8.63144605323107], [8.123714703496653, 10.02771030787032], [9.519978021868333, 11.29704008660772], [10.916241340240013, 12.566370801612686], [11.931704975976418, 13.327969230615668], [13.074103087614674, 14.216500262985358], [14.089566723351076, 15.231964834989332], [14.851165152354056, 15.993562327724744], [15.485830041722757, 16.50129461372673], [16.120495867359026, 17.009026899728717], [16.50129461372673, 17.135960439363], [17.135960439363, 17.135960439363], [17.135960439363, 17.89755793209841], [17.77062439246413, 18.278356678466114], [18.278356678466114, 18.65915729736896], [18.405290218100397, 18.913022504102386], [18.278356678466114, 18.786088964468103], [19.547688329738655, 19.801553536472078]]

        # Extract data for interpolation
        pwm_steps = np.linspace(-255, 255, len(data))  # Assuming PWM steps from -255 to 255
        right_wheel_velocities = [row[0] for row in data]
        left_wheel_velocities = [row[1] for row in data]

        # Create interpolation functions
        self.right_wheel_interp = interp1d(
            right_wheel_velocities, pwm_steps, kind='linear', fill_value="extrapolate"
        )
        self.left_wheel_interp = interp1d(
            left_wheel_velocities, pwm_steps, kind='linear', fill_value="extrapolate"
        )

    def velocity_callback(self, msg):
        # Extract desired angular velocities
        if len(msg.data) != 2:
            self.get_logger().error("Input message must contain exactly two values (right, left).")
            return

        desired_velocity_right = msg.data[0]
        desired_velocity_left = msg.data[1]

        # Map angular velocities to PWM values
        pwm_right = self.right_wheel_interp(desired_velocity_right)
        pwm_left = self.left_wheel_interp(desired_velocity_left)

        # Constrain PWM values to the range [-255, 255]
        pwm_right = max(min(int(pwm_right), 255), -255)
        pwm_left = max(min(int(pwm_left), 255), -255)

        # Publish the calculated PWM values
        pwm_msg = Float64MultiArray()
        pwm_msg.data = [pwm_right, pwm_left]
        self.publisher.publish(pwm_msg)

        self.get_logger().info(f"Input velocities: Right={desired_velocity_right}, Left={desired_velocity_left}")
        self.get_logger().info(f"Calculated PWMs: Right={pwm_right}, Left={pwm_left}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityToPWMNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
