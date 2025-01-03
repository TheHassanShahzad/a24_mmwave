import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class MotorCalibrationNode(Node):
    def __init__(self):
        super().__init__("motor_calibration")
        
        # Publishers and subscribers
        self.pub = self.create_publisher(Float64MultiArray, "/wheel_velocities", 10)
        self.sub = self.create_subscription(Float64MultiArray, "/state_feedback", self.state_feedback_callback, 10)
        
        # Parameters
        self.pwm = -250  # Initial PWM value
        self.pwm_step = 10
        self.max_pwm = 250
        self.samples_per_pwm = 50  # Number of callbacks to wait for
        self.recorded_data = []  # 2D list to store final velocities [vel_r, vel_l]
        
        # Recording
        self.current_samples = []
        self.callback_count = 0
        
        # Timer to publish PWM commands
        self.timer = self.create_timer(0.1, self.publish_pwm_command)  # 10 Hz
    
    def publish_pwm_command(self):
        # Publish the current PWM to both wheels
        msg = Float64MultiArray()
        msg.data = [float(self.pwm), float(self.pwm)]
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing PWM: {self.pwm}")
    
    def state_feedback_callback(self, msg):
        # Record state feedback
        self.callback_count += 1
        
        # Extract angular velocities (assuming format: [pos_r, vel_r, pos_l, vel_l])
        vel_r = msg.data[1]
        vel_l = msg.data[3]
        self.current_samples.append((vel_r, vel_l))
        
        # After enough samples, record the 50th callback data and update PWM
        if self.callback_count >= self.samples_per_pwm:
            final_vel_r, final_vel_l = self.current_samples[-1]
            self.recorded_data.append([final_vel_r, final_vel_l])
            
            self.get_logger().info(f"Recorded Velocities: Right={final_vel_r}, Left={final_vel_l}")
            
            # Prepare for the next PWM value
            self.callback_count = 0
            self.current_samples = []
            self.pwm += self.pwm_step
            
            # Stop if max PWM is reached
            if self.pwm > self.max_pwm:
                self.stop_recording()
    
    def stop_recording(self):
        self.get_logger().info("Stopping recording...")
        
        # Publish [0, 0] to stop the motors
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0, 0.0]
        self.pub.publish(stop_msg)
        self.get_logger().info("Motors stopped.")
        
        # Print final recorded data
        
        print("\nFinal Recorded Data (Right and Left Wheel Velocities):")
        for i, (vel_r, vel_l) in enumerate(self.recorded_data):
            print(f"Step {i+1}: Right={vel_r}, Left={vel_l}")
        
        print(self.recorded_data)

        # Shutdown the node
        rclpy.shutdown()

def main():
    rclpy.init()
    node = MotorCalibrationNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
