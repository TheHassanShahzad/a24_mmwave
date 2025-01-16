#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan

class PCLToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pcl_to_laserscan_node')

        # Parameters you might want to adjust
        self.deg_per_scan_point = 1.0         # 1 degree per laser scan bin
        self.range_min = 0.0                 # Minimum valid range
        self.range_max = 20.0                # Maximum range of the radar
        self.angle_min = -math.pi            # -180 degrees
        self.angle_max =  math.pi            # +180 degrees

        # Derived
        self.angle_increment = math.radians(self.deg_per_scan_point)
        self.num_scan_points = int(
            round((self.angle_max - self.angle_min) / self.angle_increment)
        )

        # Subscriber: read incoming PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ti_mmwave/radar_scan_pcl',
            self.cloud_callback,
            10
        )

        # Publisher: publish LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

    def cloud_callback(self, pcl_msg: PointCloud2):
        """
        Convert each (x,y) point in the pointcloud to a polar coordinate
        (range, angle). Place the result in LaserScan bins.
        """

        # Prepare the LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = pcl_msg.header  # copy frame_id & timestamp from cloud
        # If the frame is something like "/ti_mmwave_0", you may want to strip the leading '/' 
        # or rename it. That depends on your TF setup.
        
        if scan_msg.header.frame_id.startswith('/'):
                    scan_msg.header.frame_id = scan_msg.header.frame_id[1:]

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Initialize "infinite" distances. We will fill them with actual data if we have a valid point.
        ranges = [float('inf')] * self.num_scan_points

        # Parse the PointCloud2 raw data
        # According to your fields:
        #   x: float32 @ offset 0
        #   y: float32 @ offset 4
        #   z: float32 @ offset 8
        #   (there may be padding at offset 12)
        #   intensity: float32 @ offset 16
        #   velocity: float32 @ offset 20
        #   total point_step: 32
        #   so, be careful about unused bytes if any.
        point_step = pcl_msg.point_step
        data = pcl_msg.data
        num_points = pcl_msg.width * pcl_msg.height

        for i in range(num_points):
            # Calculate where this point starts in the byte array
            start_idx = i * point_step

            # Unpack x, y
            x = struct.unpack_from('f', data, start_idx + 0)[0]
            y = struct.unpack_from('f', data, start_idx + 4)[0]
            # z = struct.unpack_from('f', data, start_idx + 8)[0]  # if needed

            # You could unpack intensity and velocity if useful:
            # intensity = struct.unpack_from('f', data, start_idx + 16)[0]
            # velocity = struct.unpack_from('f', data, start_idx + 20)[0]

            # Compute range and angle
            r = math.sqrt(x*x + y*y)
            if r < 1e-5:
                # Too close, skip
                continue

            angle = math.atan2(y, x)  # [-pi, +pi]

            # Filter out points that are out of the configured radar range
            if r > self.range_max or r < self.range_min:
                continue

            # Find which bin in LaserScan this angle belongs to
            # e.g., for an angle in [-pi, +pi], bin 0 => angle_min
            bin_angle = angle - self.angle_min
            bin_index = int(bin_angle / self.angle_increment)

            if 0 <= bin_index < self.num_scan_points:
                # We want the minimum range for that bin (the closest object)
                if r < ranges[bin_index]:
                    ranges[bin_index] = r

        # Replace 'inf' with a max range or leave them as is
        # so the LaserScan indicates no return.
        # For example, just leave them as inf:
        scan_msg.ranges = ranges

        # Publish the LaserScan
        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PCLToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
