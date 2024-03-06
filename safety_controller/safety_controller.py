#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("publish_topic", "/vesc/high_level/input/nav_0")

        self.VELOCITY = 0.0
        self.STEER = 0.0

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('publish_topic').get_parameter_value().string_value
	
        self.subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 10)
        self.vel_subscription = self.create_subscription(AckermannDriveStamped, "/vesc/high_level/output", self.get_velocity, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

    def get_velocity(self):
        self.VELOCITY = self.get_parameter(self.vel_subscription.drive.speed).get_parameter_value().double_value
        self.STEER = self.get_parameter(self.vel_subscription.drive.steering_angle).get_parameter_value().double_value

    def lidar_callback(self, msg):
        """
        Process Lidar data
        """
        detect = self.safety_control(msg)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = detect
        drive_msg.drive.steering_angle = self.STEER

        self.publisher.publish(drive_msg)

    def slice_scan(self, ranges):
        """
        slice scan into managable pieces a
        """
        back_front_divider = int(len(ranges) * 16/33)
        front_middle_divider = int(len(ranges) * 16/33)
        total_length = int(len(ranges))
        bls = ranges[total_length - back_front_divider:total_length]
        ms = ranges[front_middle_divider:total_length - front_middle_divider]
        brs = ranges[0:back_front_divider]
        return {'brs': brs, "ms": ms, "bls" : bls}
    
    def return_data(self, data, flag = 0):
        """
        Returns appropriate data slice for side and speed
        """
        data = self.slice_scan(data)
        if flag:
            return np.array(data["ms"])
        if self.SIDE == -1:
            return np.array(data["brs"])
        return np.flip(np.array(data["bls"]))

    def safety_control(self, msg):
        """
        Compute distance to front wall and stop the car if the wall is too close
        """
        # distance from wall to stop the car
        distance_needed_to_stop = self.VELOCITY * 1/3
        # LIDAR scans from the front of the car
        front_data = np.array(self.return_data(msg.ranges, 1))
        # Angle values of the lidar scans
        middle_angles = np.array(np.cos(self.return_data(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment).tolist(), 1)))
        # scaled distance due to angle
        true_dist = np.multiply(middle_angles, front_data)
        if np.median(true_dist) < distance_needed_to_stop:
            return 0.0
        return self.VELOCITY


def main(args=None):
    
    rclpy.init(args=args)
    wall_follower = SafetyController()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()