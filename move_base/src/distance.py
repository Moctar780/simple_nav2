#! /usr/bin/python3

"""
distance.py

This module implements a ROS 2 node for estimating a robot's 2D position (distance traveled) using odometry and IMU sensor data.
It subscribes to Odometry and IMU topics, processes the data to estimate the robot's pose, and republishes a filtered Odometry message.
A TF broadcaster is also provided for coordinate transforms.

Classes:
    DistanceEstimation: ROS 2 Node for distance estimation using odometry and IMU data.

Functions:
    main(args=None): Entry point for starting the ROS 2 node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np 
from tf2_ros import TransformBroadcaster, TransformStamped
class DistanceEstimation(Node):
    """
    ROS 2 Node for estimating the robot's 2D position using odometry and IMU data.

    Subscribes:
        - "imu" (sensor_msgs/Imu): IMU sensor data for orientation.
        - "/odom" (nav_msgs/Odometry): Odometry data for position and velocity.

    Publishes:
        - "odom_filtered" (nav_msgs/Odometry): Filtered odometry with estimated position.

    Broadcasts:
        - TF transforms for robot pose updates.

    Attributes:
        pose_2d_data (list): Latest odometry position [x, y].
        imu_data (float): Latest orientation (theta, in radians).
        last_pose (list): Previous odometry position [x, y].
        current_pose (list): Current odometry position [x, y].
        pose_robot (np.ndarray): Estimated robot position [x, y].
    """
    def __init__(self):
        """
        Initialize the DistanceEstimation node, set up subscriptions, publishers, and internal state.
        """
        super().__init__("DistanceEstimation")
        
        # Subscriptions to IMU and Odometry data
        imu_subs     = self.create_subscription(Imu, "imu", self.imu_call, 10)
        pose_subs    = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        # Publisher for filtered odometry
        self.republish_odom = self.create_publisher(Odometry, "odom_filtered", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        # Internal state initialization
        self.pose_2d_data = list()   # Stores latest odometry position [x, y]
        self.imu_data     = 0.0      # Stores latest robot orientation (theta)
    
        # For tracking previous and current odometry
        self.last_pose = list()
        self.current_pose = list()
        self.imu_msgs  = Imu()
        
        self.last_pose = [0.0, 0.0]  # Initialize last position to origin
        
        # Estimated robot position (x, y)
        self.pose_robot = np.array([0.0, 0.0])
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for processing incoming odometry data.

        Args:
            msg (nav_msgs.msg.Odometry): Incoming odometry message with position and velocity.

        Updates the estimated robot position based on the difference between the last and current positions,
        as well as the current orientation (theta) from IMU data. Republishes the filtered odometry.
        """
        # Get current time for message headers
        time = self.get_clock().now().seconds_nanoseconds()
        odom_msgs = Odometry()
        
        odom_msgs.header.frame_id = "base_footprint"
        odom_msgs.header.stamp.nanosec = time[1]
        odom_msgs.header.stamp.sec = time[0]
        odom_msgs.child_frame_id = "odom"
        
        new = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        
        # If robot is moving (linear velocity is non-zero)
        if new[0]:
            self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            # Calculate distance between previous and current position
            dist = math.dist(self.last_pose, self.current_pose)
            if new[0] < 0.0: dist = - dist  # Adjust sign if moving backward
            # Calculate change in position based on orientation (theta)
            point_t = np.array([np.cos(self.imu_data)*dist, np.sin(self.imu_data)*dist])
            
            # Update estimated robot position
            self.pose_robot += point_t
            # Save current position as last_pose for next iteration
            self.last_pose = self.current_pose.copy()
        else:
            # If robot stopped, just update last_pose
            self.last_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            
        # Update odometry message with estimated position and orientation
        odom_msgs.pose.pose.position.x = self.pose_robot[0]
        odom_msgs.pose.pose.position.y = self.pose_robot[1]
        odom_msgs.pose.pose.orientation = self.imu_msgs.orientation
        
        self.republish_odom.publish(odom_msgs)
        
        # Broadcast TF transform for the robot's estimated position
        transform = TransformStamped()
        transform.header.stamp.sec = time[0]
        transform.header.stamp.nanosec = time[1]
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.pose_robot[0]
        transform.transform.translation.y = self.pose_robot[1]
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = self.imu_msgs.orientation.x
        transform.transform.rotation.y = self.imu_msgs.orientation.y
        transform.transform.rotation.z = self.imu_msgs.orientation.z
        transform.transform.rotation.w = self.imu_msgs.orientation.w
        
        self.tf_broadcaster.sendTransform(transform)
        # Log the current estimated pose and orientation
        # self.get_logger().info(f"pose: [x: {self.pose_robot[0]}, y: {self.pose_robot[1]}], thata: {self.imu_data}")
    
    def imu_call(self, msg: Imu):
        """
        Callback for processing incoming IMU data.

        Args:
            msg (sensor_msgs.msg.Imu): Incoming IMU message with orientation in quaternion.

        Converts quaternion orientation to Euler angles and saves yaw (theta) for pose estimation.
        """
        # Convert quaternion orientation to euler angles (roll, pitch, yaw)
        angle_euler = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        # Save only yaw (theta) for 2D pose estimation
        self.imu_data = angle_euler[2]
        self.imu_msgs = msg
    
    
def main(args=None):
    """
    Entry point for the ROS 2 node.

    Args:
        args: Optional command-line arguments.

    Initializes the ROS 2 node and enters the main spinning loop.
    """
    rclpy.init(args=args)
    node = DistanceEstimation()

    try:
        rclpy.spin(node)  # Keep node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
