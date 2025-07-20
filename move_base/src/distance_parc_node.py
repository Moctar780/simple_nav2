#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math
import numpy as np 
import cmath
from tf2_ros import TransformBroadcaster
class DistanceEstimation(Node):
    def __init__(self):
        super().__init__("DistanceEstimation")
        
        cmd_vel_subs = self.create_subscription(TwistStamped, "/robot_base_controller/cmd_vel_unstamped", self.cmd_vel_moniter, 10)
        imu_subs     = self.create_subscription(Imu, "/imu_plugin/out", self.imu_call, 10)
        pose_subs    = self.create_subscription(Odometry, "/robot_base_controller/odom", self.pose_2d, 10)
        
        self.republish_odom = self.create_publisher(Odometry, "odom_filtered", 10)
        # timer = self.create_timer(0.1, self.time)
        self.tf = TransformBroadcaster(self)
        
        self.pose_2d_data = list()
        self.cmd_data     = list()
        self.imu_data     = 0.0
        
        self.imu_msgs     = Imu()
        
        self.pose_2d_data.append(0.0)
        self.pose_2d_data.append(0.0)
        self.cmd_data.append(0.0)
        self.cmd_data.append(0.0)
        
        self.last_pose = list()
        self.current_pose = list()
        self.current =  False
        
        
        self.pose_robot = np.array([0.0, 0.0])
    def time(self):
        time = self.get_clock().now().seconds_nanoseconds()
        odom_msgs = Odometry()
        t = TransformStamped()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "odom_filtre"
        t.header.stamp.sec = time [0]
        t.header.stamp.nanosec = time[1]
        
        odom_msgs.header.frame_id = "base_footprint"
        odom_msgs.header.stamp.nanosec = time[1]
        odom_msgs.header.stamp.sec = time[0]
        odom_msgs.child_frame_id = "odom_filtre"
        
        odom_msgs.pose.pose.position.x = self.pose_robot[0]
        odom_msgs.pose.pose.position.y = self.pose_robot[1]
        odom_msgs.pose.pose.orientation = self.imu_msgs.orientation
        self.republish_odom.publish(odom_msgs)
        
        t.transform.translation.x = - self.pose_robot[0]
        t.transform.translation.y = - self.pose_robot[1]
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = self.imu_msgs.orientation.x
        t.transform.rotation.y = self.imu_msgs.orientation.y
        t.transform.rotation.z = self.imu_msgs.orientation.z
        t.transform.rotation.w = self.imu_msgs.orientation.w
        
        
        self.tf.sendTransform(t)
        
        self.get_logger().info(f"pose: [x: {self.pose_robot[0]}, y: {self.pose_robot[1]}], thata: {self.imu_data}")
        # self.cmd_data[0] = cmd_vel.twist.linear.x
        # self.cmd_data[1] = cmd_vel.twist.angular.z
        
    def cmd_vel_moniter(self, cmd_vel: TwistStamped):
        
        new = [cmd_vel.twist.linear.x, cmd_vel.twist.angular.z]
        # si la nouvelle vitesse [linear and engular est null ]
        if new[0]:
            self.current_pose = self.pose_2d_data.copy()
            self.current = True
            self.get_logger().info("current")
            # odom_msgs.pose.pose.position.x = self.pose_robot[0]
            # odom_msgs.pose.pose.position.y = self.pose_robot[1]
            # odom_msgs.pose.pose.orientation = self.imu_msgs.orientation
            # self.last_pose = self.current_pose.copy()
            
        else:
            # odom_msgs.pose.pose.orientation = self.imu_msgs.orientation
            self.last_pose = self.pose_2d_data.copy()
            self.get_logger().info("last")

            
        
        
    
    def imu_call(self, msg: Imu):
        angle_euler = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        self.imu_data = angle_euler[2]
        self.imu_msgs = msg
    
    def pose_2d(self, msg: Odometry):
        self.pose_2d_data[0] = msg.pose.pose.position.x
        self.pose_2d_data[1] = msg.pose.pose.position.y
    
def main(args=None):

    rclpy.init(args=args)
    node = DistanceEstimation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


