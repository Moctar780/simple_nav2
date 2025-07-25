#! /usr/bin/python3
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
import rclpy
import sys
import os
from launch_ros.substitutions import FindPackageShare
from rclpy.action import ActionClient
from robot_msgs.action import MovePoint
from rclpy.action.client import CancelGoal, ClientGoalHandle
move_base = FindPackageShare(package='move_base').find("move_base")
import time
class PoseBag(Node):
    def __init__(self):
        super().__init__("PoseBag")
        options = ["play", "record", "display"]
        self._plan = self.create_publisher(Path, "/plan", 10)
        _option = self.declare_parameter("option", "")
        _file_name = self.declare_parameter("filename", "")
        _dst_to_save = self.declare_parameter("dst_to_save", 0.5)
        
        
        option_value = _option.get_parameter_value().string_value
        file_value = _file_name.get_parameter_value().string_value
        self.precision = _dst_to_save.get_parameter_value().double_value
        
        # Test option parameter and file if its exist
        if ( option_value not in options) :
            print (" invalide parameter")
            sys.exit(-1)

        elif option_value == "record":
            self._subs_pose = self.create_subscription(Pose2D, "pose2d", self.record_callback, 10)
            self.poses_path = Path()
            self.poses_path.header.frame_id = "odom"
            self.file = open(file_value, "w")
            self.last = ()
        elif option_value == "display":
            
            
            self.file = open(file_value, "r")
            x_y_coord = self.readFile(file_value)
            
            self.poses_path = self.convertPoseToPath(x_y_coord)
            
            self.poses_path.header.frame_id = "odom"
            self.poses_path.header.stamp = self.get_clock().now().to_msg()
            
            
            self.get_logger().info("display path")
            self.get_logger().info(f"number of point in path: {len(self.poses_path.poses)}")
            self.get_logger().info("end of display path")
            self.file.close()
            timer = self.create_timer(1, self.publishPath)
        else:
            """option mode option: read file send to action_move and publish path
            """
            if not os.path.isfile(file_value):
                print("Invalide file ")
                sys.exit(-1)
            x_y_coord = self.readFile(file_value)
            action_client = ActionClient(self, MovePoint, "move_base")
            while not action_client.server_is_ready():
                self.get_logger().info("move base is not ready")
                time.sleep(1)
            self.get_logger().info("move base is now ready")
            goal = MovePoint.Goal()
            goal.x = x_y_coord[0]
            goal.y = x_y_coord[1]
            action_client.send_goal_async(goal)
            sys.exit(0)
            
            
    
    def publishPath(self,):
        """publish path to the topic /plan
        """
        self._plan.publish(self.poses_path)
        
    def record_callback(self, msg : Pose2D):
        
        if self.last :
            n_vleur = abs(self.last[0] - msg.x ) >  self.precision or abs(self.last[1] - msg.y) > self.precision
            if n_vleur:
                pose = f'{msg.x}, {msg.y};'
                self.get_logger().info(pose)
                self.last = ()
                self.file.write(pose)
                
                pose_stamp = PoseStamped()
                pose_stamp.pose.position.x = msg.x
                pose_stamp.pose.position.y = msg.y
                self.poses_path.poses.append(pose_stamp)
                
                self._plan.publish(self.poses_path)
        
        else:
            self.last = (msg.x, msg.y)
    
    def convertPoseToPath(self, poses: list) -> Path:
        """convert pose_manage type into path type

        Args:
            poses (list[tuple]): a list of point

        Returns:
            Path: a path msgs
        """
        path = Path()
        # path.header.stamp = self.get_clock().now()
        path.header.frame_id = "odom"
        x = poses[0]
        y = poses[1]
        for point in zip(x, y):
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            
            path.poses.append(pose)
        
        return path
    
    def readFile(self, file_name: str) -> list:
        """read pose filename type coordinate

        Args:
            file_name (str): the path of the filename

        Returns:
            list(tuple): list of tuple coordinate point
        """
        file = open(file_name).readline()
        split_x_y = file.split(";")
        x_y_coord_str =  split_x_y[:-1]
        x_y_coord_float = list()
        x_coord, y_coord = list(), list()
        for point in x_y_coord_str:
            data =  point.split(",")
            x = float(data[0])
            y = float(data[1])
            x_coord.append(x)
            y_coord.append(y)
            
            # x_y_coord_float.append((x, y))
        return [x_coord, y_coord]
    
def main(args=None):

    rclpy.init(args=args)
    node = PoseBag()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
