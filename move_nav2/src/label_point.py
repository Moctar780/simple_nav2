#! /usr/bin/python3

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import rclpy
import  os

class LabelPoint(Node):
    def __init__(self):
        super().__init__("Label")
        options = ["labelling", "move"]
        option = self.declare_parameter("option", "labelling")
        map_file = self.declare_parameter("filename", "")
        option_value = option.get_parameter_value().string_value
        map_file_value = map_file.get_parameter_value().string_value
        if option_value not in options:
            # parametre non trouve, exit du programme
            print(f"option [{option_value}] non reconnue")
            sys.exit(-1)
        elif option_value == options[0]:
            # demarrage du labelling
            self.file_save = open(map_file_value, "w")
            self.subs_map = self.create_subscription(PoseStamped, "clicked_map", self.label, 10)
        else:
            # demarrage du nav par label
            if not os.path.exists(map_file_value):
                print(f"filename: {map_file_value}, don't exist please try again")
                sys.exit(-1)
            file_data = open(map_file_value, "r").readline().split(";")[:-1]
            self.menu_data = {}
            for i in file_data:
                split_coor = i.split(",")
                name = split_coor[2]
                x    = split_coor[0]
                y    = split_coor[1]
                self.menu_data[name.strip()] = (x, y)
            timer = self.create_timer(1, self.menu)
            self.pub_label_point = self.create_publisher(PoseStamped, "/goal_pose", 10)
        
        self.label_map = {}
    def label(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        print(f"Coordonnee: [x : {x}, y: {y}]")
        name = input("Veuillez entrez un nom: ")
        file_form = f"{x}, {y}, {name};"
        self.file_save.write(file_form)
        
    
    def menu(self,):
        self.affiche()
        menu = input("Entre un numero: ")
        if menu == "exit":sys.exit(0)
        elif menu not in self.menu_data.keys():
            print("lieu non trouve")
            return 
        
        print(self.menu_data[menu])
        coor = self.menu_data[menu]
        x =  float(coor[0].strip())
        y = float(coor[1].strip())
        
        goal = PoseStamped()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.header.frame_id = "map"
        
        self.pub_label_point.publish(goal)
        
    
    def affiche(self):
        
        for i in self.menu_data.items():
            print(i[0], f" -> x: {i[1][0]}, y: {i[1][1]}")
     
     
def main(args=None):
    """
    Entry point for the ROS 2 node.

    Args:
        args: Optional command-line arguments.

    Initializes the ROS 2 node and enters the main spinning loop.
    """
    rclpy.init(args=args)
    node = LabelPoint()

    try:
        rclpy.spin(node)  # Keep node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
