#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <robot_msgs/srv/update_param.hpp>


using namespace std::placeholders;


class OdomToPose: public rclcpp::Node
{
	public:
		OdomToPose() : Node("OdomToPose")
	{
		pose_topic  = 	this -> declare_parameter("pose_topic", "pose2d");
		odom_topic  = 	this -> declare_parameter("odom_topic", "/odom_filtered");
		// robot initial pose [x, y] and angle [theta rd]
		init_x      =	this -> declare_parameter<double>("init_x", 0.0);
		init_y      =	this -> declare_parameter<double>("init_y", 0.0);
		init_theta  = 	this -> declare_parameter<double>("init_theta", 0.0);

		odom_subs   =  	this -> create_subscription<nav_msgs::msg::Odometry>(
				odom_topic,
				10,
				std::bind(&OdomToPose::odomCallback, this, _1));
		
		// publisher de la position du robot
		pose_pub   = 	this -> create_publisher<geometry_msgs::msg::Pose2D>(pose_topic, 10);
		update_param =  this-> create_service<robot_msgs::srv::UpdateParam>("update", std::bind(&OdomToPose::updateParameter, this, _1, _2));


	}

	private:
		std::string pose_topic, odom_topic;
		double init_x, init_y, init_theta;

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs;

		rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub;
		rclcpp::Service<robot_msgs::srv::UpdateParam>::SharedPtr update_param;

		void odomCallback(const nav_msgs::msg::Odometry & msg)
		{
			tf2::Quaternion q(
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w
                        );

			tf2::Matrix3x3 m(q);

			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			
			geometry_msgs::msg::Pose2D pose_robot_2d;
			pose_robot_2d.x = msg.pose.pose.position.x - init_x;
			pose_robot_2d.y = msg.pose.pose.position.y - init_y;
			pose_robot_2d.theta = yaw + init_theta;

			pose_pub -> publish(pose_robot_2d);
		}

		void updateParameter(std::shared_ptr<robot_msgs::srv::UpdateParam::Request> req,
							std::shared_ptr<robot_msgs::srv::UpdateParam::Response> res)
		{
			try
			{
				init_x = get_parameter("init_x").as_double();
				init_y = get_parameter("init_y").as_double();
				init_theta = get_parameter("init_theta").as_double();
				res -> state= true;
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				res -> state = false;
			}

		}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin( std::make_shared<OdomToPose>());
	rclcpp::shutdown();

	return 0;
}

