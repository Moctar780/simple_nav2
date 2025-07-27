#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_msgs/action/move_point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <chrono>
#include <functional>
#include <complex>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace agrobot
{
    using CallBackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using nav_point = robot_msgs::action::MovePoint;
	using goal_point = rclcpp_action::ServerGoalHandle<nav_point>;

    auto SUCCESS  = CallBackReturn::SUCCESS;
    auto ERROR    = CallBackReturn::ERROR;
    auto FAIL     = CallBackReturn::FAILURE;

    class Move: public rclcpp_lifecycle::LifecycleNode
    {
        public:
            Move(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


            CallBackReturn
            on_configure(const rclcpp_lifecycle::State & state);
            
            CallBackReturn
            on_activate(const rclcpp_lifecycle::State & state);

            CallBackReturn
            on_deactivate(const rclcpp_lifecycle::State & state);

            CallBackReturn
            on_shutdown(const rclcpp_lifecycle::State & state);

            CallbackReturn
            on_cleanup(const rclcpp_lifecycle::State & state);

        private:
            // publish command necessary to move the robot to his goal
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> pub_cmd;

            // publish additional information, current path, current-dist to goal etc
            rclcpp::TimerBase::SharedPtr timer_info;

            // publish current path to topic plan
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> pub_path;

            // Get robot pose subscription [x, y, theta] and calculate the command necessary to
            // move to goal point
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subs_robot_pose;

            // action server, receive a set of point and move the robot in their points
            rclcpp_action::Server<nav_point>::SharedPtr action_server;

            // variable 
            std::string cmd_topic, action_name, pose_topic, path_topic;
            // difference between robot current angle and goal angle
            float angle_accuracy;

            // difference between robot current pose and goal pose
            float goal_accuracy;

            // robot velocity angle rad/s
            float omega;

            // robot velocity speed m/s
            float speed;

            // index to current goal in the set of goals
            u_int index;

            // goal state
            bool goal_exist;
            
            // angle minimal to add speed
            float angle_minimal;

            // speed if angle minimal is necessary
            float vitesse_if_rotation;
            // goal handle [ set of points]
            std::shared_ptr<goal_point> goal_handle;

            // current goal in complex form
            geometry_msgs::msg::Point current_goal;

            geometry_msgs::msg::Twist getVelocityFromPose(float goal_x, float goal_y, float goal_theta);

            // stop publish twist when it is call
            void stopRobot();

            // push current goal to move the robot in this goal
            void executeGoal(std::shared_ptr<geometry_msgs::msg::Pose2D> pose);

            // get goal by index
            geometry_msgs::msg::Point getGoalByIndex(u_int index);

            void publishGoalInfo();

            rclcpp_action::GoalResponse handle_goal(

            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const nav_point::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(
		    const std::shared_ptr<goal_point> goal_handle);

            void handle_accepted(const std::shared_ptr<goal_point> goal_handle);
    };
}