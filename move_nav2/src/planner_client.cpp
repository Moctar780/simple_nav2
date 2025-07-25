#include <memory>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

class ClientPlanner: public rclcpp::Node
{
    public:
        ClientPlanner():Node("client"), start(false)
        {
            action_path = rclcpp_action::create_client<ComputPath>(this, "/compute_path_to_pose");

            timer = this -> create_wall_timer(1s, [this]()
        {
            if (start)
            {
                auto goal_msgs = ComputPath::Goal();
                goal_msgs.goal = goal_pose;
                goal_msgs.use_start = false;
                goal_msgs.planner_id = "GridBased";

                action_path -> async_send_goal(goal_msgs);
            }
        } );
            subs_goal_pose = this -> create_subscription<geometry_msgs::msg::PoseStamped>(
                    "goal_pose", 10,  [this] (std::shared_ptr<geometry_msgs::msg::PoseStamped> msg ) {
                                    
                        RCLCPP_INFO(this -> get_logger(), "x : %f, y : %f, z : %f",
                                    msg -> pose.position.x, msg -> pose.position.y, msg -> pose.position.z);
                                    
                        
                        start =  true;
                        goal_pose = *msg;

                                    
                    } );
          

        }
    private:
	void goal_callback(std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
	{
		RCLCPP_INFO(this -> get_logger(), "x : %f, y : %f, z : %f",
				msg -> pose.position.x, msg -> pose.position.y, msg -> pose.position.z);
	}
        using ComputPath = nav2_msgs::action::ComputePathToPose;
        std::shared_ptr<rclcpp_action::Client<ComputPath>> action_path;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subs_goal_pose;
        // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subs_init_pose;
        // geometry_msgs::msg::PoseStamped pose_init;
        bool start;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::TimerBase::SharedPtr timer;
        // std::unique_ptr<ActionClient> action_client;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<ClientPlanner>());
    rclcpp::shutdown();
    
    return 0;
}
