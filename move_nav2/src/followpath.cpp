#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ClientControlleur: public rclcpp::Node
{
    public:
        ClientControlleur():Node("ClientControlleur")
        {
            action_s = rclcpp_action::create_client<Follow>(this, "/follow_path");
            subs_path = this -> create_subscription<nav_msgs::msg::Path>
            (
                "plan", 10, [this](std::shared_ptr<nav_msgs::msg::Path> path)
                {
                    auto goal_msgs = Follow::Goal();
                    goal_msgs.path = *path;
                    goal_msgs.goal_checker_id = "general_goal_checker";
                    //goal_msgs.progress_checker_id = "progress_checker";
                    //goal_msgs.controller_id = "FollowPath";

                    auto send_goal_option  = rclcpp_action::Client<Follow>::SendGoalOptions();
                    send_goal_option.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle)
                    {

                    };
                    send_goal_option.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const Follow::Feedback> feedback)
                    {
                        // RCLCPP_INFO(this -> get_logger(), "dst: %f, speed: %f", feedback -> distance_to_goal, feedback -> speed);
                    };
                    send_goal_option.result_callback = [this](const GoalHandle::WrappedResult & result)
                    {
                        RCLCPP_INFO(this -> get_logger(), "error_code : %i, error_msgs: %s", result.code, result.result ->result);
                    };

                    action_s -> async_send_goal(goal_msgs, send_goal_option);

                }
            );
        }
    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subs_path;
        using Follow = nav2_msgs::action::FollowPath;
        using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;

        std::shared_ptr<rclcpp_action::Client<Follow>> action_s;



};

int main(int argc, char** argv)
{
    rclcpp::init(argc , argv);
    rclcpp::spin( std::make_shared<ClientControlleur>());
    rclcpp::shutdown();
    return 0;
}