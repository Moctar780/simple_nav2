#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/action/move_point.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
using namespace rclcpp_action;

class TestCancel: public rclcpp::Node 
{
    public:
    TestCancel(): Node("TestCancel")
    {
        client = rclcpp_action::create_client<robot_msgs::action::MovePoint>(this, "/move_base");

        client ->async_cancel_all_goals();
    }
    private:
    std::shared_ptr<rclcpp_action::Client<robot_msgs::action::MovePoint>> client;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<TestCancel>());

    rclcpp::shutdown();
    return 0;
}