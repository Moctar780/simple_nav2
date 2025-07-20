#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class Convert: public rclcpp::Node
{
    public:
        Convert():Node("convert")
        {
            out_twist = this -> create_publisher<geometry_msgs::msg::TwistStamped>("/robot_base_controller/cmd_vel_unstamped", 10);
            in_twist = this -> create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel_stamped", 10, [this](std::shared_ptr<geometry_msgs::msg::Twist> twist)
                {
                    auto msg =  geometry_msgs::msg::TwistStamped();
                    msg.twist  = *twist;
                    out_twist -> publish(msg);
                }
            );

        }
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr in_twist;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr out_twist;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Convert>());
    rclcpp::shutdown();
    return 0;
}