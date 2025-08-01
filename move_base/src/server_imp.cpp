#include <move_base/server.hpp>

using namespace agrobot;

Move::Move(const rclcpp::NodeOptions & options): LifecycleNode("Move", "", options)
{
    RCLCPP_INFO(get_logger(), "Demarrage du server[%s]", this -> get_name());
    goal_exist = false;

}

CallBackReturn Move::on_configure(const rclcpp_lifecycle::State &)
{
    
    RCLCPP_INFO(this -> get_logger(), "Configuring node %s", this -> get_name());
    cmd_topic    = this -> declare_parameter("cmd_topic", "cmd_vel");
    RCLCPP_INFO(this -> get_logger(), "Declare cmd_topic : value [ %s ]", cmd_topic.c_str());
    
    action_name  = this -> declare_parameter("action_name", "move_base");
    RCLCPP_INFO(this -> get_logger(), "Declare action name : value [ %s ]", action_name.c_str());

    pose_topic   = this -> declare_parameter("pose2d", "pose2d");
    RCLCPP_INFO(this -> get_logger(), "Declare pose2d name : value [ %s ]", pose_topic.c_str());

    path_topic   = this -> declare_parameter("path", "plan");
    RCLCPP_INFO(this -> get_logger(), "Declare path : value [ %s ]", path_topic.c_str());

    angle_accuracy = this -> declare_parameter("angle_accuracy", 0.1);
    RCLCPP_INFO(this -> get_logger(), "Declare angle accuracy : value [ %f ]",angle_accuracy);

    goal_accuracy  = this -> declare_parameter("goal_accuracy", 0.15);
    RCLCPP_INFO(this -> get_logger(), "Declare goal accuracy : value [ %f ]", goal_accuracy);

    speed          = this -> declare_parameter("speed", 0.3);
    RCLCPP_INFO(this -> get_logger(), "Declare speed : value [ %f ]", speed);

    omega          = this -> declare_parameter("omega", 0.3);
    RCLCPP_INFO(this -> get_logger(), "Declare omega : value [ %f ]", omega);

    angle_minimal   = 	this -> declare_parameter<double>("angle_minimal", 0.15);
	vitesse_if_rotation         = 	this -> declare_parameter<double>("vitesse_if_rotation", 0.15);
		


    index = 0;
    length = 0;

    pub_cmd =  this -> create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    pub_path = this -> create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    
    return SUCCESS;



}

CallBackReturn Move::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(this -> get_logger(), "activating node: %s", this -> get_name());

    subs_robot_pose = this -> create_subscription<geometry_msgs::msg::Pose2D>(pose_topic, 10, std::bind(&Move::executeGoal, this, _1));

    timer_info      = this -> create_wall_timer(1s, std::bind(&Move::publishGoalInfo, this));

    action_server = rclcpp_action::create_server<nav_point>(
        this,
        action_name,
        std::bind(&Move::handle_goal, this, _1, _2),
        std::bind(&Move::handle_cancel, this, _1),
        std::bind(&Move::handle_accepted, this, _1));
    
    LifecycleNode::on_activate(state);
    
    return SUCCESS;

    

}
CallBackReturn Move::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(this -> get_logger(), "deactivate %s", this -> get_name());
    subs_robot_pose = nullptr;
    action_server = nullptr;
    stopRobot();
    LifecycleNode::on_deactivate(state);
    
    return SUCCESS;


}
CallBackReturn Move::on_shutdown(const rclcpp_lifecycle::State& state)
{

    stopRobot();
    return SUCCESS;

}

CallBackReturn Move::on_cleanup(const rclcpp_lifecycle::State& state)
{

    stopRobot();
    return SUCCESS;

}

void Move::stopRobot()
{
    if( !pub_cmd -> is_activated())
    {
        RCLCPP_ERROR(this -> get_logger(), "%s is not activate", cmd_topic.c_str());
        return;
    }

    auto stop_msgs = geometry_msgs::msg::Twist();
    stop_msgs.linear.x = 0.0;
    stop_msgs.angular.z = 0.0;

    pub_cmd -> publish(stop_msgs);
}

rclcpp_action::GoalResponse Move::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const nav_point::Goal> goal
)
{
    int end = goal -> x.size();
		RCLCPP_INFO(this->get_logger(), "Received goal request, begin goal [x: %f, y: %f], end: [x: %f, y: %f]",
		 goal->x[0], goal->y[0], goal -> x[end -1], goal -> y[end-1]);
		(void)uuid;

		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse Move::handle_cancel(
		const std::shared_ptr<goal_point> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
    index = 0;
    goal_exist = false;
    this -> stopRobot();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void Move::handle_accepted(const std::shared_ptr<goal_point> goal_handle)
{
    
    this -> goal_handle = goal_handle;
    goal_exist = true;
    index = 0;
    current_goal = getGoalByIndex(index);
}

geometry_msgs::msg::Point Move::getGoalByIndex(u_int index)
{
    auto point = geometry_msgs::msg::Point();
    point.x =  goal_handle -> get_goal() -> x[index];
    point.y = goal_handle -> get_goal() -> y[index];
    
    return point;
}


geometry_msgs::msg::Twist Move::getVelocityFromPose(float dx, float dy, float robot_theta)
{
    geometry_msgs::msg::Twist cmd_msgs;

    double dist = sqrt(pow(dx, 2) + pow(dy, 2));

    double theta_target = std::atan2(dy, dx);

	// 3) Différence d'angle brute
	double error = theta_target - robot_theta;

	// 4) Normalisation dans [-pi, +pi]
	error = std::atan2(std::sin(error), std::cos(error));


	// 6) Si on est assez proche (distance faible), on arrête tout
	if (dist < static_cast<double>(goal_accuracy))
	{
        cmd_msgs.linear.x = 0.0;
        cmd_msgs.angular.y = 0.0;
        return cmd_msgs;
	}

	// 7) Si l'erreur angulaire est au-dessus du seuil, on fait tourner seulement
	else if (std::abs(error) > static_cast<double>(angle_accuracy))
	{
		// On tourne vers la cible
        cmd_msgs.linear.x = std::abs(error) < angle_minimal ? vitesse_if_rotation: 0.0;
		if ( error > 0.0)
		{   
            cmd_msgs.angular.z = omega;  
		}
		else
		{
            cmd_msgs.angular.z = -omega;

		}
		return cmd_msgs;
	}
	else
	{
		// 8) On est orienté vers la cible : on avance en ligne droite
		cmd_msgs.linear.x = speed;
        cmd_msgs.angular.y = 0.0;
        return cmd_msgs;
	}

}

void Move::executeGoal(std::shared_ptr<geometry_msgs::msg::Pose2D> robot_pose)
{
    if (goal_exist)
        {
            
            float trajet_point_x = current_goal.x - robot_pose -> x;
            float trajet_point_y = current_goal.y - robot_pose -> y;
            auto command_robot = getVelocityFromPose(trajet_point_x, trajet_point_y, robot_pose -> theta);

            if (command_robot.linear.x == 0.0 && command_robot.angular.z == 0.0)
            {
                index++;
                if (index >= goal_handle -> get_goal() -> x.size())
                {
                goal_exist = false;
                stopRobot();
                RCLCPP_INFO(this -> get_logger(), "End the trajet");
                }
                else
                {
                    current_goal =  getGoalByIndex(index);
                    trajet_point_x = current_goal.x - robot_pose -> x;
                    trajet_point_y = current_goal.y - robot_pose -> y;
                    command_robot = getVelocityFromPose(trajet_point_x, trajet_point_y, robot_pose -> theta);

                    RCLCPP_INFO(this -> get_logger(), "Push point [%f, %f]", trajet_point_x, trajet_point_y);

                }

            }
            
            pub_cmd -> publish(command_robot);
          
        }
}

void Move::publishGoalInfo()
{
    if (goal_exist)
		{
			auto path = nav_msgs::msg::Path();
			path.header.frame_id = "odom";
			path.header.stamp = this -> get_clock() -> now();
            float dist_cumule = 0.0;
			for(u_int i = index -1 ; i < goal_handle -> get_goal() -> x.size() ; i++ )
			{
				auto pose = geometry_msgs::msg::PoseStamped();
                auto current_point = getGoalByIndex(i);

				pose.pose.position.x = current_point.x;
				pose.pose.position.y = current_point.y;
				path.poses.push_back(pose);
				
			}
			pub_path -> publish(path);
        }
}

float Move::getPathLenght()
{
    float distance_total = 0.0;
    //for(u_int i = index; i < )
}

float dist(float x1, float x2, float y1, float y2)
{
    return sqrt( pow(x1 - y1, 2) + pow(x2 - y2, 2));
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<agrobot::Move>();
    rclcpp::spin( node -> get_node_base_interface());
    rclcpp::shutdown();

    return 0;

}