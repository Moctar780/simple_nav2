#include <memory>
#include <functional>
#include <thread>
#include <complex>
#include <cmath>

#include <chrono>
#include <robot_msgs/action/move_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_msgs/srv/update_param.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using namespace std::placeholders;
using namespace std::chrono_literals;

class Move : public rclcpp::Node
{
public:
	using nav_point = robot_msgs::action::MovePoint;
	using goal_point = rclcpp_action::ServerGoalHandle<nav_point>;

	Move(const rclcpp::NodeOptions &option = rclcpp::NodeOptions()) : Node("MovePoint", option), goal_exist(false), index(0), dist_t(0)
	{
		action_name     = 	this -> declare_parameter("action_name", "move_base");
		cmd_vel_topic   = 	this -> declare_parameter("cmd_vel_topic", "cmd_vel");
		pose_2d_topic   =	this -> declare_parameter("pose_2d_topic", "pose2d");

		angle_precision = 	this -> declare_parameter<double>("angle_accuracy", 0.1);
		goal_precision  = 	this -> declare_parameter<double>("goal_accuracy", 0.1);
		omega           = 	this -> declare_parameter<double>("omega", 0.5);
		vitesse         = 	this -> declare_parameter<double>("vitesse", 1.0);
		angle_minimal   = 	this -> declare_parameter<double>("angle_minimal", 0.15);
		vitesse_if_rotation         = 	this -> declare_parameter<double>("vitesse_if_rotation", 0.15);
		

		action_server   = rclcpp_action::create_server<nav_point>(
			this,
			action_name,
			std::bind(&Move::handle_goal, this, _1, _2),
			std::bind(&Move::handle_cancel, this, _1),
			std::bind(&Move::handle_accepted, this, _1));

		robot_pose      = this->create_subscription<geometry_msgs::msg::Pose2D>(
			pose_2d_topic, 10,
			std::bind(&Move::execute, this, _1));

		cmd_vel         = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

		timer_path_pub  = this -> create_wall_timer(1s, std::bind(&Move::pathPublish, this));
		path_pub        = this -> create_publisher<nav_msgs::msg::Path>("plan", 10);

		update_param    = this-> create_service<robot_msgs::srv::UpdateParam>("update", std::bind(&Move::updateParameter, this, _1, _2));
		RCLCPP_INFO(this->get_logger(), "Demarrage du serveur [%s] ...", action_name.c_str());

	}

private:
	bool goal_exist;

	std::string action_name, cmd_vel_topic, pose_2d_topic;

	rclcpp_action::Server<nav_point>::SharedPtr action_server;
	// create a subscripter to souscribe to odom topic and get robot pose
	rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr robot_pose;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

	rclcpp::TimerBase::SharedPtr timer_path_pub;
	// service update MoveToPoint parameters
	rclcpp::Service<robot_msgs::srv::UpdateParam>::SharedPtr update_param;

	void updateParameter(std::shared_ptr<robot_msgs::srv::UpdateParam::Request> req,
							std::shared_ptr<robot_msgs::srv::UpdateParam::Response> res);
	
	

	float angle_precision, goal_precision, omega, vitesse, angle_minimal, vitesse_if_rotation ;

	float const VITESSE_ZERO= 0.0f, ROTATION_ZERO = 0.0f;
	int index;
	float dist_t;

	// position en cour du robot
	geometry_msgs::msg::Pose2D current_pose;
	std::shared_ptr<goal_point> goal_handle;
	std::complex<double> current_goal;

	// fonction fournissant la commande de la vitesse
	// et de la rotation a la base mobile
	std::pair<float, float> cmdvelRegulator(std::complex<double> &, double);
	// function to  publish current path
	void pathPublish()
	{
		if (goal_exist)
		{
			auto path = nav_msgs::msg::Path();
			path.header.frame_id = "odom";
			path.header.stamp = this -> get_clock() -> now();
			float dist_total = 0;

			for(u_int i = index -1 ; i < goal_handle -> get_goal() -> x.size() ; i++ )
			{
				auto pose = geometry_msgs::msg::PoseStamped();
				pose.pose.position.x = goal_handle -> get_goal() -> x[i];
				pose.pose.position.y = goal_handle -> get_goal() -> y[i];

				path.poses.push_back(pose);
				dist_total += this -> dist(pose.pose.position.x, pose.pose.position.y, 
											goal_handle -> get_goal() -> x[i+1],
											goal_handle -> get_goal() -> y[i+1]);
			}
			path_pub -> publish(path);

			dist_t = dist_total;
		}
		

	}

	float dist(int p1_x, int p1_y, int p2_x, int p2_y)
	{
		return sqrt( pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
	}

	rclcpp_action::GoalResponse handle_goal(

		const rclcpp_action::GoalUUID &uuid,
		std::shared_ptr<const nav_point::Goal> goal)
	{
		int end = goal -> x.size();
		RCLCPP_INFO(this->get_logger(), "Received goal request, begin goal [x: %f, y: %f], end: [x: %f, y: %f]",
		 goal->x[0], goal->y[0], goal -> x[end -1], goal -> y[end-1]);
		(void)uuid;

		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<goal_point> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
		index = 0;
		goal_exist = false;
		this -> stop_robot();

		return rclcpp_action::CancelResponse::ACCEPT;
	}
	void handle_accepted(const std::shared_ptr<goal_point> goal_handle)
	{
		// std::thread{std::bind(&Move::execute, this, _1), goal_handle}.detach();
		// this -> execute(goal_handle);
		this -> goal_handle = goal_handle;
		goal_exist = true;
	
	}
	void stop_robot()
	{
		auto put_cmdvel      = geometry_msgs::msg::Twist();
		put_cmdvel.linear.x  = 0.0;
		put_cmdvel.angular.z = 0.0;
		cmd_vel->publish(put_cmdvel);
	}

	bool pushGoal(const std::shared_ptr<geometry_msgs::msg::Pose2D> current_pose)
	{
		std::complex<double> trajet_point { current_goal - std::complex<double>(current_pose -> x, current_pose -> y)};

		auto [v_cmd, omega_cmd] = cmdvelRegulator(trajet_point, current_pose -> theta);
		
		if (v_cmd == VITESSE_ZERO && omega_cmd == ROTATION_ZERO)
		{
			
			index++;
			if (index >= goal_handle -> get_goal() -> x.size())
			{
				return false;
				RCLCPP_INFO(this -> get_logger(), "End  the trajet");
			}
			
			else
			{
				current_goal = getGoal(index);
				RCLCPP_INFO(this -> get_logger(), "Push Goal [x: %f, y: %f]", current_goal.real(), current_goal.imag());
				
				return true;
			}
			
		}
		else
			{
				return true;
			}


	}

	std::complex<double> getGoal(int index)
	{
		std::complex<double> point { goal_handle -> get_goal() -> x[index], goal_handle -> get_goal() -> y[index]};

		return point;
	}
	void execute(const std::shared_ptr<geometry_msgs::msg::Pose2D> current_pose)
	{

		if(goal_exist)
		{
			auto state_goal = pushGoal(current_pose);

			auto put_cmdvel = geometry_msgs::msg::Twist();
			if (!state_goal)
			{
				goal_exist = false;
				index = 0;
				stop_robot();
				auto result =  std::make_shared<nav_point::Result>();
				result -> state = true;
				goal_handle -> succeed(result);

			}
			
			auto feedback = std::make_shared<nav_point::Feedback>();

			auto robot_current_pose = std::complex<double>(current_pose -> x, current_pose -> y);
			// 1) On recalcule le vecteur (dx, dy) vers le but

			std::complex<double> trajet_point { current_goal- robot_current_pose};

			// 2) On récupère (v, omega)
			auto [v_cmd, omega_cmd] = cmdvelRegulator(trajet_point, current_pose -> theta);

			// 
			if ( v_cmd != VITESSE_ZERO || omega_cmd != ROTATION_ZERO)
			{
				// si la vitesse et la rotation n'est plus necessaire on arrete le robot
				put_cmdvel.linear.x = v_cmd;
				put_cmdvel.angular.z = omega_cmd;
				cmd_vel->publish(put_cmdvel);

				feedback->current_x = current_goal.real();
				feedback->current_y = current_goal.imag();

				goal_handle->publish_feedback(feedback);
				// RCLCPP_INFO( this -> get_logger(), "Distance restant: %f, temps restant: %f", dist_t, static_cast<double> (dist_t)/vitesse);
				// std::cout << "Distance restante : " << dist_t << " temps restant : " << dist_t / vitesse << "\n";
			}
			
				// Si (v==0 && omega==0) alors on considère qu’on a terminé
				
		}
		
	}

};

std::pair<float, float> Move::cmdvelRegulator(std::complex<double> &target_point, double robot_theta)
{
	// 1) Calcul de la distance réelle au but
	double dx = target_point.real();
	double dy = target_point.imag();
	double dist = std::norm(target_point);

	// 2) Calcul de l'angle absolu vers le but
	double theta_target = std::atan2(dy, dx);

	// 3) Différence d'angle brute
	double error = theta_target - robot_theta;

	// 4) Normalisation dans [-pi, +pi]
	error = std::atan2(std::sin(error), std::cos(error));

	// 5) Retour d'informations pour debug
	// RCLCPP_INFO(this->get_logger(), "Dist au but: %.3f, erreur angle normalisee: %.3f (theta_robot=%.3f, theta_target=%.3f)",
	// 			dist, error, robot_theta, theta_target);

	// 6) Si on est assez proche (distance faible), on arrête tout
	if (dist < static_cast<double>(goal_precision))
	{
		return {0.0f, 0.0f};
	}

	// 7) Si l'erreur angulaire est au-dessus du seuil, on fait tourner seulement
	if (std::abs(error) > static_cast<double>(angle_precision))
	{
		// On tourne vers la cible
		float v_omega = abs(error) < angle_minimal ? vitesse_if_rotation : 0.0;
		if ( error > 0.0)
		{
			return {v_omega, omega};
		}
		else
		{
			return {v_omega, -omega};

		}
		
	}
	else
	{
		// 8) On est orienté vers la cible : on avance en ligne droite
		
		return {vitesse, 0.0f};
	}

}


void Move::updateParameter(std::shared_ptr<robot_msgs::srv::UpdateParam::Request> req,
							std::shared_ptr<robot_msgs::srv::UpdateParam::Response> res)
		{
			try
			{
				angle_precision  = get_parameter("angle_accuracy").as_double();
				goal_precision   = get_parameter("goal_accuracy").as_double();
				omega            = get_parameter("omega").as_double();
				vitesse          = get_parameter("omega").as_double();

				res -> state= true;
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				res -> state = false;
			}

		}

RCLCPP_COMPONENTS_REGISTER_NODE(Move)

/*int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<Move>());
rclcpp::shutdown();

return 0;
} */
