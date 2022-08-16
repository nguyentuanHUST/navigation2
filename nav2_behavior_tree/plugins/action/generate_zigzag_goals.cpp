#include "nav2_behavior_tree/plugins/action/generate_zigzag_goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace nav2_behavior_tree
{
	GenerateZigzagGoals::GenerateZigzagGoals(
			const std::string &name,
			const BT::NodeConfiguration &conf) : BT::ActionNodeBase(name, conf)
	{
		node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
		// node_->declare_parameter("robot_width", rclcpp::ParameterValue(0.33));
		node_->get_parameter("robot_width", robot_width_);
		// node_->declare_parameter("robot_length", rclcpp::ParameterValue(0.61));
		node_->get_parameter("robot_length", robot_length_);
		node_->get_parameter("inflation_radius", inflation_radius_);
		node_->declare_parameter("right_offset", rclcpp::ParameterValue(0.0));
		node_->get_parameter("right_offset", right_side_offset_);
		node_->declare_parameter("left_offset", rclcpp::ParameterValue(0.0));
		node_->get_parameter("left_offset", left_side_offset_);
		path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("path_from_feet", rclcpp::QoS(10));
	}
	BT::NodeStatus GenerateZigzagGoals::tick()
	{
		RCLCPP_INFO(node_->get_logger(), "GenerateZigzagGoals ticks");
		getInput("legs", legs_);
		tf2::Vector3 leg23_v(legs_[3].x - legs_[2].x, legs_[3].y - legs_[2].y, 0);
		tf2::Vector3 leg10_v(legs_[0].x - legs_[1].x, legs_[0].y - legs_[1].y, 0);
		tf2::Vector3 leg21_v(legs_[1].x - legs_[2].x, legs_[1].y - legs_[2].y, 0);
		tf2::Vector3 leg30_v(legs_[0].x - legs_[3].x, legs_[0].y - legs_[3].y, 0);
		tf2::Vector3 leg2_v(legs_[2].x, legs_[2].y, 0);
		tf2::Vector3 leg1_v(legs_[1].x, legs_[1].y, 0);
		double yaw = tf2::tf2Angle(leg23_v, tf2::Vector3(1, 0, 0));
		std::vector<tf2::Transform> below_goals_in_legs_frame;
		std::vector<tf2::Transform> beyond_goals_in_legs_frame;
		int k = 1;
		geometry_msgs::msg::TransformStamped base_wheel_to_base_link = tf_->lookupTransform("base_wheel", "base_link", tf2::TimePointZero, tf2::durationFromSec(1));
		double x1 = base_wheel_to_base_link.transform.translation.x;
		RCLCPP_INFO(node_->get_logger(), "x1 %f", x1);
		double x2 = robot_length_ / 2 - x1;
		RCLCPP_INFO(node_->get_logger(), "x2 %f", x2);
		double x3 = robot_length_ - x2;
		RCLCPP_INFO(node_->get_logger(), "x3 %f", x3);
		// Zigzag cleaning
		{
			double safety = 0.1;
			tf2::Vector3 delta_23_s = leg23_v / leg23_v.length() * (x3 + safety);
			tf2::Vector3 delta_23_l = leg23_v - delta_23_s;
			// tf2::Vector3 delta_10_s = leg10_v / leg10_v.length() * (x3 + safety);
			// tf2::Vector3 delta_10_l = leg10_v - delta_10_s;
			while (true)
			{
				double l = right_side_offset_ + robot_width_ * k;
				tf2::Vector3 delta_2 = leg21_v / leg21_v.length() * l;
				tf2::Vector3 delta_3 = leg30_v / leg30_v.length() * l;
				if (l > leg21_v.length() - left_side_offset_)
					break;
				tf2::Transform tf = tf2::Transform::getIdentity();
				tf.setOrigin(leg2_v + delta_23_s + delta_2);
				tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
				// }
				below_goals_in_legs_frame.push_back(tf);
				tf.setOrigin(leg2_v + delta_23_l + delta_3);
				beyond_goals_in_legs_frame.push_back(tf);
				k++;
			}
		}

		for (int i = below_goals_in_legs_frame.size() - 1; i >= 0;)
		{
			geometry_msgs::msg::PoseStamped goal;
			goal.header.frame_id = "map";
			tf2::Transform tf = below_goals_in_legs_frame[i];
			tf2::toMsg(tf, goal.pose);
			goals_.push_back(goal);
			tf = beyond_goals_in_legs_frame[i];
			tf2::toMsg(tf, goal.pose);
			goals_.push_back(goal);
			if (i-- == 0)
				break;
			tf = beyond_goals_in_legs_frame[i];
			tf2::toMsg(tf, goal.pose);
			goals_.push_back(goal);
			tf = below_goals_in_legs_frame[i];
			tf2::toMsg(tf, goal.pose);
			goals_.push_back(goal);
			i--;
		}
		// geometry_msgs::msg::PoseStamped exit_point;
		// exit_point.pose.position.x = (beyond_goals_in_legs_frame.front().getOrigin().getX() + 
		// 	beyond_goals_in_legs_frame.back().getOrigin().getX())/2;
		// exit_point.pose.position.y = (beyond_goals_in_legs_frame.front().getOrigin().getY() + 
		// 	beyond_goals_in_legs_frame.back().getOrigin().getY())/2;
		// 	exit_point.pose.orientation.w = 1;
		// exit_point.header.frame_id = "map";
		// goals_.push_back(exit_point);
		setOutput("zigzag_goals", goals_);
		nav_msgs::msg::Path path;
		path.poses = goals_;
		path.header.frame_id = "map";
		path.header.stamp = node_->get_clock()->now();
		path_pub_->publish(path);
		goals_.clear();
		return BT::NodeStatus::SUCCESS;
	}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
	factory.registerNodeType<nav2_behavior_tree::GenerateZigzagGoals>("GenerateZigzagGoals");
}