#include "nav2_behavior_tree/plugins/action/get_legs.hpp"

namespace nav2_behavior_tree
{
	GetLegs::GetLegs(
    const std::string & name,
    const BT::NodeConfiguration & conf):
		BT::ActionNodeBase(name, conf),
		num_medium_(1) {
		node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
		getInput("nums", num_medium_);
	}

  BT::NodeStatus GetLegs::tick() {
		RCLCPP_INFO(node_->get_logger(), "GetLegs ticks");
		setStatus(BT::NodeStatus::RUNNING);
		std::vector<geometry_msgs::msg::Point> legs(4);
		int n = 0;
		while(n < num_medium_) {
			try
			{
				geometry_msgs::msg::TransformStamped leg_0 = tf_->lookupTransform("map", "leg_0", tf2::TimePointZero, tf2::durationFromSec(1));
				geometry_msgs::msg::TransformStamped leg_1 = tf_->lookupTransform("map", "leg_1", tf2::TimePointZero, tf2::durationFromSec(1));
				geometry_msgs::msg::TransformStamped leg_2 = tf_->lookupTransform("map", "leg_2", tf2::TimePointZero, tf2::durationFromSec(1));
				geometry_msgs::msg::TransformStamped leg_3 = tf_->lookupTransform("map", "leg_3", tf2::TimePointZero, tf2::durationFromSec(1));
				if(leg_0 == leg_1 || leg_0 == leg_2 || leg_0 == leg_3 || leg_1 == leg_2 || leg_1 == leg_3 || leg_2 == leg_3) {
					continue;
				}
				getMedium(legs[2], n, leg_2);
				getMedium(legs[1], n, leg_1);
				getMedium(legs[0], n, leg_0);
				getMedium(legs[3], n, leg_3);
				n++;
			}
			catch (tf2::TransformException &e)
			{
				RCLCPP_ERROR(node_->get_logger(), "Can not get all legs pose %s", e.what());
				return BT::NodeStatus::FAILURE;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		setOutput("legs", legs);
		return BT::NodeStatus::SUCCESS;
	}

	void GetLegs::getMedium(geometry_msgs::msg::Point& leg, int n, geometry_msgs::msg::TransformStamped& new_leg) {
		leg.x = (leg.x * n + new_leg.transform.translation.x)/(n+1);
		leg.y = (leg.y * n + new_leg.transform.translation.y)/(n+1);
		leg.z = (leg.z * n + new_leg.transform.translation.z)/(n+1);
	}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetLegs>("GetLegs");
}