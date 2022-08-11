#include "nav2_behavior_tree/plugins/condition/is_leg_detected.hpp"

namespace nav2_behavior_tree
{
	IsLegDetected::IsLegDetected(
		const std::string &condition_name,
		const BT::NodeConfiguration &conf):
		BT::ConditionNode(condition_name, conf),
		leg_detection_status_(false) {
		getInput("topic", leg_status_topic_);
		node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		callback_group_ = node_->create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive,
			false);
		callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

		rclcpp::SubscriptionOptions sub_option;
		sub_option.callback_group = callback_group_;
		leg_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
			leg_status_topic_,
			rclcpp::SystemDefaultsQoS(),
			std::bind(&IsLegDetected::legDetectedCallback, this, std::placeholders::_1),
			sub_option);

	}
	BT::NodeStatus IsLegDetected::tick() {
		RCLCPP_INFO(node_->get_logger(), "IsLegDetected ticks");
		callback_group_executor_.spin_some();
		if (leg_detection_status_) {
			return BT::NodeStatus::SUCCESS;
		}
		return BT::NodeStatus::FAILURE;
	}
	void IsLegDetected::legDetectedCallback(std_msgs::msg::Bool::SharedPtr msg) {
		leg_detection_status_ = msg->data;
	}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsLegDetected>("IsLegDetected");
}
