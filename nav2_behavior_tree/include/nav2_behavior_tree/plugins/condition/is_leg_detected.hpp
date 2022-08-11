#pragma once

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{
	class IsLegDetected : public BT::ConditionNode
	{
	public:
		IsLegDetected(
			const std::string &condition_name,
			const BT::NodeConfiguration &conf);
		BT::NodeStatus tick() override;
		void legDetectedCallback(std_msgs::msg::Bool::SharedPtr msg);
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<std::string>(
					"topic", std::string("/leg_detection_status"), "leg detection topic"),
			};
		}

	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::CallbackGroup::SharedPtr callback_group_;
		rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leg_status_sub_;
		std::string leg_status_topic_;
		bool leg_detection_status_;
	};
}