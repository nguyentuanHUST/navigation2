#pragma once

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{
	class IsNotCorner : public BT::ConditionNode
	{
	public:
		IsNotCorner(
				const std::string &condition_name,
				const BT::NodeConfiguration &conf);
		IsNotCorner() = delete;
		~IsNotCorner() override;
		BT::NodeStatus tick() override;
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<bool>("is_not_corner", true, 
					"Check whether is corner of the rack")
			};
		}
	};
}