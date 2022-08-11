#pragma once

#include <string>
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

namespace nav2_behavior_tree
{
	class ReconfigureService : public BtServiceNode<rcl_interfaces::srv::SetParameters>
	{
		public:
		ReconfigureService(
			const std::string & service_node_name,
    	const BT::NodeConfiguration & conf
		);
		void on_tick() override;
		static BT::PortsList providedPorts()
  		{
			return BtServiceNode::providedBasicPorts({
				BT::InputPort<double>("param_value", "Value of param"),
				BT::InputPort<std::string>("param_name", "Name of param"),
			});
		}
	};
}