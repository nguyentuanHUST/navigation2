#include "nav2_behavior_tree/plugins/action/dynamic_reconfigure_service.hpp"

namespace nav2_behavior_tree
{
	ReconfigureService::ReconfigureService(
		const std::string &service_node_name,
		const BT::NodeConfiguration &conf):
		BtServiceNode<rcl_interfaces::srv::SetParameters>(service_node_name, conf) 
		{
		}
	void ReconfigureService::on_tick() {
		double value;
		std::string name;
		getInput("param_value", value);
		getInput("param_name", name);
		rcl_interfaces::msg::Parameter param;
		param.name = name;
		param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
		param.value.double_value = value;
		request_->parameters.push_back(param);
	}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ReconfigureService>("ReconfigureService");
}
