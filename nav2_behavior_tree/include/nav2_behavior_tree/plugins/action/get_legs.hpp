#pragma once
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class GetLegs : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePath constructor
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GetLegs(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("legs", "Four legs transform in medium"),
      BT::InputPort<int>("nums", 1, "Number of medium times"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
	void getMedium(geometry_msgs::msg::Point&, int n, geometry_msgs::msg::TransformStamped&);
	rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
	int num_medium_;
};

}  // namespace nav2_behavior_tree
