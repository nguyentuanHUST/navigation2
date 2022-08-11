#pragma once 
#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate
 */
class DistributeGoal : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RateController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  DistributeGoal(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "Goals list to publish"),
			BT::InputPort<bool>("cobweb", false, "Whether cobweb or zigzag"),
			BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_goal", "Current goal to publish"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_start", "Current start to publish"),
			BT::OutputPort<double>("backup_dist", "Backup distance"),
      // BT::OutputPort<double>("rot_angle", "Backup distance"),
			BT::OutputPort<bool>("brush_up", "Whether to control brush up"),
			BT::OutputPort<bool>("use_beam_alignment", "Whether to use camera to align above beam"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

	rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
	std::vector<geometry_msgs::msg::PoseStamped> goals_;
	uint32_t goal_index_;
	bool is_cob_web_;
  bool is_last_goal_success_;
};
}