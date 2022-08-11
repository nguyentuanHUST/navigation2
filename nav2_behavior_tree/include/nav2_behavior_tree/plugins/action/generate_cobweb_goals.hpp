#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class GenerateCobWebGoals : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePath constructor
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GenerateCobWebGoals(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::Point>>("legs", "Four legs transform in medium"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("cobweb_goals", "Cobweb goal"),
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
	rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double robot_width_;
  double robot_length_;
  double inflation_radius_;
  std::vector<geometry_msgs::msg::Point> legs_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace nav2_behavior_tree
