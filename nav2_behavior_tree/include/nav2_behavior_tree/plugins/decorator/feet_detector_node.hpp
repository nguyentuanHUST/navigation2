// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__FEET_DETECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__FEET_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <deque>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class FeetDetector : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::FeetDetector
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  FeetDetector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "goal",
        "Goal to navigate to"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_leg0(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_updated_leg1(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_updated_leg2(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_updated_leg3(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leg0_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leg1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leg2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leg3_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  bool leg0_received{false};
  bool leg1_received{false};
  bool leg2_received{false};
  bool leg3_received{false};
  bool get_all_legs{false};

  geometry_msgs::msg::PointStamped leg0_;
  geometry_msgs::msg::PointStamped leg1_;
  geometry_msgs::msg::PointStamped leg2_;
  geometry_msgs::msg::PointStamped leg3_;


  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  bool goals_calculated_{false};
  int current_goal_index_;
  double robot_width_;
  double robot_length_;
  double right_side_offset_;
  double left_side_offset_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
