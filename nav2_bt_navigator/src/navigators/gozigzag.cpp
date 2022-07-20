// Copyright (c) 2021 Samsung Research
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/gozigzag.hpp"


namespace nav2_bt_navigator
{

bool
GoZigZag::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  (void)odom_smoother;
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}

std::string
GoZigZag::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  
  if (!node->has_parameter("default_go_zigzag_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_go_zigzag_bt_xml",
      pkg_share_dir +
      "/behavior_trees/zigzag.xml");
  }
  
  node->get_parameter("default_go_zigzag_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
GoZigZag::cleanup()
{
  self_client_.reset();
  return true;
}

bool
GoZigZag::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Goal received %d %f %f %f", goal->nums, goal->x, goal->y, goal->speed);
  initializeGoal(goal);

  return true;
}

void
GoZigZag::goalCompleted(typename ActionT::Result::SharedPtr /*result*/)
{
}

void
GoZigZag::onLoop()
{
}

void
GoZigZag::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  (void)goal;
//   RCLCPP_INFO(logger_, "Received goal preemption request");

//   if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
//     (goal->behavior_tree.empty() &&
//     bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
//   {
//     // if pending goal requests the same BT as the current goal, accept the pending goal
//     // if pending goal has an empty behavior_tree field, it requests the default BT file
//     // accept the pending goal if the current goal is running the default BT file
//     initializeGoalPose(bt_action_server_->acceptPendingGoal());
//   } else {
//     RCLCPP_WARN(
//       logger_,
//       "Preemption request was rejected since the requested BT XML file is not the same "
//       "as the one that the current goal is executing. Preemption with a new BT is invalid "
//       "since it would require cancellation of the previous goal instead of true preemption."
//       "\nCancel the current goal and send a new action request if you want to use a "
//       "different BT XML file. For now, continuing to track the last goal until completion.");
//     bt_action_server_->terminatePendingGoal();
//   }
}

void
GoZigZag::initializeGoal(ActionT::Goal::ConstSharedPtr goal)
{
  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("nums", goal->nums);  // NOLINT
  blackboard->set<double>("x", goal->x);
  blackboard->set<double>("y", goal->y);
  blackboard->set<double>("speed", goal->speed);
  blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10000));

  // Update the goal pose on the blackboard
}


}  // namespace nav2_bt_navigator
