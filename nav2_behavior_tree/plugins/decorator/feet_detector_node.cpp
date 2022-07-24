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

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "tf2_eigen/tf2_eigen.h"

#include "nav2_behavior_tree/plugins/decorator/feet_detector_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

  using std::placeholders::_1;
  using namespace std::chrono_literals;

  FeetDetector::FeetDetector(
      const std::string &name,
      const BT::NodeConfiguration &conf)
      : BT::DecoratorNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    // TODO: Get robot dimension
    node_->declare_parameter("robot_width", rclcpp::ParameterValue(0.33));
    node_->get_parameter("robot_width", robot_width_);
    node_->declare_parameter("robot_length", rclcpp::ParameterValue(0.61));
    node_->get_parameter("robot_length", robot_length_);
    node_->declare_parameter("lidar_frame", rclcpp::ParameterValue("lidar_1"));
    node_->get_parameter("lidar_frame", robot_base_frame_);
    node_->declare_parameter("map_frame", rclcpp::ParameterValue("map"));
    node_->get_parameter("map_frame", global_frame_);
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
    node_->get_parameter("transform_tolerance", transform_tolerance_);
    node_->declare_parameter("right_offset", rclcpp::ParameterValue(0.0));
    node_->get_parameter("right_offset", right_side_offset_);
    node_->declare_parameter("left_offset", rclcpp::ParameterValue(0.0));
    node_->get_parameter("left_offset", left_side_offset_);
    node_->declare_parameter("inflation_radius", rclcpp::ParameterValue(0.05));
    node_->get_parameter("inflation_radius", inflation_radius_);

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    leg0_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "leg_0",
        10,
        std::bind(&FeetDetector::callback_updated_leg0, this, _1),
        sub_option);
    leg1_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "leg_1",
        10,
        std::bind(&FeetDetector::callback_updated_leg1, this, _1),
        sub_option);
    leg2_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "leg_2",
        10,
        std::bind(&FeetDetector::callback_updated_leg2, this, _1),
        sub_option);
    leg3_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "leg_3",
        10,
        std::bind(&FeetDetector::callback_updated_leg3, this, _1),
        sub_option);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("path_from_feet", rclcpp::QoS(10));
    RCLCPP_INFO(node_->get_logger(), "Feet detector constructor");
  }

  inline BT::NodeStatus FeetDetector::tick()
  {
    callback_group_executor_.spin_some();
    if (leg0_received && leg1_received && leg2_received && leg3_received)
    {
      if (!get_all_legs)
      {
        try
        {
          geometry_msgs::msg::TransformStamped leg_0 = tf_->lookupTransform(global_frame_, "leg_0", tf2::TimePointZero, tf2::durationFromSec(1));
          leg0_.point.x = leg_0.transform.translation.x;
          leg0_.point.y = leg_0.transform.translation.y;
          leg0_.point.z = leg_0.transform.translation.z;
          geometry_msgs::msg::TransformStamped leg_1 = tf_->lookupTransform(global_frame_, "leg_1", tf2::TimePointZero, tf2::durationFromSec(1));
          leg1_.point.x = leg_1.transform.translation.x;
          leg1_.point.y = leg_1.transform.translation.y;
          leg1_.point.z = leg_1.transform.translation.z;
          geometry_msgs::msg::TransformStamped leg_2 = tf_->lookupTransform(global_frame_, "leg_2", tf2::TimePointZero, tf2::durationFromSec(1));
          leg2_.point.x = leg_2.transform.translation.x;
          leg2_.point.y = leg_2.transform.translation.y;
          leg2_.point.z = leg_2.transform.translation.z;
          geometry_msgs::msg::TransformStamped leg_3 = tf_->lookupTransform(global_frame_, "leg_3", tf2::TimePointZero, tf2::durationFromSec(1));
          leg3_.point.x = leg_3.transform.translation.x;
          leg3_.point.y = leg_3.transform.translation.y;
          leg3_.point.z = leg_3.transform.translation.z;
          get_all_legs = true;
        }
        catch (tf2::TransformException &e)
        {
          RCLCPP_ERROR(node_->get_logger(), "Can not get all legs pose %s", e.what());
          return BT::NodeStatus::FAILURE;
        }
      }
      if (!goals_calculated_)
      {
        tf2::Vector3 leg23_v(leg3_.point.x - leg2_.point.x, leg3_.point.y - leg2_.point.y, 0);
        double yaw = tf2::tf2Angle(leg23_v, tf2::Vector3(1, 0, 0));
        // RCLCPP_INFO(node_->get_logger(), "leg23_v %f %f", leg23_v.x(), leg23_v.y());
        // RCLCPP_INFO(node_->get_logger(), "yaw %f", yaw);
        tf2::Quaternion leg_23_q(tf2::Vector3(0, 0, 1), yaw);
        tf2::Transform leg_23(leg_23_q, tf2::Vector3(leg2_.point.x, leg2_.point.y, 0));
        std::vector<tf2::Transform> below_goals_in_legs_frame;
        std::vector<tf2::Transform> beyond_goals_in_legs_frame;
        std::vector<tf2::Transform> cobweb_goals;
        int k = 0;
        double table_width_ = std::hypot(leg2_.point.x - leg1_.point.x, leg2_.point.y - leg1_.point.y);
        double table_length = std::hypot(leg23_v.x(), leg23_v.y());
        geometry_msgs::msg::TransformStamped base_wheel_to_base_link = tf_->lookupTransform("base_wheel", "base_link", tf2::TimePointZero, tf2::durationFromSec(1));
        double x1 = base_wheel_to_base_link.transform.translation.x;
        RCLCPP_INFO(node_->get_logger(), "x1 %f", x1);
        double x2 = robot_length_ / 2 - x1;
        RCLCPP_INFO(node_->get_logger(), "x2 %f", x2);
        double x3 = robot_length_ - x2;
        RCLCPP_INFO(node_->get_logger(), "x3 %f", x3);
        // Cobweb cleaning
        {
          double safety = 0.04;
          tf2::Transform tf = tf2::Transform::getIdentity();
          tf.setOrigin(tf2::Vector3(x3 + safety, right_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
          cobweb_goals.push_back(tf);
          tf.setOrigin(tf2::Vector3(table_length - (x3 + safety), right_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
          cobweb_goals.push_back(tf);
          tf.setOrigin(tf2::Vector3(x3 + safety, right_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI));
          cobweb_goals.push_back(tf);
          tf.setOrigin(tf2::Vector3(x3 + safety, table_width_ - left_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
          cobweb_goals.push_back(tf);
          tf.setOrigin(tf2::Vector3(table_length - (x3 + safety), table_width_ - left_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
          cobweb_goals.push_back(tf);
          tf.setOrigin(tf2::Vector3(x3 + safety, table_width_ - left_side_offset_, 0));
          tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI));
          cobweb_goals.push_back(tf);
        }
        while (true)
        {
          double l = right_side_offset_ + robot_width_ * k;
          if (l > table_width_ - left_side_offset_)
            break;
          tf2::Transform tf = tf2::Transform::getIdentity();
          // if(k == 0) {
          //   tf.setOrigin(tf2::Vector3(0.15 + 0.06, l, 0));
          // } else {
          tf.setOrigin(tf2::Vector3(0.52 + 0.06, l, 0));
          // }
          below_goals_in_legs_frame.push_back(tf);
          tf.setOrigin(tf2::Vector3(table_length - 0.52 - 0.06, l, 0));
          beyond_goals_in_legs_frame.push_back(tf);
          k++;
        }

        for (unsigned int i = 0; i < below_goals_in_legs_frame.size();)
        {
          below_goals_in_legs_frame[i].setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
          i++;
          if (i >= below_goals_in_legs_frame.size())
            break;
          below_goals_in_legs_frame[i].setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2));
          i++;
        }
        for (unsigned int i = 0; i < beyond_goals_in_legs_frame.size();)
        {
          beyond_goals_in_legs_frame[i].setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2));
          i++;
          if (i >= beyond_goals_in_legs_frame.size())
            break;
          beyond_goals_in_legs_frame[i].setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI));
          i++;
        }
        // TODO: Calculate list goal from table size and robot size
        nav_msgs::msg::Path path;
        for (unsigned int i = 0; i < cobweb_goals.size(); i++)
        {
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = global_frame_;
          tf2::Transform tf = leg_23 * cobweb_goals[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
        }
        for (unsigned int i = 0; i < below_goals_in_legs_frame.size();)
        {
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = global_frame_;
          tf2::Transform tf = leg_23 * below_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          tf = leg_23 * beyond_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);

          if (++i >= below_goals_in_legs_frame.size())
            break;
          tf = leg_23 * beyond_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          tf = leg_23 * below_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          ++i;
        }
        RCLCPP_INFO(node_->get_logger(), "Goals size %d", static_cast<int>(goals_.size()));
        path.poses = goals_;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->get_clock()->now();
        path_pub_->publish(path);
      }
      double last_radius = getInflationRadius();
      reconfigInflationRadius(inflation_radius_);
      for (unsigned int i = 0; i < goals_.size(); i++)
      {
        RCLCPP_INFO(node_->get_logger(), "Goals %d", i);
        geometry_msgs::msg::PoseStamped goal = goals_[i];
        goal.header.stamp = node_->get_clock()->now();
        goal.header.frame_id = global_frame_;
        setOutput("goal", goal);
        child_node_->executeTick();
        while (child_node_->waitValidStatus() == BT::NodeStatus::RUNNING)
        {
          child_node_->executeTick();
          // RCLCPP_INFO(node_->get_logger(), "Child node running");
        }
        RCLCPP_INFO(node_->get_logger(), "Goals %d finish", i);
      }
      reconfigInflationRadius(last_radius);
      goals_.clear();
      leg0_received = leg1_received = leg2_received = leg3_received = get_all_legs = false;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Legs not detected");
      return BT::NodeStatus::RUNNING;
    }
  }

  void FeetDetector::reconfigInflationRadius(double radius)
  {
    set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>
      ("/local_costmap/local_costmap/set_parameters", rmw_qos_profile_services_default, callback_group_);
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = std::string("inflation_layer.inflation_radius");
    param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = radius;
    request->parameters.push_back(param);
    auto future_result = set_params_client_->async_send_request(request);
    callback_group_executor_.spin_some();
    auto result = future_result.wait_for(10s);
    if (result == std::future_status::timeout)
    {
      RCLCPP_ERROR(node_->get_logger(), "feet_detector_node dynamic reconfigure failed");
    }

    if (result == std::future_status::ready)
    {
      std::vector<rcl_interfaces::msg::SetParametersResult> set_results = future_result.get()->results;
      for (unsigned int i = 0; i < set_results.size(); i++)
      {
        if (!set_results[i].successful)
        {
          RCLCPP_ERROR(node_->get_logger(), "Dynamic reconfigure param %s failed reason: %s", request->parameters[i].name.c_str(), set_results[i].reason.c_str());
        }
      }
    }
    set_params_client_.reset();
  }
  
  double FeetDetector::getInflationRadius()
  {
    double radius = 0.5;
    get_params_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>
      ("/local_costmap/local_costmap/get_parameters", rmw_qos_profile_services_default, callback_group_);
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(std::string("inflation_layer.inflation_radius"));
    auto future_result = get_params_client_->async_send_request(request);
    callback_group_executor_.spin_some();
    auto result = future_result.wait_for(10s);
    if (result == std::future_status::timeout)
    {
      RCLCPP_ERROR(node_->get_logger(), "feet_detector_node get param failed");
    }

    if (result == std::future_status::ready)
    {
      std::vector<rcl_interfaces::msg::ParameterValue> get_results = future_result.get()->values;
      radius = get_results[0].double_value;
    }
    return radius;
  }

  void
  FeetDetector::callback_updated_leg0(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    (void)msg;
    // TODO: Convert coord to frame map
    //  leg0_ = *msg;
    leg0_received = true;
  }

  void
  FeetDetector::callback_updated_leg1(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    (void)msg;
    // leg1_ = *msg;
    leg1_received = true;
  }

  void
  FeetDetector::callback_updated_leg2(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    (void)msg;
    // leg2_ = *msg;
    leg2_received = true;
  }

  void
  FeetDetector::callback_updated_leg3(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    (void)msg;
    // leg3_ = *msg;
    leg3_received = true;
  }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::FeetDetector>("FeetDetector");
}
