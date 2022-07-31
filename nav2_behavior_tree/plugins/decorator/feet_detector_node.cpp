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
        tf2::Vector3 leg10_v(leg0_.point.x - leg1_.point.x, leg0_.point.y - leg1_.point.y, 0);
        tf2::Vector3 leg21_v(leg1_.point.x - leg2_.point.x, leg1_.point.y - leg2_.point.y, 0);
        tf2::Vector3 leg30_v(leg0_.point.x - leg3_.point.x, leg0_.point.y - leg3_.point.y, 0);
        RCLCPP_INFO(node_->get_logger(),"leg30 %f %f %f", leg30_v.getX(),leg30_v.getY(),leg30_v.getZ() );
        tf2::Vector3 leg2_v(leg2_.point.x, leg2_.point.y, 0);
        tf2::Vector3 leg1_v(leg1_.point.x, leg1_.point.y, 0);
        double yaw = tf2::tf2Angle(leg23_v, tf2::Vector3(1, 0, 0));
        std::vector<tf2::Transform> below_goals_in_legs_frame;
        std::vector<tf2::Transform> beyond_goals_in_legs_frame;
        std::vector<tf2::Transform> cobweb_goals;
        int k = 0;
        geometry_msgs::msg::TransformStamped base_wheel_to_base_link = tf_->lookupTransform("base_wheel", "base_link", tf2::TimePointZero, tf2::durationFromSec(1));
        double x1 = base_wheel_to_base_link.transform.translation.x;
        RCLCPP_INFO(node_->get_logger(), "x1 %f", x1);
        double x2 = robot_length_ / 2 - x1;
        RCLCPP_INFO(node_->get_logger(), "x2 %f", x2);
        double x3 = robot_length_ - x2;
        RCLCPP_INFO(node_->get_logger(), "x3 %f", x3);
        // Cobweb cleaning
        {
          double safety = 0.06;
          tf2::Vector3 delta_23_s = leg23_v / leg23_v.length() *  (x3 + safety);
          tf2::Vector3 delta_23_l = leg23_v - delta_23_s;
          tf2::Vector3 delta_10_s = leg10_v / leg10_v.length() *  (x3 + safety);
          tf2::Vector3 delta_10_l = leg10_v - delta_10_s;
          while (true)
          {
            double l = right_side_offset_ + robot_width_ * k;
            tf2::Vector3 delta_2 = leg21_v / leg21_v.length() * l;
            tf2::Vector3 delta_3 = leg30_v / leg30_v.length() * l;
            RCLCPP_INFO(node_->get_logger(),"delta_3 %f %f %f", delta_3.getX(),delta_3.getY(),delta_3.getZ() );
            if (l > leg21_v.length() - left_side_offset_)
              break;
            tf2::Transform tf = tf2::Transform::getIdentity();
            // if(k == 0) {
            //   tf.setOrigin(tf2::Vector3(0.15 + 0.06, l, 0));
            // } else {
            tf.setOrigin( leg2_v + delta_23_s + delta_2);
            tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
            // }
            below_goals_in_legs_frame.push_back(tf);
            tf.setOrigin(leg2_v + delta_23_l + delta_3);
            beyond_goals_in_legs_frame.push_back(tf);
            k++;
          }
          {
            tf2::Transform tf = tf2::Transform::getIdentity();
            tf.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
            tf.setOrigin(leg2_v + delta_23_s);
            cobweb_goals.push_back(tf);
            tf.setOrigin(leg2_v + delta_23_l);
            cobweb_goals.push_back(tf);
            tf.setOrigin(leg2_v + delta_23_s);
            cobweb_goals.push_back(tf);
            tf.setOrigin(leg1_v + delta_10_s);
            cobweb_goals.push_back(tf);
            tf.setOrigin(leg1_v + delta_10_l);
            cobweb_goals.push_back(tf);
            tf.setOrigin(leg1_v + delta_10_s);
            cobweb_goals.push_back(tf);
          }   
        }

        nav_msgs::msg::Path path;
        for (unsigned int i = 0; i < cobweb_goals.size(); i++)
        {
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = global_frame_;
          tf2::toMsg(cobweb_goals[i], goal.pose);
          goals_.push_back(goal);
        }
        for (unsigned int i = 0; i < below_goals_in_legs_frame.size();)
        {
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = global_frame_;
          tf2::Transform tf = below_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          tf = beyond_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          if (++i >= below_goals_in_legs_frame.size())
            break;
          tf = beyond_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          tf = below_goals_in_legs_frame[i];
          tf2::toMsg(tf, goal.pose);
          goals_.push_back(goal);
          ++i;
        }
        RCLCPP_INFO(node_->get_logger(), "Goals size %d", static_cast<int>(goals_.size()));
        go_backup_.resize(goals_.size());
        for(unsigned int i = 0; i < go_backup_.size(); i++) {
          go_backup_[i] = false;
        }
        go_backup_[1] = go_backup_[2] = go_backup_[4] = go_backup_[5] = true; //cobweb corner
        go_backup_[7] = go_backup_[go_backup_.size() - 1] = true;

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
        if(i > 0) {
            geometry_msgs::msg::PoseStamped start = goals_[i - 1];
          start.header.stamp = node_->get_clock()->now();
          start.header.frame_id = global_frame_;
          setOutput("start", start);
        }
        if(go_backup_[i]) {
          setOutput("is_not_corner", false);
        } else {
          setOutput("is_not_corner", true);
        }
        child_node_->executeTick();
        while (child_node_->waitValidStatus() == BT::NodeStatus::RUNNING)
        {
          child_node_->executeTick();
          // RCLCPP_INFO(node_->get_logger(), "Child node running");
        }
        RCLCPP_INFO(node_->get_logger(), "Goals %d finish", i);
      }
      // for (int i = goals_.size() - 2; i >= 0; i--)
      // {
      //   RCLCPP_INFO(node_->get_logger(), "Goals %d", i);
      //   geometry_msgs::msg::PoseStamped goal = goals_[i];
      //   goal.header.stamp = node_->get_clock()->now();
      //   goal.header.frame_id = global_frame_;
      //   setOutput("goal", goal);
      //   child_node_->executeTick();
      //   while (child_node_->waitValidStatus() == BT::NodeStatus::RUNNING)
      //   {
      //     child_node_->executeTick();
      //     // RCLCPP_INFO(node_->get_logger(), "Child node running");
      //   }
      //   RCLCPP_INFO(node_->get_logger(), "Goals %d finish", i);
      // }
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
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Set local inflation radius failed");
    }
    set_params_client_.reset();
    set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>
      ("/global_costmap/global_costmap/set_parameters", rmw_qos_profile_services_default, callback_group_);
    future_result = set_params_client_->async_send_request(request);
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Set global inflation radius failed");
    }
  }
  
  double FeetDetector::getInflationRadius()
  {
    double radius = 0.5;
    get_params_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>
      ("/local_costmap/local_costmap/get_parameters", rmw_qos_profile_services_default, callback_group_);
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(std::string("inflation_layer.inflation_radius"));
    auto future_result = get_params_client_->async_send_request(request);
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Get parameter failed");
    }
    radius = future_result.get()->values[0].double_value;
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
