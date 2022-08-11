#include "nav2_behavior_tree/plugins/decorator/distribute_goal.hpp"
#include "behaviortree_cpp_v3/exceptions.h"

namespace nav2_behavior_tree
{
	DistributeGoal::DistributeGoal(
		const std::string &name,
		const BT::NodeConfiguration &conf)
		: BT::DecoratorNode(name, conf),
		goal_index_(0),
		is_cob_web_(false),
		is_last_goal_success_(false)
	{
		node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
		tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
		getInput("cobweb", is_cob_web_);
	}
	BT::NodeStatus DistributeGoal::tick() {
		// RCLCPP_INFO(node_->get_logger(), "DistributeGoal ticks");
		getInput("goals", goals_);
		setStatus(BT::NodeStatus::RUNNING);
		while (goal_index_ < goals_.size())
		{
			setOutput("current_goal", goals_[goal_index_]);
			if(goal_index_ == 0 || !is_last_goal_success_) {
				geometry_msgs::msg::TransformStamped tf = tf_->lookupTransform("map", 
					"base_wheel", tf2::TimePointZero, tf2::durationFromSec(1));
				geometry_msgs::msg::PoseStamped robot_pose;
				robot_pose.header.frame_id = "map";
				robot_pose.pose.position.x = tf.transform.translation.x;
				robot_pose.pose.position.y = tf.transform.translation.y;
				setOutput("current_start",robot_pose);
			} else {
				setOutput("current_start", goals_[goal_index_ - 1]);
			}
			if(is_cob_web_) {
				std::vector<uint32_t> backup_idx{1,2,4,5};
				if(std::count(backup_idx.begin(), backup_idx.end(), goal_index_)) {
					setOutput("backup_dist", 0.2);
				} else {
					setOutput("backup_dist", 0.0);
				}
			} else {
				std::vector<uint32_t> backup_idx{1, static_cast<uint32_t>(goals_.size()) - 2};
				if(std::count(backup_idx.begin(), backup_idx.end(), goal_index_)) {
					setOutput("backup_dist", 0.2);
				} else {
					setOutput("backup_dist", 0.0);
				}
			}
			BT::NodeStatus child_state = child_node_->executeTick();
			switch (child_state)
			{
			case BT::NodeStatus::SUCCESS:
			{
				RCLCPP_INFO(node_->get_logger(), "goal success %d", goal_index_);
				goal_index_++;
				haltChild();
				is_last_goal_success_ = true;
			}
			break;
			case BT::NodeStatus::RUNNING:
			{
				return BT::NodeStatus::RUNNING;
			}
			break;
			case BT::NodeStatus::FAILURE:
			{
				goal_index_++;
				haltChild();
				is_last_goal_success_ = false;
			}
			break;
			default:
				throw BT::LogicError("A child node must never return IDLE");
				break;
			}
		}
		goal_index_ = 0;
		return BT::NodeStatus::SUCCESS;
	}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistributeGoal>("DistributeGoal");
}
