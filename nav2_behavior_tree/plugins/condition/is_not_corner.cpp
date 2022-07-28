#include <string>
#include <chrono>
#include <iostream>
// #include <type_traits>
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_behavior_tree/plugins/condition/is_not_corner.hpp"

namespace nav2_behavior_tree{
IsNotCorner::IsNotCorner(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}
IsNotCorner::~IsNotCorner() {

}
BT::NodeStatus IsNotCorner::tick()
{
    bool is_not_corner;
    getInput<bool>("is_not_corner", is_not_corner);
    if(is_not_corner) {
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "Go backward" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsNotCorner>("IsNotCorner");
}