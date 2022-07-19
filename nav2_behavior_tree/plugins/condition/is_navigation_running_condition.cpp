#include <string>

#include "nav2_behavior_tree/plugins/condition/is_navigation_running_condition.hpp"

namespace nav2_behavior_tree
{

IsNavigationRunningCondition::IsNavigationRunningCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  navigation_status_topic_("/navigation_status"),
  is_running_(false)
{
  getInput("navigation_status_topic", navigation_status_topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  navigation_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    navigation_status_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsNavigationRunningCondition::navigationCallback, this, std::placeholders::_1));
}

BT::NodeStatus IsNavigationRunningCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_running_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsNavigationRunningCondition::navigationCallback(std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_running_ = msg->data == 1;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsNavigationRunningCondition>("IsNavigationRunning");
}
