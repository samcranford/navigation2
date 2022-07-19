#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_RUNNING_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_RUNNING_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class IsNavigationRunningCondition : public BT::ConditionNode
{
public:
  IsNavigationRunningCondition (
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsNavigationRunningCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "navigation_status_topic", std::string("/navigation_status"), "Navigation Status topic"),
    };
  }

private:
  void navigationCallback(std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navigation_sub_;
  std::string navigation_status_topic_;
  bool is_running_;
  std::mutex mutex_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_RUNNING_CONDITION_HPP_
