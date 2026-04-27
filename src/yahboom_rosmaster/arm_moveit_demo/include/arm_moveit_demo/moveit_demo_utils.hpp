#pragma once

#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace arm_moveit_demo
{
class ScopedMoveItNode
{
public:
  ScopedMoveItNode(int argc, char** argv, const std::string& node_name)
  {
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared(
      node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    executor_.add_node(node_);
    spinner_ = std::thread([this]() { executor_.spin(); });
  }

  ~ScopedMoveItNode()
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

inline bool succeeded(const moveit::core::MoveItErrorCode& code)
{
  return static_cast<bool>(code);
}
}  // namespace arm_moveit_demo
