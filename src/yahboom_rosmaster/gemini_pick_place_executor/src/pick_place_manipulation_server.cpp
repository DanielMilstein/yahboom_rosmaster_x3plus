#include <algorithm>
#include <chrono>
#include <fstream>
#include <cmath>
#include <ctime>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yahboom_rosmaster_msgs/action/pick_place_manipulation.hpp>

using PickPlaceManipulation = yahboom_rosmaster_msgs::action::PickPlaceManipulation;
using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlaceManipulation>;

namespace
{
constexpr const char* kArmGroup = "arm_group";
constexpr const char* kGripGroup = "grip_group";
constexpr const char* kObjectId = "gemini_pick_target";
constexpr const char* kDestinationId = "gemini_pick_destination";
constexpr const char* kAttachLink = "arm_link5";
constexpr const char* kTaskFrame = "base_footprint";

bool succeeded(const moveit::core::MoveItErrorCode& code)
{
  return static_cast<bool>(code);
}

bool label_is_cylinder_like(const std::string& label)
{
  std::string lower = label;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lower.find("can") != std::string::npos || lower.find("bottle") != std::string::npos ||
         lower.find("cylinder") != std::string::npos;
}
}  // namespace

class PickPlaceManipulationServer : public std::enable_shared_from_this<PickPlaceManipulationServer>
{
public:
  explicit PickPlaceManipulationServer(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    node_->declare_parameter("approach_offset_m", 0.02);
    node_->declare_parameter("retreat_offset_m", 0.08);
    node_->declare_parameter("tool_contact_offset_m", 0.09);
    node_->declare_parameter("fallback_tool_contact_offset_m", 0.11);
    node_->declare_parameter("grasp_tool_offset_x_m", -0.09);
    node_->declare_parameter("grasp_tool_offset_z_m", 0.09);
    node_->declare_parameter("place_tool_offset_x_m", -0.09);
    node_->declare_parameter("place_tool_offset_z_m", 0.09);
    node_->declare_parameter("fallback_grasp_tool_offset_z_m", 0.11);
    node_->declare_parameter("pre_grasp_standoff_x_m", 0.187);
    node_->declare_parameter("pre_grasp_standoff_z_m", 0.48);
    node_->declare_parameter("try_top_down_first", true);
    node_->declare_parameter("try_diagonal_first", true);
    node_->declare_parameter("try_center_position_first", true);
    node_->declare_parameter("center_pre_grasp_clearance_m", 0.12);
    node_->declare_parameter("center_grasp_z_offset_m", 0.03);
    node_->declare_parameter("center_place_clearance_m", 0.12);
    node_->declare_parameter("diagonal_approach_offset_m", 0.08);
    node_->declare_parameter("diagonal_lift_offset_m", 0.12);
    node_->declare_parameter("diagonal_tool_offset_m", 0.09);
    node_->declare_parameter("diagonal_tool_offset_z_m", 0.10);
    node_->declare_parameter("top_down_tool_offset_z_m", 0.09);
    node_->declare_parameter("top_down_pre_grasp_clearance_m", 0.02);
    node_->declare_parameter("top_down_place_clearance_m", 0.02);
    node_->declare_parameter("existing_target_object_id", "task_can");
    node_->declare_parameter("add_destination_collision_object", false);
    node_->declare_parameter("max_grasp_width_m", 0.13);
    node_->declare_parameter("min_object_dimension_m", 0.01);
    node_->declare_parameter("planning_time_sec", 2.0);
    node_->declare_parameter("planning_attempts", 3);
    node_->declare_parameter("goal_position_tolerance_m", 0.015);
    node_->declare_parameter("goal_orientation_tolerance_rad", 0.08);
    node_->declare_parameter("prefer_position_only_targets", false);
    node_->declare_parameter("allow_position_only_fallback", false);
    node_->declare_parameter("allow_sampled_position_fallback", false);
    node_->declare_parameter("sampled_position_attempts", 2400);
    node_->declare_parameter("sampled_position_tolerance_m", 0.05);
    node_->declare_parameter("sampled_grasp_tolerance_m", 0.015);
    node_->declare_parameter("sampled_place_tolerance_m", 0.025);
    node_->declare_parameter("use_named_prepose_fallback", true);
    node_->declare_parameter("named_prepose", "init");
    node_->declare_parameter("allow_joint_tabletop_fallback", false);
    node_->declare_parameter("debug_pose_log_dir", "/tmp");
    node_->declare_parameter("velocity_scaling", 0.35);
    node_->declare_parameter("acceleration_scaling", 0.35);
    node_->declare_parameter("enable_base_reposition", true);
    node_->declare_parameter("base_cmd_vel_topic", "/mecanum_drive_controller/cmd_vel");
    node_->declare_parameter("base_target_x_m", 0.34);
    node_->declare_parameter("base_target_y_m", 0.0);
    node_->declare_parameter("base_target_x_tolerance_m", 0.03);
    node_->declare_parameter("base_target_y_tolerance_m", 0.03);
    node_->declare_parameter("base_max_adjust_x_m", 0.25);
    node_->declare_parameter("base_max_adjust_y_m", 0.16);
    node_->declare_parameter("base_max_reposition_nudges", 4);
    node_->declare_parameter("base_reach_min_x_m", 0.16);
    node_->declare_parameter("base_reach_max_x_m", 0.42);
    node_->declare_parameter("base_reach_max_abs_y_m", 0.15);
    node_->declare_parameter("base_max_linear_x_mps", 0.06);
    node_->declare_parameter("base_max_linear_y_mps", 0.05);
    node_->declare_parameter("base_command_rate_hz", 20.0);
    node_->declare_parameter("eef_qx", 0.08637313729188083);
    node_->declare_parameter("eef_qy", 0.40819551903689105);
    node_->declare_parameter("eef_qz", 0.10050372779920543);
    node_->declare_parameter("eef_qw", 0.9032248336328139);
    node_->declare_parameter("diagonal_pitch_offset_rad", -0.7853981633974483);
    node_->declare_parameter("top_down_pitch_offset_rad", -1.5707963267948966);

    action_server_ = rclcpp_action::create_server<PickPlaceManipulation>(
      node_,
      "pick_place_manipulation",
      std::bind(&PickPlaceManipulationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceManipulationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceManipulationServer::handle_accepted, this, std::placeholders::_1));

    base_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      node_->get_parameter("base_cmd_vel_topic").as_string(), 10);
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PickPlaceManipulation::Goal> goal)
  {
    if (!valid_dimensions(goal->target_dimensions) || !valid_dimensions(goal->destination_dimensions)) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting manipulation goal with invalid dimensions");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickPlace>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    std::thread([self = shared_from_this(), goal_handle]() { self->execute(goal_handle); }).detach();
  }

  bool valid_dimensions(const geometry_msgs::msg::Vector3& dimensions) const
  {
    const double minimum = node_->get_parameter("min_object_dimension_m").as_double();
    return std::isfinite(dimensions.x) && std::isfinite(dimensions.y) && std::isfinite(dimensions.z) &&
           dimensions.x >= minimum && dimensions.y >= minimum && dimensions.z >= 0.0;
  }

  geometry_msgs::msg::Pose make_pose(const geometry_msgs::msg::Point& point) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation.x = node_->get_parameter("eef_qx").as_double();
    pose.orientation.y = node_->get_parameter("eef_qy").as_double();
    pose.orientation.z = node_->get_parameter("eef_qz").as_double();
    pose.orientation.w = node_->get_parameter("eef_qw").as_double();
    return pose;
  }

  geometry_msgs::msg::Pose make_pitch_offset_pose(
    const geometry_msgs::msg::Point& point,
    double pitch_offset) const
  {
    auto pose = make_pose(point);
    const double half_pitch = pitch_offset * 0.5;
    const double rx = 0.0;
    const double ry = std::sin(half_pitch);
    const double rz = 0.0;
    const double rw = std::cos(half_pitch);
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;

    pose.orientation.x = qw * rx + qx * rw + qy * rz - qz * ry;
    pose.orientation.y = qw * ry - qx * rz + qy * rw + qz * rx;
    pose.orientation.z = qw * rz + qx * ry - qy * rx + qz * rw;
    pose.orientation.w = qw * rw - qx * rx - qy * ry - qz * rz;

    const double norm = std::sqrt(
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w);
    if (norm > 1.0e-9) {
      pose.orientation.x /= norm;
      pose.orientation.y /= norm;
      pose.orientation.z /= norm;
      pose.orientation.w /= norm;
    }
    return pose;
  }

  geometry_msgs::msg::Point offset_from_contact(
    const geometry_msgs::msg::Point& contact,
    double approach_x,
    double approach_y,
    double approach_z,
    double offset) const
  {
    const double norm = std::sqrt(
      approach_x * approach_x + approach_y * approach_y + approach_z * approach_z);
    if (norm <= 1.0e-9) {
      return contact;
    }

    geometry_msgs::msg::Point point = contact;
    point.x -= approach_x / norm * offset;
    point.y -= approach_y / norm * offset;
    point.z -= approach_z / norm * offset;
    return point;
  }

  geometry_msgs::msg::Point approach_contact_point(
    const geometry_msgs::msg::Point& contact,
    double approach_x,
    double approach_y,
    double approach_z,
    double distance) const
  {
    return offset_from_contact(contact, approach_x, approach_y, approach_z, distance);
  }

  geometry_msgs::msg::Pose make_approach_relative_pose(
    const geometry_msgs::msg::Point& contact,
    double approach_x,
    double approach_y,
    double approach_z,
    double tool_offset,
    double pitch_offset = 0.0) const
  {
    return make_pitch_offset_pose(
      offset_from_contact(contact, approach_x, approach_y, approach_z, tool_offset),
      pitch_offset);
  }

  std::vector<geometry_msgs::msg::Pose> generate_poses(
    const PickPlaceManipulation::Goal& goal,
    double tool_offset) const
  {
    const double approach = node_->get_parameter("approach_offset_m").as_double();
    const double retreat = node_->get_parameter("retreat_offset_m").as_double();

    constexpr double approach_x = 1.0;
    constexpr double approach_y = 0.0;
    constexpr double approach_z = 0.0;

    const auto pre_grasp_contact = approach_contact_point(
      goal.target_center, approach_x, approach_y, approach_z, approach);
    auto pre_grasp = make_approach_relative_pose(
      pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset);

    auto grasp = make_approach_relative_pose(
      goal.target_center, approach_x, approach_y, approach_z, tool_offset);

    auto lift = grasp;
    lift.position.z += retreat;

    auto pre_place_contact = goal.destination_center;
    pre_place_contact.z += retreat;
    auto pre_place = make_approach_relative_pose(
      pre_place_contact, approach_x, approach_y, approach_z, tool_offset);

    auto place = make_approach_relative_pose(
      goal.destination_center, approach_x, approach_y, approach_z, tool_offset);

    return {
      pre_grasp,
      grasp,
      lift,
      pre_place,
      place,
    };
  }

  std::vector<std::pair<std::string, std::vector<geometry_msgs::msg::Pose>>> generate_pose_candidates(
    const PickPlaceManipulation::Goal& goal) const
  {
    std::vector<std::pair<std::string, std::vector<geometry_msgs::msg::Pose>>> candidates;
    const double tool_offset = node_->get_parameter("tool_contact_offset_m").as_double();

    if (node_->get_parameter("try_center_position_first").as_bool()) {
      const double pre_clearance = node_->get_parameter("center_pre_grasp_clearance_m").as_double();
      const double grasp_z_offset = node_->get_parameter("center_grasp_z_offset_m").as_double();
      const double place_clearance = node_->get_parameter("center_place_clearance_m").as_double();
      constexpr double approach_x = 0.0;
      constexpr double approach_y = 0.0;
      constexpr double approach_z = -1.0;

      auto pre_grasp_contact = goal.target_center;
      pre_grasp_contact.z += pre_clearance;

      auto grasp_contact = goal.target_center;
      grasp_contact.z += grasp_z_offset;

      auto pre_place_contact = goal.destination_center;
      pre_place_contact.z += place_clearance;

      auto place_contact = goal.destination_center;
      place_contact.z += grasp_z_offset;

      candidates.emplace_back("center_position_only", std::vector<geometry_msgs::msg::Pose>{
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset),
        make_approach_relative_pose(grasp_contact, approach_x, approach_y, approach_z, tool_offset),
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset),
        make_approach_relative_pose(pre_place_contact, approach_x, approach_y, approach_z, tool_offset),
        make_approach_relative_pose(place_contact, approach_x, approach_y, approach_z, tool_offset),
      });
    }

    if (node_->get_parameter("try_diagonal_first").as_bool()) {
      const double approach = node_->get_parameter("diagonal_approach_offset_m").as_double();
      const double lift = node_->get_parameter("diagonal_lift_offset_m").as_double();
      const double pitch_offset = node_->get_parameter("diagonal_pitch_offset_rad").as_double();
      constexpr double approach_x = 1.0;
      constexpr double approach_y = 0.0;
      constexpr double approach_z = -1.0;

      const auto pre_grasp_contact = approach_contact_point(
        goal.target_center, approach_x, approach_y, approach_z, approach);
      const auto lift_contact = approach_contact_point(
        goal.target_center, approach_x, approach_y, approach_z, lift);
      auto pre_place_contact = approach_contact_point(
        goal.destination_center, approach_x, approach_y, approach_z, approach * 0.5);
      pre_place_contact.z += lift;

      std::vector<geometry_msgs::msg::Pose> diagonal{
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(goal.target_center, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(lift_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(pre_place_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(goal.destination_center, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
      };
      candidates.emplace_back("diagonal", diagonal);

      std::vector<geometry_msgs::msg::Pose> diagonal_alt{
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(goal.target_center, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(lift_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(pre_place_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(goal.destination_center, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
      };
      candidates.emplace_back("diagonal_alt", diagonal_alt);

      auto diagonal_high = diagonal;
      diagonal_high[0].position.z += 0.03;
      diagonal_high[2].position.z += 0.03;
      diagonal_high[3].position.z += 0.03;
      candidates.emplace_back("diagonal_high", diagonal_high);
    }

    if (node_->get_parameter("try_top_down_first").as_bool()) {
      const double pre_clearance = node_->get_parameter("top_down_pre_grasp_clearance_m").as_double();
      const double place_clearance = node_->get_parameter("top_down_place_clearance_m").as_double();
      const double pitch_offset = node_->get_parameter("top_down_pitch_offset_rad").as_double();
      constexpr double approach_x = 0.0;
      constexpr double approach_y = 0.0;
      constexpr double approach_z = -1.0;

      auto pre_grasp_contact = goal.target_center;
      pre_grasp_contact.z += pre_clearance;
      auto pre_place_contact = goal.destination_center;
      pre_place_contact.z += place_clearance;

      std::vector<geometry_msgs::msg::Pose> top_down{
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(goal.target_center, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(pre_place_contact, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
        make_approach_relative_pose(goal.destination_center, approach_x, approach_y, approach_z, tool_offset, pitch_offset),
      };
      candidates.emplace_back("top_down", top_down);

      std::vector<geometry_msgs::msg::Pose> top_down_alt{
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(goal.target_center, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(pre_grasp_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(pre_place_contact, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
        make_approach_relative_pose(goal.destination_center, approach_x, approach_y, approach_z, tool_offset, -pitch_offset),
      };
      candidates.emplace_back("top_down_alt", top_down_alt);

      auto top_down_high = top_down;
      top_down_high[0].position.z += 0.08;
      top_down_high[2].position.z += 0.08;
      top_down_high[3].position.z += 0.08;
      candidates.emplace_back("top_down_high", top_down_high);
    }

    auto high_standoff = generate_poses(
      goal,
      tool_offset);
    high_standoff.front().position.x = node_->get_parameter("pre_grasp_standoff_x_m").as_double();
    high_standoff.front().position.y = goal.target_center.y;
    high_standoff.front().position.z = node_->get_parameter("pre_grasp_standoff_z_m").as_double();

    candidates.emplace_back("high_standoff", high_standoff);
    candidates.emplace_back("front_back", generate_poses(
      goal,
      tool_offset));
    candidates.emplace_back("front_back_fallback", generate_poses(
      goal,
      node_->get_parameter("fallback_tool_contact_offset_m").as_double()));
    candidates.emplace_back("front_back_low", generate_poses(goal, 0.085));
    return candidates;
  }

  moveit_msgs::msg::CollisionObject make_collision_object(
    const std::string& id,
    const std::string& label,
    const geometry_msgs::msg::Point& center,
    const geometry_msgs::msg::Vector3& dimensions,
    const std::string& frame_id) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header.frame_id = frame_id;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    if (label_is_cylinder_like(label)) {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {
        std::max(dimensions.z, node_->get_parameter("min_object_dimension_m").as_double()),
        std::max(dimensions.x, dimensions.y) * 0.5,
      };
    } else {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {
        std::max(dimensions.x, node_->get_parameter("min_object_dimension_m").as_double()),
        std::max(dimensions.y, node_->get_parameter("min_object_dimension_m").as_double()),
        std::max(dimensions.z, node_->get_parameter("min_object_dimension_m").as_double()),
      };
    }

    geometry_msgs::msg::Pose pose;
    pose.position = center;
    pose.orientation.w = 1.0;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    return object;
  }

  double grasp_width(const PickPlaceManipulation::Goal& goal) const
  {
    if (label_is_cylinder_like(goal.target_label)) {
      return std::min(goal.target_dimensions.x, goal.target_dimensions.y);
    }
    return goal.target_dimensions.y;
  }

  void configure_group(moveit::planning_interface::MoveGroupInterface& group) const
  {
    group.allowReplanning(true);
    group.setPlanningTime(node_->get_parameter("planning_time_sec").as_double());
    group.setNumPlanningAttempts(node_->get_parameter("planning_attempts").as_int());
    group.setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance_m").as_double());
    group.setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance_rad").as_double());
    group.setMaxVelocityScalingFactor(node_->get_parameter("velocity_scaling").as_double());
    group.setMaxAccelerationScalingFactor(node_->get_parameter("acceleration_scaling").as_double());
  }

  bool plan_and_maybe_execute(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& stage_name,
    bool execute_motion,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    PickPlaceManipulation::Result& result)
  {
    publish_feedback(goal_handle, stage_name, "planning");
    arm_group.clearPoseTargets();
    arm_group.clearPathConstraints();
    arm_group.setPoseReferenceFrame(kTaskFrame);
    arm_group.setEndEffectorLink(kAttachLink);
    arm_group.setStartStateToCurrentState();
    diagnose_ik(arm_group, pose, stage_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    if (node_->get_parameter("prefer_position_only_targets").as_bool()) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Planning %s as position-only target at %.3f %.3f %.3f in %s",
        stage_name.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        kTaskFrame);
      arm_group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z, kAttachLink);
      planned = succeeded(arm_group.plan(plan));
      if (!planned && node_->get_parameter("allow_sampled_position_fallback").as_bool()) {
        planned = plan_sampled_position_target(
          arm_group, pose, stage_name, sampled_position_tolerance(stage_name), plan);
      }
    } else {
      arm_group.setPoseTarget(pose);
      planned = succeeded(arm_group.plan(plan));
    }
    if (!planned && !node_->get_parameter("prefer_position_only_targets").as_bool() &&
        node_->get_parameter("allow_position_only_fallback").as_bool()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Pose target failed for %s; retrying position-only target at %.3f %.3f %.3f in %s",
        stage_name.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        kTaskFrame);
      arm_group.clearPoseTargets();
      arm_group.clearPathConstraints();
      arm_group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z, kAttachLink);
      planned = succeeded(arm_group.plan(plan));
    }
    result.planned.push_back(planned);
    result.executed.push_back(false);
    if (!planned) {
      result.failed_stage = stage_name;
      result.error_message = "MoveIt failed to plan stage " + stage_name;
      return false;
    }

    if (execute_motion) {
      publish_feedback(goal_handle, stage_name, "executing");
      const bool executed = succeeded(arm_group.execute(plan));
      result.executed.back() = executed;
      if (!executed) {
        result.failed_stage = stage_name;
        result.error_message = "MoveIt failed to execute stage " + stage_name;
        return false;
      }
    }
    arm_group.clearPoseTargets();
    return true;
  }

  double sampled_position_tolerance(const std::string& stage_name) const
  {
    if (stage_name == "grasp") {
      return node_->get_parameter("sampled_grasp_tolerance_m").as_double();
    }
    if (stage_name == "place") {
      return node_->get_parameter("sampled_place_tolerance_m").as_double();
    }
    return node_->get_parameter("sampled_position_tolerance_m").as_double();
  }

  bool plan_sampled_position_target(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& stage_name,
    double tolerance,
    moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    auto state = arm_group.getCurrentState(1.0);
    if (!state) {
      RCLCPP_WARN(node_->get_logger(), "Sampled position fallback %s: no current robot state", stage_name.c_str());
      return false;
    }

    const auto* joint_model_group = state->getJointModelGroup(kArmGroup);
    if (joint_model_group == nullptr) {
      RCLCPP_WARN(node_->get_logger(), "Sampled position fallback %s: no joint model group", stage_name.c_str());
      return false;
    }

    const Eigen::Vector3d target(pose.position.x, pose.position.y, pose.position.z);
    const int attempts = std::max(1, static_cast<int>(node_->get_parameter("sampled_position_attempts").as_int()));
    double best_distance = std::numeric_limits<double>::infinity();

    RCLCPP_WARN(
      node_->get_logger(),
      "Position target plan failed for %s; trying sampled joint goal fallback at %.3f %.3f %.3f with tolerance %.3f",
      stage_name.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      tolerance);

    for (int attempt = 0; attempt < attempts; ++attempt) {
      moveit::core::RobotState sampled_state(*state);
      if (attempt > 0) {
        sampled_state.setToRandomPositions(joint_model_group);
      }
      sampled_state.update();

      const Eigen::Vector3d tip = sampled_state.getGlobalLinkTransform(kAttachLink).translation();
      const double distance = (tip - target).norm();
      best_distance = std::min(best_distance, distance);
      if (distance > tolerance) {
        continue;
      }

      std::vector<double> joint_values;
      sampled_state.copyJointGroupPositions(joint_model_group, joint_values);
      arm_group.clearPoseTargets();
      arm_group.clearPathConstraints();
      arm_group.setStartStateToCurrentState();
      arm_group.setJointValueTarget(joint_values);
      if (succeeded(arm_group.plan(plan))) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Sampled position fallback succeeded for %s after %d/%d samples: tip=(%.3f, %.3f, %.3f), distance=%.3f",
          stage_name.c_str(),
          attempt + 1,
          attempts,
          tip.x(),
          tip.y(),
          tip.z(),
          distance);
        return true;
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Sampled position fallback failed for %s after %d samples; best distance=%.3f target=(%.3f, %.3f, %.3f)",
      stage_name.c_str(),
      attempts,
      best_distance,
      pose.position.x,
      pose.position.y,
      pose.position.z);
    return false;
  }

  void diagnose_ik(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& stage_name) const
  {
    auto state = arm_group.getCurrentState(1.0);
    if (!state) {
      RCLCPP_WARN(node_->get_logger(), "IK diagnostic %s: no current robot state", stage_name.c_str());
      return;
    }

    const auto* joint_model_group = state->getJointModelGroup(kArmGroup);
    if (joint_model_group == nullptr) {
      RCLCPP_WARN(node_->get_logger(), "IK diagnostic %s: no joint model group %s", stage_name.c_str(), kArmGroup);
      return;
    }

    const Eigen::Isometry3d& current_tf = state->getGlobalLinkTransform(kAttachLink);
    RCLCPP_INFO(
      node_->get_logger(),
      "IK diagnostic %s: current %s=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f), q=(%.3f, %.3f, %.3f, %.3f) frame=%s",
      stage_name.c_str(),
      kAttachLink,
      current_tf.translation().x(),
      current_tf.translation().y(),
      current_tf.translation().z(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
      kTaskFrame);

    moveit::core::RobotState ik_state(*state);
    const bool fixed_orientation_ik = ik_state.setFromIK(joint_model_group, pose, kAttachLink, 0.25);
    RCLCPP_INFO(
      node_->get_logger(),
      "IK diagnostic %s: fixed-orientation IK %s",
      stage_name.c_str(),
      fixed_orientation_ik ? "succeeded" : "failed");
    if (fixed_orientation_ik) {
      std::vector<double> positions;
      ik_state.copyJointGroupPositions(joint_model_group, positions);
      std::ostringstream values;
      for (std::size_t i = 0; i < positions.size(); ++i) {
        if (i > 0) {
          values << ", ";
        }
        values << positions[i];
      }
      RCLCPP_INFO(node_->get_logger(), "IK diagnostic %s: solution=[%s]", stage_name.c_str(), values.str().c_str());
    }
  }

  bool plan_named_target(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const std::string& target_name,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    bool execute_motion)
  {
    publish_feedback(goal_handle, "named_" + target_name, "planning");
    arm_group.clearPoseTargets();
    arm_group.clearPathConstraints();
    arm_group.setNamedTarget(target_name);
    log_named_target_values(arm_group, target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = succeeded(arm_group.plan(plan));
    if (!planned) {
      RCLCPP_WARN(node_->get_logger(), "Could not plan named arm target %s", target_name.c_str());
      return false;
    }
    if (!execute_motion) {
      return true;
    }
    publish_feedback(goal_handle, "named_" + target_name, "executing");
    return succeeded(arm_group.execute(plan));
  }

  bool plan_joint_target(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const std::vector<double>& positions,
    const std::string& stage_name,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    bool execute_motion,
    PickPlaceManipulation::Result& result)
  {
    publish_feedback(goal_handle, stage_name, "planning");
    arm_group.clearPoseTargets();
    arm_group.clearPathConstraints();
    arm_group.setJointValueTarget(positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = succeeded(arm_group.plan(plan));
    result.planned.push_back(planned);
    result.executed.push_back(false);
    if (!planned) {
      result.failed_stage = stage_name;
      result.error_message = "MoveIt failed to plan joint fallback stage " + stage_name;
      return false;
    }
    if (execute_motion) {
      publish_feedback(goal_handle, stage_name, "executing");
      const bool executed = succeeded(arm_group.execute(plan));
      result.executed.back() = executed;
      if (!executed) {
        result.failed_stage = stage_name;
        result.error_message = "MoveIt failed to execute joint fallback stage " + stage_name;
        return false;
      }
    }
    return true;
  }

  void log_named_target_values(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const std::string& target_name) const
  {
    const auto named_targets = arm_group.getNamedTargets();
    if (std::find(named_targets.begin(), named_targets.end(), target_name) == named_targets.end()) {
      RCLCPP_WARN(node_->get_logger(), "Named target %s is not in MoveIt targets", target_name.c_str());
      return;
    }

    auto state = arm_group.getCurrentState(1.0);
    if (!state) {
      return;
    }
    const auto* joint_model_group = state->getJointModelGroup(kArmGroup);
    if (joint_model_group == nullptr) {
      return;
    }

    moveit::core::RobotState named_state(*state);
    if (!named_state.setToDefaultValues(joint_model_group, target_name)) {
      RCLCPP_WARN(node_->get_logger(), "Could not resolve named target %s", target_name.c_str());
      return;
    }

    std::vector<double> values;
    named_state.copyJointGroupPositions(joint_model_group, values);
    std::ostringstream joined;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        joined << ", ";
      }
      joined << values[index];
    }
    RCLCPP_INFO(node_->get_logger(), "Named target %s joint values=[%s]", target_name.c_str(), joined.str().c_str());
  }

  bool move_gripper(
    moveit::planning_interface::MoveGroupInterface& grip_group,
    const std::string& target_name,
    bool execute_motion,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle)
  {
    publish_feedback(goal_handle, "gripper_" + target_name, execute_motion ? "executing" : "planning");
    grip_group.setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = succeeded(grip_group.plan(plan));
    if (!planned || !execute_motion) {
      return planned;
    }
    return succeeded(grip_group.execute(plan));
  }

  bool run_joint_tabletop_fallback(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    moveit::planning_interface::MoveGroupInterface& grip_group,
    moveit::planning_interface::PlanningSceneInterface& scene,
    const std::string& frame_id,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    const PickPlaceManipulation::Goal& goal,
    PickPlaceManipulation::Result& result)
  {
    RCLCPP_WARN(node_->get_logger(), "Trying v1 joint-space tabletop fallback");
    result.stage_names = {"joint_pre_grasp", "joint_grasp", "joint_lift", "joint_pre_place", "joint_place"};
    result.planned.clear();
    result.executed.clear();

    const std::vector<double> pre_grasp{0.42, 0.42, -0.70, 0.52, 0.0};
    const std::vector<double> grasp{0.42, 0.28, -0.82, 0.34, 0.0};
    const std::vector<double> lift{0.42, 0.50, -0.62, 0.58, 0.0};
    const std::vector<double> pre_place{-0.25, 0.48, -0.66, 0.56, 0.0};
    const std::vector<double> place{-0.25, 0.32, -0.82, 0.38, 0.0};

    bool ok = move_gripper(grip_group, "open", goal.execute, goal_handle);
    ok = ok && plan_joint_target(arm_group, pre_grasp, "joint_pre_grasp", goal_handle, goal.execute, result);
    ok = ok && plan_joint_target(arm_group, grasp, "joint_grasp", goal_handle, goal.execute, result);
    if (ok) {
      scene.removeCollisionObjects({kObjectId});
      ok = move_gripper(grip_group, "close", goal.execute, goal_handle);
      scene.applyCollisionObject(
        make_collision_object(kObjectId, goal.target_label, goal.target_center, goal.target_dimensions, frame_id));
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      arm_group.attachObject(kObjectId, kAttachLink);
    }
    ok = ok && plan_joint_target(arm_group, lift, "joint_lift", goal_handle, goal.execute, result);
    ok = ok && plan_joint_target(arm_group, pre_place, "joint_pre_place", goal_handle, goal.execute, result);
    ok = ok && plan_joint_target(arm_group, place, "joint_place", goal_handle, goal.execute, result);
    if (ok) {
      ok = move_gripper(grip_group, "open", goal.execute, goal_handle);
      arm_group.detachObject(kObjectId);
    }
    return ok;
  }

  std::string debug_pose_log_path() const
  {
    const std::string log_dir = node_->get_parameter("debug_pose_log_dir").as_string();
    const std::time_t now = std::time(nullptr);
    std::ostringstream path;
    path << log_dir << "/pick_place_poses_" << now << ".json";
    return path.str();
  }

  void write_pose_json(
    std::ostream& out,
    const geometry_msgs::msg::Pose& pose,
    const std::string& indent) const
  {
    out << indent << "{\n";
    out << indent << "  \"position\": {\"x\": " << pose.position.x
        << ", \"y\": " << pose.position.y
        << ", \"z\": " << pose.position.z << "},\n";
    out << indent << "  \"orientation\": {\"x\": " << pose.orientation.x
        << ", \"y\": " << pose.orientation.y
        << ", \"z\": " << pose.orientation.z
        << ", \"w\": " << pose.orientation.w << "}\n";
    out << indent << "}";
  }

  void write_debug_pose_log(
    const PickPlaceManipulation::Goal& goal,
    const std::vector<std::string>& stage_names,
    const std::vector<std::pair<std::string, std::vector<geometry_msgs::msg::Pose>>>& pose_candidates,
    const std::string& path) const
  {
    std::ofstream out(path);
    if (!out) {
      RCLCPP_WARN(node_->get_logger(), "Could not write debug pose log to %s", path.c_str());
      return;
    }

    out << "{\n";
    out << "  \"target\": {\n";
    out << "    \"label\": \"" << goal.target_label << "\",\n";
    out << "    \"center\": {\"x\": " << goal.target_center.x
        << ", \"y\": " << goal.target_center.y
        << ", \"z\": " << goal.target_center.z << "},\n";
    out << "    \"dimensions\": {\"x\": " << goal.target_dimensions.x
        << ", \"y\": " << goal.target_dimensions.y
        << ", \"z\": " << goal.target_dimensions.z << "}\n";
    out << "  },\n";
    out << "  \"destination\": {\n";
    out << "    \"label\": \"" << goal.destination_label << "\",\n";
    out << "    \"center\": {\"x\": " << goal.destination_center.x
        << ", \"y\": " << goal.destination_center.y
        << ", \"z\": " << goal.destination_center.z << "},\n";
    out << "    \"dimensions\": {\"x\": " << goal.destination_dimensions.x
        << ", \"y\": " << goal.destination_dimensions.y
        << ", \"z\": " << goal.destination_dimensions.z << "}\n";
    out << "  },\n";
    out << "  \"cartesian_candidates\": [\n";
    for (std::size_t candidate_index = 0; candidate_index < pose_candidates.size(); ++candidate_index) {
      out << "    {\n";
      out << "      \"candidate\": " << candidate_index + 1 << ",\n";
      out << "      \"name\": \"" << pose_candidates[candidate_index].first << "\",\n";
      out << "      \"stages\": [\n";
      for (std::size_t stage_index = 0; stage_index < pose_candidates[candidate_index].second.size(); ++stage_index) {
        out << "        {\n";
        out << "          \"name\": \"" << stage_names[stage_index] << "\",\n";
        out << "          \"pose\": ";
        write_pose_json(out, pose_candidates[candidate_index].second[stage_index], "          ");
        out << "\n        }";
        if (stage_index + 1 < pose_candidates[candidate_index].second.size()) {
          out << ",";
        }
        out << "\n";
      }
      out << "      ]\n";
      out << "    }";
      if (candidate_index + 1 < pose_candidates.size()) {
        out << ",";
      }
      out << "\n";
    }
    out << "  ],\n";
    out << "  \"joint_tabletop_fallback\": {\n";
    out << "    \"stage_names\": [\"joint_pre_grasp\", \"joint_grasp\", \"joint_lift\", \"joint_pre_place\", \"joint_place\"],\n";
    out << "    \"joint_names\": [\"arm_joint1\", \"arm_joint2\", \"arm_joint3\", \"arm_joint4\", \"arm_joint5\"],\n";
    out << "    \"positions\": [[0.42, 0.42, -0.70, 0.52, 0.0], [0.42, 0.28, -0.82, 0.34, 0.0], [0.42, 0.50, -0.62, 0.58, 0.0], [-0.25, 0.48, -0.66, 0.56, 0.0], [-0.25, 0.32, -0.82, 0.38, 0.0]]\n";
    out << "  }\n";
    out << "}\n";
    RCLCPP_INFO(node_->get_logger(), "Wrote pick/place debug poses to %s", path.c_str());
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    const std::string& stage_name,
    const std::string& state)
  {
    auto feedback = std::make_shared<PickPlaceManipulation::Feedback>();
    feedback->stage_name = stage_name;
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  void publish_base_velocity(double vx, double vy)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node_->now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = vx;
    command.twist.linear.y = vy;
    base_cmd_pub_->publish(command);
  }

  void stop_base()
  {
    for (int i = 0; i < 5; ++i) {
      publish_base_velocity(0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  bool point_in_base_reach_window(const geometry_msgs::msg::Point& point) const
  {
    const double min_x = node_->get_parameter("base_reach_min_x_m").as_double();
    const double max_x = node_->get_parameter("base_reach_max_x_m").as_double();
    const double max_abs_y = node_->get_parameter("base_reach_max_abs_y_m").as_double();
    return point.x >= min_x && point.x <= max_x && std::abs(point.y) <= max_abs_y;
  }

  bool task_in_base_reach_window(const PickPlaceManipulation::Goal& goal) const
  {
    const auto candidates = generate_pose_candidates(goal);
    if (candidates.empty()) {
      return false;
    }
    for (const auto& pose : candidates.front().second) {
      if (!point_in_base_reach_window(pose.position)) {
        return false;
      }
    }
    return true;
  }

  bool common_base_move_interval(
    const std::vector<geometry_msgs::msg::Point>& points,
    bool x_axis,
    double min_after_move,
    double max_after_move,
    double& lower,
    double& upper) const
  {
    if (points.empty()) {
      lower = 0.0;
      upper = 0.0;
      return false;
    }

    lower = -std::numeric_limits<double>::infinity();
    upper = std::numeric_limits<double>::infinity();
    for (const auto& point : points) {
      const double value = x_axis ? point.x : point.y;
      lower = std::max(lower, value - max_after_move);
      upper = std::min(upper, value - min_after_move);
    }
    return lower <= upper;
  }

  std::vector<geometry_msgs::msg::Point> base_reach_stage_points(
    const PickPlaceManipulation::Goal& goal) const
  {
    std::vector<geometry_msgs::msg::Point> points;
    const auto candidates = generate_pose_candidates(goal);
    if (candidates.empty()) {
      return points;
    }
    for (const auto& pose : candidates.front().second) {
      points.push_back(pose.position);
    }
    return points;
  }

  bool compute_base_reach_move(
    const PickPlaceManipulation::Goal& goal,
    double& move_x,
    double& move_y) const
  {
    const double desired_x = node_->get_parameter("base_target_x_m").as_double();
    const double desired_y = node_->get_parameter("base_target_y_m").as_double();
    const double x_tolerance = node_->get_parameter("base_target_x_tolerance_m").as_double();
    const double y_tolerance = node_->get_parameter("base_target_y_tolerance_m").as_double();
    const double max_step_x = node_->get_parameter("base_max_adjust_x_m").as_double();
    const double max_step_y = node_->get_parameter("base_max_adjust_y_m").as_double();
    const double min_x = node_->get_parameter("base_reach_min_x_m").as_double();
    const double max_x = node_->get_parameter("base_reach_max_x_m").as_double();
    const double max_abs_y = node_->get_parameter("base_reach_max_abs_y_m").as_double();

    double lower_x = 0.0;
    double upper_x = 0.0;
    double lower_y = 0.0;
    double upper_y = 0.0;
    const auto stage_points = base_reach_stage_points(goal);
    const bool x_possible = common_base_move_interval(stage_points, true, min_x, max_x, lower_x, upper_x);
    const bool y_possible = common_base_move_interval(
      stage_points, false, -max_abs_y, max_abs_y, lower_y, upper_y);
    if (!x_possible || !y_possible) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No single base pose can put generated arm stages in reach window: "
        "target=(%.3f, %.3f), destination=(%.3f, %.3f), x_window=[%.3f, %.3f], y_window=[%.3f, %.3f]",
        goal.target_center.x,
        goal.target_center.y,
        goal.destination_center.x,
        goal.destination_center.y,
        min_x,
        max_x,
        -max_abs_y,
        max_abs_y);
      return false;
    }

    double desired_move_x = goal.target_center.x - desired_x;
    double desired_move_y = goal.target_center.y - desired_y;
    if (std::abs(desired_move_x) <= x_tolerance && task_in_base_reach_window(goal)) {
      desired_move_x = 0.0;
    }
    if (std::abs(desired_move_y) <= y_tolerance && task_in_base_reach_window(goal)) {
      desired_move_y = 0.0;
    }

    const double full_move_x = std::clamp(desired_move_x, lower_x, upper_x);
    const double full_move_y = std::clamp(desired_move_y, lower_y, upper_y);
    move_x = std::clamp(full_move_x, -max_step_x, max_step_x);
    move_y = std::clamp(full_move_y, -max_step_y, max_step_y);
    return true;
  }

  void apply_base_frame_shift(PickPlaceManipulation::Goal& goal, double move_x, double move_y) const
  {
    goal.target_center.x -= move_x;
    goal.target_center.y -= move_y;
    goal.destination_center.x -= move_x;
    goal.destination_center.y -= move_y;
  }

  void execute_base_nudge(double move_x, double move_y)
  {
    const double max_vx = std::max(0.001, node_->get_parameter("base_max_linear_x_mps").as_double());
    const double max_vy = std::max(0.001, node_->get_parameter("base_max_linear_y_mps").as_double());
    const double rate_hz = std::max(1.0, node_->get_parameter("base_command_rate_hz").as_double());
    const double duration = std::max(std::abs(move_x) / max_vx, std::abs(move_y) / max_vy);
    const double vx = duration > 0.0 ? move_x / duration : 0.0;
    const double vy = duration > 0.0 ? move_y / duration : 0.0;
    const int iterations = std::max(1, static_cast<int>(std::ceil(duration * rate_hz)));
    const auto sleep_duration = std::chrono::duration<double>(1.0 / rate_hz);

    for (int i = 0; rclcpp::ok() && i < iterations; ++i) {
      publish_base_velocity(vx, vy);
      std::this_thread::sleep_for(sleep_duration);
    }
    stop_base();
  }

  bool reposition_base_for_reach(
    PickPlaceManipulation::Goal& goal,
    bool execute_motion,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle)
  {
    if (!node_->get_parameter("enable_base_reposition").as_bool()) {
      return true;
    }

    const int max_nudges = std::max(
      1, static_cast<int>(node_->get_parameter("base_max_reposition_nudges").as_int()));
    const double desired_x = node_->get_parameter("base_target_x_m").as_double();
    const double desired_y = node_->get_parameter("base_target_y_m").as_double();
    const double x_tolerance = node_->get_parameter("base_target_x_tolerance_m").as_double();
    const double y_tolerance = node_->get_parameter("base_target_y_tolerance_m").as_double();
    const bool target_centered =
      std::abs(goal.target_center.x - desired_x) <= x_tolerance &&
      std::abs(goal.target_center.y - desired_y) <= y_tolerance;
    if (target_centered && task_in_base_reach_window(goal)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Base reposition skipped: target is centered and generated stages are in reach, target=(%.3f, %.3f), destination=(%.3f, %.3f)",
        goal.target_center.x,
        goal.target_center.y,
        goal.destination_center.x,
        goal.destination_center.y);
      return true;
    }

    publish_feedback(goal_handle, "base_reposition", execute_motion ? "executing" : "planning");
    for (int nudge = 1; nudge <= max_nudges; ++nudge) {
      double move_x = 0.0;
      double move_y = 0.0;
      if (!compute_base_reach_move(goal, move_x, move_y)) {
        return false;
      }
      if (std::abs(move_x) <= 1.0e-6 && std::abs(move_y) <= 1.0e-6) {
        break;
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "Base reposition nudge %d/%d: dx=%.3f dy=%.3f, before target=(%.3f, %.3f), destination=(%.3f, %.3f)",
        nudge,
        max_nudges,
        move_x,
        move_y,
        goal.target_center.x,
        goal.target_center.y,
        goal.destination_center.x,
        goal.destination_center.y);

      if (execute_motion) {
        execute_base_nudge(move_x, move_y);
      }
      apply_base_frame_shift(goal, move_x, move_y);

      RCLCPP_INFO(
        node_->get_logger(),
        "After base nudge %d/%d: target=(%.3f, %.3f, %.3f), destination=(%.3f, %.3f, %.3f)",
        nudge,
        max_nudges,
        goal.target_center.x,
        goal.target_center.y,
        goal.target_center.z,
        goal.destination_center.x,
        goal.destination_center.y,
        goal.destination_center.z);

      if (task_in_base_reach_window(goal)) {
        return true;
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Base reposition stopped before task was fully in reach window: target=(%.3f, %.3f), destination=(%.3f, %.3f)",
      goal.target_center.x,
      goal.target_center.y,
      goal.destination_center.x,
      goal.destination_center.y);
    return task_in_base_reach_window(goal);
  }

  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto adjusted_goal = *goal;
    auto result = std::make_shared<PickPlaceManipulation::Result>();
    const std::vector<std::string> stage_names = {"pre_grasp", "grasp", "lift", "pre_place", "place"};
    result->stage_names = stage_names;
    if (!reposition_base_for_reach(adjusted_goal, adjusted_goal.execute, goal_handle)) {
      result->success = false;
      result->failed_stage = "base_reposition";
      result->error_message = "Could not reposition mobile base";
      goal_handle->abort(result);
      return;
    }

    const auto pose_candidates = generate_pose_candidates(adjusted_goal);
    result->poses = pose_candidates.front().second;
    write_debug_pose_log(adjusted_goal, stage_names, pose_candidates, debug_pose_log_path());

    const double max_width = node_->get_parameter("max_grasp_width_m").as_double();
    const double width = grasp_width(adjusted_goal);
    if (width > max_width) {
      result->success = false;
      result->failed_stage = "geometry_check";
      result->error_message = "Target is too wide for v1 front-back grasp: width=" +
                              std::to_string(width) + " max=" + std::to_string(max_width);
      goal_handle->abort(result);
      return;
    }

    moveit::planning_interface::MoveGroupInterface arm_group(node_, kArmGroup);
    moveit::planning_interface::MoveGroupInterface grip_group(node_, kGripGroup);
    configure_group(arm_group);
    configure_group(grip_group);
    arm_group.setPoseReferenceFrame(kTaskFrame);
    arm_group.setEndEffectorLink(kAttachLink);
    moveit::planning_interface::PlanningSceneInterface scene;
    const std::string frame_id = kTaskFrame;

    std::vector<std::string> cleanup_ids{kObjectId, kDestinationId};
    const std::string existing_target_object_id = node_->get_parameter("existing_target_object_id").as_string();
    if (!existing_target_object_id.empty() && existing_target_object_id != kObjectId) {
      cleanup_ids.push_back(existing_target_object_id);
    }
    scene.removeCollisionObjects(cleanup_ids);
    scene.applyCollisionObject(
      make_collision_object(
        kObjectId, adjusted_goal.target_label, adjusted_goal.target_center, adjusted_goal.target_dimensions, frame_id));
    if (node_->get_parameter("add_destination_collision_object").as_bool()) {
      scene.applyCollisionObject(make_collision_object(
        kDestinationId,
        adjusted_goal.destination_label,
        adjusted_goal.destination_center,
        adjusted_goal.destination_dimensions,
        frame_id));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    bool ok = move_gripper(grip_group, "open", adjusted_goal.execute, goal_handle);
    if (!ok) {
      result->failed_stage = "gripper_open";
      result->error_message = "Could not plan or execute gripper open";
    }

    if (ok) {
      ok = false;
      if (node_->get_parameter("use_named_prepose_fallback").as_bool()) {
        const std::string named_prepose = node_->get_parameter("named_prepose").as_string();
        plan_named_target(arm_group, named_prepose, goal_handle, adjusted_goal.execute);
      }
      for (std::size_t candidate_index = 0; candidate_index < pose_candidates.size(); ++candidate_index) {
        result->poses = pose_candidates[candidate_index].second;
        result->planned.clear();
        result->executed.clear();
        result->failed_stage = "";
        result->error_message = "";
        RCLCPP_INFO(
          node_->get_logger(),
          "Trying pick/place pose candidate %zu (%s): pre_grasp=(%.3f, %.3f, %.3f)",
          candidate_index + 1,
          pose_candidates[candidate_index].first.c_str(),
          result->poses.front().position.x,
          result->poses.front().position.y,
          result->poses.front().position.z);
        ok = run_pose_sequence(
          arm_group,
          grip_group,
          scene,
          frame_id,
          goal_handle,
          adjusted_goal,
          pose_candidates[candidate_index].first,
          stage_names,
          *result);
        if (ok) {
          break;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "Pick/place pose candidate %zu (%s) failed at %s: %s",
          candidate_index + 1,
          pose_candidates[candidate_index].first.c_str(),
          result->failed_stage.c_str(),
          result->error_message.c_str());
        arm_group.detachObject(kObjectId);
        scene.removeCollisionObjects(cleanup_ids);
        scene.applyCollisionObject(
          make_collision_object(
            kObjectId, adjusted_goal.target_label, adjusted_goal.target_center, adjusted_goal.target_dimensions, frame_id));
        if (node_->get_parameter("add_destination_collision_object").as_bool()) {
          scene.applyCollisionObject(make_collision_object(
            kDestinationId,
            adjusted_goal.destination_label,
            adjusted_goal.destination_center,
            adjusted_goal.destination_dimensions,
            frame_id));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
    }

    if (!ok && node_->get_parameter("allow_joint_tabletop_fallback").as_bool()) {
      ok = run_joint_tabletop_fallback(arm_group, grip_group, scene, frame_id, goal_handle, adjusted_goal, *result);
    }

    arm_group.detachObject(kObjectId);
    scene.removeCollisionObjects(cleanup_ids);
    arm_group.clearPoseTargets();
    arm_group.clearPathConstraints();
    grip_group.clearPoseTargets();
    grip_group.clearPathConstraints();
    publish_feedback(goal_handle, ok ? "complete" : result->failed_stage, ok ? "succeeded" : "failed");

    result->success = ok;
    if (ok) {
      result->failed_stage = "";
      result->error_message = "";
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  bool run_pose_sequence(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    moveit::planning_interface::MoveGroupInterface& grip_group,
    moveit::planning_interface::PlanningSceneInterface& scene,
    const std::string& frame_id,
    const std::shared_ptr<GoalHandlePickPlace>& goal_handle,
    const PickPlaceManipulation::Goal& goal,
    const std::string& candidate_name,
    const std::vector<std::string>& stage_names,
    PickPlaceManipulation::Result& result)
  {
    bool ok = true;
    for (std::size_t index = 0; ok && index < result.poses.size(); ++index) {
      const std::string feedback_stage = candidate_name + "/" + stage_names[index];
      if (stage_names[index] == "grasp") {
        std::vector<std::string> grasp_clear_ids{kObjectId};
        const std::string existing_target_object_id = node_->get_parameter("existing_target_object_id").as_string();
        if (!existing_target_object_id.empty() && existing_target_object_id != kObjectId) {
          grasp_clear_ids.push_back(existing_target_object_id);
        }
        scene.removeCollisionObjects(grasp_clear_ids);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
      ok = plan_and_maybe_execute(arm_group, result.poses[index], feedback_stage, goal.execute, goal_handle, result);
      if (!ok) {
        result.failed_stage = stage_names[index];
        break;
      }

      if (stage_names[index] == "grasp") {
        ok = move_gripper(grip_group, "close", goal.execute, goal_handle);
        if (!ok) {
          result.failed_stage = "gripper_close";
          result.error_message = "Could not plan or execute gripper close";
          break;
        }
        scene.applyCollisionObject(
          make_collision_object(kObjectId, goal.target_label, goal.target_center, goal.target_dimensions, frame_id));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        arm_group.attachObject(kObjectId, kAttachLink);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      } else if (stage_names[index] == "place") {
        ok = move_gripper(grip_group, "open", goal.execute, goal_handle);
        if (!ok) {
          result.failed_stage = "gripper_release";
          result.error_message = "Could not plan or execute gripper release";
          break;
        }
        arm_group.detachObject(kObjectId);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
    }
    return ok;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<PickPlaceManipulation>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pick_place_manipulation_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<PickPlaceManipulationServer>(node);
  (void)server;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
