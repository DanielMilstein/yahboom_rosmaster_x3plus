#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
struct Bounds
{
  double min;
  double max;
};

struct ReachabilityCloud
{
  std::string name;
  std::string topic;
  geometry_msgs::msg::Quaternion orientation;
  std::vector<Eigen::Vector3d> points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

geometry_msgs::msg::Quaternion normalized_quaternion(double x, double y, double z, double w)
{
  Eigen::Quaterniond q(w, x, y, z);
  q.normalize();

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}
}  // namespace

class ReachabilityCloudPublisher : public rclcpp::Node
{
public:
  ReachabilityCloudPublisher() : Node("reachability_cloud_publisher")
  {
    declare_parameter("group_name", "arm_group");
    declare_parameter("tip_link", "arm_link5");
    declare_parameter("frame_id", "base_footprint");
    declare_parameter("seed_named_state", "init");
    declare_parameter("ik_timeout_sec", 0.008);
    declare_parameter("ik_attempts", 3);
    declare_parameter("resolution_m", 0.03);
    declare_parameter("x_min", 0.04);
    declare_parameter("x_max", 0.42);
    declare_parameter("y_min", -0.22);
    declare_parameter("y_max", 0.22);
    declare_parameter("z_min", 0.12);
    declare_parameter("z_max", 0.52);

    declare_parameter("front_back_qx", 0.08637313729188083);
    declare_parameter("front_back_qy", 0.40819551903689105);
    declare_parameter("front_back_qz", 0.10050372779920543);
    declare_parameter("front_back_qw", 0.9032248336328139);
    declare_parameter("front_back_roll", 0.0);
    declare_parameter("front_back_pitch", 0.85);
    declare_parameter("front_back_yaw", 0.0);
    declare_parameter("top_down_qx", 0.0);
    declare_parameter("top_down_qy", 0.70710678);
    declare_parameter("top_down_qz", 0.0);
    declare_parameter("top_down_qw", 0.70710678);
    declare_parameter("top_down_roll", 0.0);
    declare_parameter("top_down_pitch", 1.5708);
    declare_parameter("top_down_yaw", 0.0);
    declare_parameter("degree_45_qx", 0.0);
    declare_parameter("degree_45_qy", 0.38268343);
    declare_parameter("degree_45_qz", 0.0);
    declare_parameter("degree_45_qw", 0.92387953);
    declare_parameter("degree_45_roll", 0.0);
    declare_parameter("degree_45_pitch", 0.7854);
    declare_parameter("degree_45_yaw", 0.0);

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    status_publisher_ = create_publisher<std_msgs::msg::String>("reachability/status", qos);
    clouds_ = {
      make_cloud("front_back", "reachability/reachable_front_back", "front_back", qos),
      make_cloud("top_down", "reachability/reachable_top_down", "top_down", qos),
      make_cloud("45_degree", "reachability/reachable_45_degree", "degree_45", qos),
    };

    compute_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        compute_timer_->cancel();
        compute_clouds();
      });
    publish_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { publish_clouds(); });
  }

private:
  ReachabilityCloud make_cloud(
    const std::string& name,
    const std::string& topic,
    const std::string& parameter_prefix,
    const rclcpp::QoS& qos)
  {
    ReachabilityCloud cloud;
    cloud.name = name;
    cloud.topic = topic;
    cloud.orientation = normalized_quaternion(
      get_parameter(parameter_prefix + "_qx").as_double(),
      get_parameter(parameter_prefix + "_qy").as_double(),
      get_parameter(parameter_prefix + "_qz").as_double(),
      get_parameter(parameter_prefix + "_qw").as_double());
    cloud.publisher = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
    return cloud;
  }

  void compute_clouds()
  {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const moveit::core::RobotModelPtr model = loader.getModel();
    if (!model) {
      RCLCPP_ERROR(get_logger(), "Could not load MoveIt robot model from robot_description");
      return;
    }

    const std::string group_name = get_parameter("group_name").as_string();
    const std::string tip_link = get_parameter("tip_link").as_string();
    const auto* joint_model_group = model->getJointModelGroup(group_name);
    if (joint_model_group == nullptr) {
      RCLCPP_ERROR(get_logger(), "MoveIt group '%s' does not exist", group_name.c_str());
      return;
    }
    publish_status("computing");

    moveit::core::RobotState seed_state(model);
    seed_state.setToDefaultValues();
    const std::string seed_named_state = get_parameter("seed_named_state").as_string();
    if (!seed_named_state.empty() && !seed_state.setToDefaultValues(joint_model_group, seed_named_state)) {
      RCLCPP_WARN(get_logger(), "Could not apply seed named state '%s'; using default state", seed_named_state.c_str());
    }

    const Bounds x{get_parameter("x_min").as_double(), get_parameter("x_max").as_double()};
    const Bounds y{get_parameter("y_min").as_double(), get_parameter("y_max").as_double()};
    const Bounds z{get_parameter("z_min").as_double(), get_parameter("z_max").as_double()};
    const double resolution = get_parameter("resolution_m").as_double();
    const double ik_timeout = get_parameter("ik_timeout_sec").as_double();
    const int ik_attempts = std::max(1, static_cast<int>(get_parameter("ik_attempts").as_int()));

    if (resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "resolution_m must be positive");
      return;
    }

    const auto count_axis = [resolution](const Bounds& bounds) {
      return static_cast<std::size_t>(std::floor((bounds.max - bounds.min) / resolution)) + 1;
    };
    const std::size_t x_count = count_axis(x);
    const std::size_t total_samples = x_count * count_axis(y) * count_axis(z);
    std::size_t samples = 0;
    std::size_t x_index = 0;
    for (double px = x.min; px <= x.max + 1e-9; px += resolution) {
      for (double py = y.min; py <= y.max + 1e-9; py += resolution) {
        for (double pz = z.min; pz <= z.max + 1e-9; pz += resolution) {
          ++samples;
          for (auto& cloud : clouds_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = px;
            pose.position.y = py;
            pose.position.z = pz;
            pose.orientation = cloud.orientation;

            if (has_ik_solution(seed_state, joint_model_group, pose, tip_link, ik_timeout, ik_attempts)) {
              cloud.points.emplace_back(px, py, pz);
            }
          }
        }
      }
      ++x_index;
      RCLCPP_INFO(
        get_logger(),
        "Reachability progress: x slice %zu/%zu, samples=%zu/%zu, front_back=%zu top_down=%zu 45_degree=%zu",
        x_index,
        x_count,
        samples,
        total_samples,
        clouds_[0].points.size(),
        clouds_[1].points.size(),
        clouds_[2].points.size());
      publish_status("computing " + std::to_string(samples) + "/" + std::to_string(total_samples));
      publish_clouds();
    }

    RCLCPP_INFO(
      get_logger(),
      "Reachability grid complete in model frame '%s': %zu samples, front_back=%zu top_down=%zu 45_degree=%zu",
      model->getModelFrame().c_str(),
      samples,
      clouds_[0].points.size(),
      clouds_[1].points.size(),
      clouds_[2].points.size());
    publish_status("complete");
    publish_clouds();
  }

  void publish_status(const std::string& status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher_->publish(msg);
  }

  bool has_ik_solution(
    const moveit::core::RobotState& seed_state,
    const moveit::core::JointModelGroup* joint_model_group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& tip_link,
    double ik_timeout,
    int ik_attempts) const
  {
    for (int attempt = 0; attempt < ik_attempts; ++attempt) {
      moveit::core::RobotState ik_state(seed_state);
      if (attempt > 0) {
        ik_state.setToRandomPositions(joint_model_group);
      }
      if (ik_state.setFromIK(joint_model_group, pose, tip_link, ik_timeout)) {
        return true;
      }
    }
    return false;
  }

  void publish_clouds()
  {
    const std::string frame_id = get_parameter("frame_id").as_string();
    for (const auto& cloud_data : clouds_) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = frame_id;
      cloud.header.stamp = now();
      cloud.height = 1;

      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(cloud_data.points.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      for (const auto& point : cloud_data.points) {
        *iter_x = static_cast<float>(point.x());
        *iter_y = static_cast<float>(point.y());
        *iter_z = static_cast<float>(point.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }

      cloud_data.publisher->publish(cloud);
    }
  }

  std::vector<ReachabilityCloud> clouds_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr compute_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReachabilityCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
