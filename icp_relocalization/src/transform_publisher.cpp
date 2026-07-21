#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace
{
bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w);
}

bool normalized_transform_from_pose(
  const geometry_msgs::msg::Pose & pose, tf2::Transform & transform)
{
  if (!finite_pose(pose)) {
    return false;
  }

  tf2::Quaternion rotation;
  tf2::fromMsg(pose.orientation, rotation);
  const double length_squared = rotation.length2();
  if (!std::isfinite(length_squared) || length_squared < 1.0e-12) {
    return false;
  }

  rotation.normalize();
  transform.setOrigin(
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(rotation);
  return true;
}

std::array<double, 36> rotate_pose_covariance(
  const std::array<double, 36> & covariance,
  const tf2::Matrix3x3 & rotation)
{
  Eigen::Matrix<double, 6, 6> input;
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      input(row, column) = covariance[static_cast<std::size_t>(row * 6 + column)];
    }
  }

  Eigen::Matrix3d rotation_matrix;
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      rotation_matrix(row, column) =
        rotation[static_cast<int>(row)][static_cast<int>(column)];
    }
  }

  Eigen::Matrix<double, 6, 6> basis_change =
    Eigen::Matrix<double, 6, 6>::Zero();
  basis_change.block<3, 3>(0, 0) = rotation_matrix;
  basis_change.block<3, 3>(3, 3) = rotation_matrix;
  const Eigen::Matrix<double, 6, 6> output =
    basis_change * input * basis_change.transpose();

  std::array<double, 36> result{};
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      result[static_cast<std::size_t>(row * 6 + column)] = output(row, column);
    }
  }
  return result;
}
}  // namespace

class TransformPublisherNode : public rclcpp::Node
{
public:
  TransformPublisherNode()
  : Node("transform_publisher_node")
  {
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map_frame");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom_frame");
    sensor_frame_id_ =
      declare_parameter<std::string>("sensor_frame_id", "lidar_frame");
    icp_result_topic_ =
      declare_parameter<std::string>("icp_result_topic", "/icp_result");
    input_odometry_topic_ = declare_parameter<std::string>(
      "input_odometry_topic", "/relocalizing/odom_frame/odometry");
    output_odometry_topic_ = declare_parameter<std::string>(
      "output_odometry_topic", "/relocalizing/map_frame/odometry");

    if (map_frame_id_.empty() || odom_frame_id_.empty() ||
      sensor_frame_id_.empty())
    {
      throw std::invalid_argument("map, odom and sensor frame names must not be empty");
    }
    if (map_frame_id_ == odom_frame_id_ || map_frame_id_ == sensor_frame_id_ ||
      odom_frame_id_ == sensor_frame_id_)
    {
      throw std::invalid_argument("map, odom and sensor frame names must be unique");
    }
    if (icp_result_topic_.empty() || input_odometry_topic_.empty() ||
      output_odometry_topic_.empty())
    {
      throw std::invalid_argument("localization topic names must not be empty");
    }

    const auto icp_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    icp_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      icp_result_topic_, icp_qos,
      std::bind(
        &TransformPublisherNode::initial_pose_callback, this,
        std::placeholders::_1));

    const auto odometry_qos =
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odometry_topic_, odometry_qos,
      std::bind(
        &TransformPublisherNode::odometry_callback, this,
        std::placeholders::_1));
    map_odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odometry_topic_, odometry_qos);

    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(
      get_logger(),
      "Map-frame odometry: %s (%s -> %s) + %s (%s -> %s) => %s (%s -> %s)",
      icp_result_topic_.c_str(), map_frame_id_.c_str(), odom_frame_id_.c_str(),
      input_odometry_topic_.c_str(), odom_frame_id_.c_str(),
      sensor_frame_id_.c_str(), output_odometry_topic_.c_str(),
      map_frame_id_.c_str(), sensor_frame_id_.c_str());
  }

private:
  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_id_) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting %s: frame_id '%s' must be '%s'",
        icp_result_topic_.c_str(), message->header.frame_id.c_str(),
        map_frame_id_.c_str());
      return;
    }

    tf2::Transform candidate;
    if (!normalized_transform_from_pose(message->pose.pose, candidate)) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting %s: pose is not finite or quaternion is invalid",
        icp_result_topic_.c_str());
      return;
    }

    if (map_to_odom_ready_) {
      const tf2::Transform delta = map_to_odom_.inverseTimes(candidate);
      const double translation_delta = delta.getOrigin().length();
      const double rotation_delta = delta.getRotation().getAngleShortestPath();
      if (translation_delta > 1.0e-6 || rotation_delta > 1.0e-6) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring changed %s after initialization "
          "(translation delta %.6g m, rotation delta %.6g rad)",
          icp_result_topic_.c_str(), translation_delta, rotation_delta);
      }
      return;
    }

    map_to_odom_ = candidate;
    map_to_odom_ready_ = true;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    if (transform.header.stamp.sec == 0 &&
      transform.header.stamp.nanosec == 0)
    {
      transform.header.stamp = now();
    }
    transform.header.frame_id = map_frame_id_;
    transform.child_frame_id = odom_frame_id_;
    transform.transform = tf2::toMsg(map_to_odom_);
    static_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
      get_logger(), "icp定位成功：x=%.3f，y=%.3f，z=%.3f",
      map_to_odom_.getOrigin().x(), map_to_odom_.getOrigin().y(),
      map_to_odom_.getOrigin().z());
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!map_to_odom_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for %s before publishing %s", icp_result_topic_.c_str(),
        output_odometry_topic_.c_str());
      return;
    }

    if (message->header.frame_id != odom_frame_id_ ||
      message->child_frame_id != sensor_frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejecting %s with frames '%s' -> '%s'; expected '%s' -> '%s'",
        input_odometry_topic_.c_str(), message->header.frame_id.c_str(),
        message->child_frame_id.c_str(), odom_frame_id_.c_str(),
        sensor_frame_id_.c_str());
      return;
    }

    tf2::Transform odom_to_sensor;
    if (!normalized_transform_from_pose(message->pose.pose, odom_to_sensor)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejecting %s: pose is not finite or quaternion is invalid",
        input_odometry_topic_.c_str());
      return;
    }

    const tf2::Transform map_to_sensor = map_to_odom_ * odom_to_sensor;
    nav_msgs::msg::Odometry map_odometry = *message;
    map_odometry.header.frame_id = map_frame_id_;
    map_odometry.child_frame_id = sensor_frame_id_;
    map_odometry.pose.pose.position.x = map_to_sensor.getOrigin().x();
    map_odometry.pose.pose.position.y = map_to_sensor.getOrigin().y();
    map_odometry.pose.pose.position.z = map_to_sensor.getOrigin().z();
    map_odometry.pose.pose.orientation = tf2::toMsg(map_to_sensor.getRotation());
    map_odometry.pose.covariance = rotate_pose_covariance(
      message->pose.covariance, map_to_odom_.getBasis());
    // Odometry 的 twist 使用 child_frame_id 表达，所以这里保持原值。
    map_odometry_publisher_->publish(map_odometry);

    // FAST-LIO 的里程计频率较高。每次发布 map 坐标系里程计后，
    // 直接读取同一条消息的坐标，最多每 1000 毫秒打印一行。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "定位成功：x=%.3f，y=%.3f，z=%.3f",
      map_odometry.pose.pose.position.x, map_odometry.pose.pose.position.y,
      map_odometry.pose.pose.position.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    icp_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odometry_publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string sensor_frame_id_;
  std::string icp_result_topic_;
  std::string input_odometry_topic_;
  std::string output_odometry_topic_;
  tf2::Transform map_to_odom_;
  bool map_to_odom_ready_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
