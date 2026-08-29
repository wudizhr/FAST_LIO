#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class Px4ImuBridge : public rclcpp::Node
{
public:
  Px4ImuBridge() : Node("px4_imu_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/fmu/out/sensor_combined");
    timesync_topic_ = declare_parameter<std::string>("timesync_topic", "/fmu/out/timesync_status");
    output_topic_ = declare_parameter<std::string>("output_topic", "/px4/imu_fastlio");
    frame_id_ = declare_parameter<std::string>("frame_id", "px4_imu_flu");
    require_synced_time_ = declare_parameter<bool>("require_synced_time", true);
    drop_clipped_ = declare_parameter<bool>("drop_clipped", true);
    max_time_error_ms_ = declare_parameter<double>("max_time_error_ms", 50.0);
    max_timestamp_gap_ms_ = declare_parameter<double>("max_timestamp_gap_ms", 30.0);

    const auto rotation = declare_parameter<std::vector<double>>(
      "rotation_px4_to_imu",
      {1.0, 0.0, 0.0,
       0.0, -1.0, 0.0,
       0.0, 0.0, -1.0});
    if (rotation.size() != 9) {
      throw std::runtime_error("rotation_px4_to_imu must contain 9 row-major values");
    }
    rotation_ << rotation[0], rotation[1], rotation[2],
      rotation[3], rotation[4], rotation[5],
      rotation[6], rotation[7], rotation[8];
    const Eigen::Matrix3d orthogonality_error = rotation_ * rotation_.transpose() - Eigen::Matrix3d::Identity();
    if (orthogonality_error.norm() > 1e-6 || std::abs(rotation_.determinant() - 1.0) > 1e-6) {
      throw std::runtime_error("rotation_px4_to_imu must be a proper orthonormal rotation matrix");
    }

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    px4_subscription_ = create_subscription<px4_msgs::msg::SensorCombined>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Px4ImuBridge::sensorCallback, this, std::placeholders::_1));
    timesync_subscription_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      timesync_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Px4ImuBridge::timesyncCallback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&Px4ImuBridge::publishDiagnostics, this));

    RCLCPP_INFO(
      get_logger(), "PX4 IMU bridge: %s -> %s, frame=%s",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

private:
  static constexpr uint64_t kUnixEpochThresholdUs = 100000000000000ULL;

  void timesyncCallback(const px4_msgs::msg::TimesyncStatus::UniquePtr msg)
  {
    estimated_offset_us_ = msg->estimated_offset;
    have_timesync_ = true;
  }

  bool sampleTimeNs(const px4_msgs::msg::SensorCombined & msg, int64_t & sample_time_ns)
  {
    int64_t sample_us = static_cast<int64_t>(msg.timestamp);
    if (msg.accelerometer_timestamp_relative !=
      px4_msgs::msg::SensorCombined::RELATIVE_TIMESTAMP_INVALID)
    {
      sample_us += static_cast<int64_t>(msg.accelerometer_timestamp_relative);
    }

    int64_t absolute_time_ns = 0;
    if (msg.timestamp >= kUnixEpochThresholdUs) {
      absolute_time_ns = sample_us * 1000LL;
    } else if (have_timesync_) {
      // PX4 defines estimated_offset as PX4 time minus companion time.
      absolute_time_ns = (sample_us - estimated_offset_us_) * 1000LL;
    } else if (require_synced_time_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping PX4 IMU: timestamp is boot-relative and TimesyncStatus is unavailable");
      return false;
    } else {
      absolute_time_ns = now().nanoseconds();
    }

    if (last_sample_time_ns_ != 0 && absolute_time_ns <= last_sample_time_ns_) {
      ++timestamp_drop_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping non-monotonic PX4 timestamp: current=%ld previous=%ld ns",
        absolute_time_ns, last_sample_time_ns_);
      return false;
    }
    if (last_sample_time_ns_ != 0) {
      const double gap_ms = static_cast<double>(absolute_time_ns - last_sample_time_ns_) / 1e6;
      if (gap_ms > max_timestamp_gap_ms_) {
        ++timestamp_gap_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Large PX4 IMU timestamp gap: %.1f ms", gap_ms);
      }
    }
    sample_time_ns = absolute_time_ns;
    last_sample_time_ns_ = sample_time_ns;
    return true;
  }

  void sensorCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg)
  {
    ++received_count_;
    if ((msg->accelerometer_clipping != 0 || msg->gyro_clipping != 0)) {
      ++clipped_count_;
      if (drop_clipped_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Dropping clipped PX4 IMU sample: accel=0x%02x gyro=0x%02x",
          msg->accelerometer_clipping, msg->gyro_clipping);
        return;
      }
    }

    int64_t sample_time_ns = 0;
    if (!sampleTimeNs(*msg, sample_time_ns)) {
      return;
    }

    const Eigen::Vector3d accel_px4(
      msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
    const Eigen::Vector3d gyro_px4(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    if (!accel_px4.allFinite() || !gyro_px4.allFinite()) {
      ++invalid_count_;
      return;
    }
    const Eigen::Vector3d accel = rotation_ * accel_px4;
    const Eigen::Vector3d gyro = rotation_ * gyro_px4;

    sensor_msgs::msg::Imu output;
    output.header.stamp = rclcpp::Time(sample_time_ns, RCL_SYSTEM_TIME);
    output.header.frame_id = frame_id_;
    output.orientation_covariance[0] = -1.0;
    output.linear_acceleration.x = accel.x();
    output.linear_acceleration.y = accel.y();
    output.linear_acceleration.z = accel.z();
    output.angular_velocity.x = gyro.x();
    output.angular_velocity.y = gyro.y();
    output.angular_velocity.z = gyro.z();
    imu_publisher_->publish(output);
    ++published_count_;

    const double time_error_ms = static_cast<double>(now().nanoseconds() - sample_time_ns) / 1e6;
    max_abs_time_error_ms_ = std::max(max_abs_time_error_ms_, std::abs(time_error_ms));
    min_accel_norm_ = std::min(min_accel_norm_, accel.norm());
    max_accel_norm_ = std::max(max_accel_norm_, accel.norm());
    if (std::abs(time_error_ms) > max_time_error_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PX4 IMU sample time differs from ROS clock by %.1f ms", time_error_ms);
    }
  }

  void publishDiagnostics()
  {
    const size_t publisher_count = get_publishers_info_by_topic(output_topic_).size();
    if (publisher_count > 1) {
      RCLCPP_ERROR(
        get_logger(),
        "%s has %zu publishers; multiple IMU bridges will corrupt timestamp ordering",
        output_topic_.c_str(), publisher_count);
    }
    const uint64_t received_delta = received_count_ - last_received_count_;
    const double rate = static_cast<double>(received_delta) / 5.0;
    RCLCPP_INFO(
      get_logger(),
      "PX4 IMU: %.1f Hz, received=%lu published=%lu clipped=%lu invalid=%lu, "
      "timestamp_drops=%lu gaps=%lu, accel_norm=[%.3f, %.3f] m/s^2, max_time_error=%.1f ms",
      rate, received_count_, published_count_, clipped_count_, invalid_count_,
      timestamp_drop_count_, timestamp_gap_count_,
      std::isfinite(min_accel_norm_) ? min_accel_norm_ : 0.0,
      max_accel_norm_, max_abs_time_error_ms_);
    last_received_count_ = received_count_;
    min_accel_norm_ = std::numeric_limits<double>::infinity();
    max_accel_norm_ = 0.0;
    max_abs_time_error_ms_ = 0.0;
  }

  std::string input_topic_;
  std::string timesync_topic_;
  std::string output_topic_;
  std::string frame_id_;
  bool require_synced_time_{true};
  bool drop_clipped_{true};
  double max_time_error_ms_{50.0};
  double max_timestamp_gap_ms_{30.0};
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  bool have_timesync_{false};
  int64_t estimated_offset_us_{0};
  uint64_t received_count_{0};
  uint64_t published_count_{0};
  uint64_t clipped_count_{0};
  uint64_t invalid_count_{0};
  uint64_t last_received_count_{0};
  uint64_t timestamp_drop_count_{0};
  uint64_t timestamp_gap_count_{0};
  int64_t last_sample_time_ns_{0};
  double min_accel_norm_{std::numeric_limits<double>::infinity()};
  double max_accel_norm_{0.0};
  double max_abs_time_error_ms_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_subscription_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4ImuBridge>());
  rclcpp::shutdown();
  return 0;
}
