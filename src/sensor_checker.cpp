#include "sensor_checker.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <atomic>

namespace sensing_utils
{

// ======================
//  SensorCheckerBase
// ======================

SensorCheckerBase::SensorCheckerBase(
    const std::string & sensor_name,
    const std::string & topic_name,
    double expected_hz)
: sensor_name_(sensor_name)
, topic_name_(topic_name)
, expected_hz_(expected_hz)
{
}

bool SensorCheckerBase::run_check(
    const rclcpp::Logger & logger,
    std::chrono::seconds max_duration,
    std::size_t min_messages)
{
  last_result_ = perform_check(max_duration, min_messages);

  if (!last_result_.ok) {
    RCLCPP_ERROR(
        logger,
        ANSI_COLOR_RED
        "[SensorCheck] Sensor '%s' (topic '%s') FAILED: %s (received=%zu, measured=%.2f Hz)"
        ANSI_COLOR_RESET,
        sensor_name_.c_str(),
        topic_name_.c_str(),
        last_result_.reason.c_str(),
        last_result_.received_messages,
        last_result_.measured_hz);
    return false;
  }

  RCLCPP_INFO(
      logger,
      ANSI_COLOR_GREEN
      "[SensorCheck] Sensor '%s' (topic '%s') OK: %.2f Hz (%zu msgs)"
      ANSI_COLOR_RESET,
      sensor_name_.c_str(),
      topic_name_.c_str(),
      last_result_.measured_hz,
      last_result_.received_messages);

  return true;
}

// ======================
//  ImageSensorChecker
// ======================

namespace
{
  // Used to create unique node names and avoid rosout warnings
  std::atomic<int> g_checker_node_id{0};

  SensorCheckResult check_image_stream_internal(
      const std::string & topic_name,
      double expected_hz,
      std::chrono::seconds max_duration,
      std::size_t min_messages)
  {
    SensorCheckResult result;

    // Create a unique node name for this check
    const int id = g_checker_node_id.fetch_add(1);
    const std::string node_name = "sensor_check_node_" + std::to_string(id);
    auto node = std::make_shared<rclcpp::Node>(node_name);

    std::vector<rclcpp::Time> stamps;
    stamps.reserve(min_messages + 5);

    // Typical QoS for sensor data
    auto qos = rclcpp::SensorDataQoS();

    // Temporary subscription to the image topic
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        topic_name,
        qos,
        [&stamps](sensor_msgs::msg::Image::ConstSharedPtr msg)
        {
          stamps.push_back(msg->header.stamp);
        });

    // Local executor to spin the temporary node
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    const auto start_time = node->now();
    const auto max_dur = rclcpp::Duration::from_seconds(max_duration.count());
    rclcpp::Rate rate(200.0);  // internal loop frequency

    while (rclcpp::ok()) {
      exec.spin_some();
      auto now = node->now();

      if (stamps.size() >= min_messages) {
        break;
      }

      if ((now - start_time) > max_dur) {
        break;
      }

      rate.sleep();
    }

    result.received_messages = stamps.size();

    if (stamps.size() < 2) {
      result.reason = "Not enough Image messages received on topic '" + topic_name + "'";
      result.ok = false;
      return result;
    }

    const double t_first = stamps.front().seconds();
    const double t_last  = stamps.back().seconds();
    const double dt = t_last - t_first;

    if (dt <= 0.0) {
      result.reason = "Invalid or non-increasing timestamps on topic '" + topic_name + "'";
      result.ok = false;
      return result;
    }

    const double hz = static_cast<double>(stamps.size() - 1) / dt;
    result.measured_hz = hz;

    // Simple tolerance: 70% to 130% of expected frequency
    const double lower = expected_hz * 0.7;
    const double upper = expected_hz * 1.3;

    if (hz < lower || hz > upper) {
      result.reason = "Measured rate " + std::to_string(hz) +
                      " Hz out of range for topic '" + topic_name +
                      "' (expected ~" + std::to_string(expected_hz) + " Hz)";
      result.ok = false;
      return result;
    }

    result.ok = true;
    result.reason = "OK";
    return result;
  }

}  // anonymous namespace


ImageSensorChecker::ImageSensorChecker(
    const std::string & sensor_name,
    const std::string & topic_name,
    double expected_hz)
: SensorCheckerBase(sensor_name, topic_name, expected_hz)
{
}

SensorCheckResult ImageSensorChecker::perform_check(
    std::chrono::seconds max_duration,
    std::size_t min_messages)
{
  return check_image_stream_internal(
      topic_name_, expected_hz_, max_duration, min_messages);
}

// ======================
//  Legacy-style wrapper
// ======================

bool check_image_sensor(
    const std::string & sensor_name,
    const std::string & topic_name,
    double expected_hz,
    const rclcpp::Logger & logger,
    std::chrono::seconds max_duration,
    std::size_t min_messages)
{
  ImageSensorChecker checker(sensor_name, topic_name, expected_hz);
  return checker.run_check(logger, max_duration, min_messages);
}

}  // namespace sensing_utils
