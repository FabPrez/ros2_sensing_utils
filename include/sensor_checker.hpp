#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <cstddef>

// ANSI color codes for colored logs
#define ANSI_COLOR_BLACK    "\033[1;30m"
#define ANSI_COLOR_RED      "\033[1;31m"
#define ANSI_COLOR_GREEN    "\033[1;32m"
#define ANSI_COLOR_YELLOW   "\033[1;33m"
#define ANSI_COLOR_BLUE     "\033[1;34m"
#define ANSI_COLOR_MAGENTA  "\033[1;35m"
#define ANSI_COLOR_CYAN     "\033[1;36m"
#define ANSI_COLOR_WHITE    "\033[1;37m"
#define ANSI_COLOR_RESET    "\033[0m"

namespace sensing_utils
{

/**
 * Result of a sensor check.
 */
struct SensorCheckResult
{
  bool ok{false};
  std::string reason;
  std::size_t received_messages{0};
  double measured_hz{0.0};
};

/**
 * Abstract base class for sensor checkers.
 *
 * Derived classes must implement perform_check(), which does the
 * actual ROS subscription & measurement, and returns a SensorCheckResult.
 *
 * This base class:
 *  - stores common parameters (name, topic, expected_hz)
 *  - handles logging with colors
 *  - stores the last result
 */
class SensorCheckerBase
{
public:
  SensorCheckerBase(
      const std::string & sensor_name,
      const std::string & topic_name,
      double expected_hz);

  virtual ~SensorCheckerBase() = default;

  /**
   * Run the sensor check.
   *
   *  - Invoca perform_check() (implementata nelle classi derivate).
   *  - Logga il risultato (verde OK, rosso FAILED).
   *  - Restituisce true se ok, false altrimenti.
   */
  bool run_check(
      const rclcpp::Logger & logger,
      std::chrono::seconds max_duration = std::chrono::seconds(3),
      std::size_t min_messages = 10);

  const SensorCheckResult & last_result() const { return last_result_; }

  const std::string & sensor_name() const { return sensor_name_; }
  const std::string & topic_name() const { return topic_name_; }
  double expected_hz() const { return expected_hz_; }

protected:
  /**
   * Implementazione specifica del tipo di sensore.
   * Deve:
   *  - effettuare la sottoscrizione al topic
   *  - raccogliere i timestamp
   *  - calcolare la frequenza
   *  - popolare SensorCheckResult (ok, reason, received_messages, measured_hz)
   */
  virtual SensorCheckResult perform_check(
      std::chrono::seconds max_duration,
      std::size_t min_messages) = 0;

  std::string sensor_name_;
  std::string topic_name_;
  double expected_hz_;

private:
  SensorCheckResult last_result_;
};

/**
 * Concrete checker for sensor_msgs::msg::Image sensors.
 *
 * Usa internamente una logica simile alla tua funzione originaria.
 */
class ImageSensorChecker : public SensorCheckerBase
{
public:
  ImageSensorChecker(
      const std::string & sensor_name,
      const std::string & topic_name,
      double expected_hz);

protected:
  SensorCheckResult perform_check(
      std::chrono::seconds max_duration,
      std::size_t min_messages) override;
};

/**
 * Optional legacy-style API: keep a simple functional wrapper
 * so you can call it exactly like prima nella tua pipeline.
 */
bool check_image_sensor(
    const std::string & sensor_name,
    const std::string & topic_name,
    double expected_hz,
    const rclcpp::Logger & logger,
    std::chrono::seconds max_duration = std::chrono::seconds(3),
    std::size_t min_messages = 10);

}  // namespace sensing_utils
