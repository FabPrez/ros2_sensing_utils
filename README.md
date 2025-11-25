# ros2_sensing_utils

sensing_utils is a lightweight ROS 2 utility library designed to validate the operational status of sensors within a robotic perception pipeline.
It provides simple, modular, and extensible sensor checkers that verify whether a sensor is publishing data at a coherent and expected frequency.
Developers can quickly diagnose issues such as inactive sensors, incorrect topics, misconfigured drivers, or unexpected publishing rates.

The library is built around an abstract SensorCheckerBase class, from which specific checkers (e.g., for sensor_msgs::msg::Image) can be derived.
This approach makes the framework easy to extend to new sensor types such as point clouds, laser scans, or IMU streams.

# how to use it

```cpp
#include <sensing_utils/sensor_checker.hpp>

// Example inside a ROS2 node method
bool MyNode::test_sensors()
{
    sensing_utils::ImageSensorChecker image_checker(
        "rgb_camera",
        "/camera/image_raw",
        30.0  // expected frequency in Hz
    );

    bool ok = image_checker.run_check(this->get_logger());

    if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "RGB camera failed the sensor check");
    }

    return ok;
}
```

Alternatively, you can use the simpler functional wrapper:

```cpp
bool ok = sensing_utils::check_image_sensor(
    "rgb_camera",
    "/camera/image_raw",
    30.0,
    this->get_logger()
);
```
