#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class ObstacleAvoider : public rclcpp::Node {
public:
  // disallow implicit conversions
  explicit ObstacleAvoider();

private:

  /// @brief Callback function for the left sensor
  /// @param msg The message received from the sensor
  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  /// @brief Callback function for the right sensor
  /// @param msg The message received from the sensor
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

  double left_sensor_value{0.0};
  double right_sensor_value{0.0};
};