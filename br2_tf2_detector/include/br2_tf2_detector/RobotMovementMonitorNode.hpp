#ifndef BR2_TF2_DETECTOR__ROBOTMOVEMENTMONITORNODE_HPP_
#define BR2_TF2_DETECTOR__ROBOTMOVEMENTMONITORNODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class RobotMovementMonitorNode : public rclcpp::Node
{
public:
  RobotMovementMonitorNode();

private:
  void robot_movement_monitor();
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped last_odom2basefootprint_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr robot_movement_pub_;
};

}  // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__ROBOTMOVEMENTMONITORNODE_HPP_
