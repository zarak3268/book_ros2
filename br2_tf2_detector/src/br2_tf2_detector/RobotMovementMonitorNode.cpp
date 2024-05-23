#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "br2_tf2_detector/RobotMovementMonitorNode.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

using namespace std::chrono_literals;

RobotMovementMonitorNode::RobotMovementMonitorNode()
: Node("robot_movement_monitor"), tf_buffer_(), tf_listener_(tf_buffer_), last_odom2basefootprint_() {
    last_odom2basefootprint_.child_frame_id = "none";
    robot_movement_pub_ = create_publisher<geometry_msgs::msg::Vector3>("robot_movement", 1);
    timer_ = create_wall_timer(
    1s, std::bind(&RobotMovementMonitorNode::robot_movement_monitor, this));
}


void RobotMovementMonitorNode::robot_movement_monitor() {
    geometry_msgs::msg::TransformStamped odom2basefootprint;
    
    try {
        odom2basefootprint = tf_buffer_.lookupTransform(
            "odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Odom2Basefootprint transform not found: %s", ex.what());
        return;
    }

    if (last_odom2basefootprint_.child_frame_id == "none") {
        last_odom2basefootprint_ = odom2basefootprint;
        return;
    }

    geometry_msgs::msg::Vector3 distance;
    distance.x = odom2basefootprint.transform.translation.x - last_odom2basefootprint_.transform.translation.x;
    distance.y = odom2basefootprint.transform.translation.y - last_odom2basefootprint_.transform.translation.y;
    distance.z = odom2basefootprint.transform.translation.z - last_odom2basefootprint_.transform.translation.z;

    robot_movement_pub_->publish(distance); 
}

}