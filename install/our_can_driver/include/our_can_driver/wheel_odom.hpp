#ifndef OUR_CAN_DRIVER__WHEEL_ODOM_HPP_
#define OUR_CAN_DRIVER__WHEEL_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace our_can_driver
{

/**
 * @class WheelOdom
 * @brief 엔코더 기반 휠 오도메트리 노드
 *
 * /joint_states 구독 → RPM
 * RPM → vx, wz 계산
 * → /wheel_odom 발행 (EKF 입력)
 */
class WheelOdom : public rclcpp::Node
{
public:
  WheelOdom();

private:
  void jointStatesCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  // RPM → 선속도/각속도
  void fromWheelRPM(
    float rpm_left, float rpm_right,
    float & vx, float & wz);

  // 파라미터
  float track_width_{0.5f};
  float wheel_radius_{0.1f};
  float gear_ratio_{6.0f};

  // 현재 상태
  float x_{0.0f};
  float y_{0.0f};
  float yaw_{0.0f};
  rclcpp::Time last_time_;

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace our_can_driver

#endif  // OUR_CAN_DRIVER__WHEEL_ODOM_HPP_