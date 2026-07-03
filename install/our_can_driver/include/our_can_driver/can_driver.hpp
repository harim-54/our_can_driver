#ifndef OUR_CAN_DRIVER__CAN_DRIVER_HPP_
#define OUR_CAN_DRIVER__CAN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

namespace our_can_driver
{

class CanDriver : public rclcpp::Node
{
public:
  CanDriver();
  ~CanDriver();

private:
  bool initCAN(const std::string & can_interface);
  void sendVelocity(int motor_id, float rpm);
  void readFeedback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timerCallback();
  void toWheelRPM(
    float vx, float wz,
    float & rpm_left, float & rpm_right);

  // CAN 소켓
  int can_socket_{-1};

  // 파라미터
  int left_motor_id_{1};
  int right_motor_id_{2};
  int flipper_motor_id_{3};
  float track_width_{0.4904f};
  float wheel_radius_{0.225f};
  float gear_ratio_{6.0f};
  float flipper_speed_{50.0f};  // 플리퍼 속도 RPM

  // 버튼 인덱스 (조이스틱 설정에 따라 변경)
  int flipper_up_button_{4};    // L1 버튼
  int flipper_down_button_{5};  // R1 버튼

  // 현재 RPM
  float rpm_left_{0.0f};
  float rpm_right_{0.0f};

  // ROS2
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace our_can_driver

#endif  // OUR_CAN_DRIVER__CAN_DRIVER_HPP_