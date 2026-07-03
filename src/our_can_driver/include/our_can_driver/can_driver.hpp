#ifndef OUR_CAN_DRIVER__CAN_DRIVER_HPP_
#define OUR_CAN_DRIVER__CAN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

namespace our_can_driver
{

enum class DriveMode {
  MANUAL,      // 수동 모드
  AUTONOMOUS   // 자율주행 모드
};

enum class FlipperMode {
  VELOCITY,    // 속도 모드 (누르는 동안 회전)
  POSITION     // 위치 모드 (버튼 한 번에 목표 각도)
};

class CanDriver : public rclcpp::Node
{
public:
  CanDriver();
  ~CanDriver();

private:
  bool initCAN(const std::string & can_interface);
  void sendVelocity(int motor_id, float rpm);
  void sendPosition(int motor_id, float degree);
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
  float flipper_speed_{50.0f};
  float flipper_position_step_{30.0f};  // 위치 모드 한 번에 30도
  float flipper_max_degree_{90.0f};     // 플리퍼 최대 각도
  float flipper_min_degree_{-90.0f};    // 플리퍼 최소 각도

  // PS4 버튼 인덱스
  // buttons
  const int BTN_L1      = 4;   // 플리퍼 올림
  const int BTN_R1      = 5;   // 플리퍼 내림
  const int BTN_SHARE   = 8;   // 수동/자율주행 전환
  const int BTN_OPTIONS = 9;   // 플리퍼 속도/위치 모드 전환
  const int BTN_PS      = 10;  // 비상 정지

  // axes
  const int AXIS_LEFT_X = 0;   // 왼쪽 스틱 좌우 (wz)
  const int AXIS_LEFT_Y = 1;   // 왼쪽 스틱 상하 (vx)

  // 현재 모드
  DriveMode drive_mode_{DriveMode::MANUAL};
  FlipperMode flipper_mode_{FlipperMode::VELOCITY};

  // 이전 버튼 상태 (토글용)
  int prev_btn_l1_{0};
  int prev_btn_r1_{0};
  int prev_btn_share_{0};
  int prev_btn_options_{0};
  int prev_btn_ps_{0};

  // 현재 플리퍼 각도 (위치 모드용)
  float flipper_current_degree_{0.0f};

  // 현재 RPM 피드백
  float rpm_left_{0.0f};
  float rpm_right_{0.0f};

  // ROS2
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace our_can_driver

#endif  // OUR_CAN_DRIVER__CAN_DRIVER_HPP_