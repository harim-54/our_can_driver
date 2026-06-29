#ifndef OUR_CAN_DRIVER__CAN_DRIVER_HPP_
#define OUR_CAN_DRIVER__CAN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

namespace our_can_driver
{

/**
 * @class CanDriver
 * @brief RMD-X8-120 CAN 통신 드라이버
 *
 * /cmd_vel 구독 → CAN으로 모터 명령
 * CAN에서 RPM 읽기 → /joint_states 발행
 */
class CanDriver : public rclcpp::Node
{
public:
  CanDriver();
  ~CanDriver();

private:
  // CAN 소켓 초기화
  bool initCAN(const std::string & can_interface);

  // 모터에 속도 명령 전송
  void sendVelocity(int motor_id, float rpm);

  // 모터 피드백 읽기
  void readFeedback();

  // /cmd_vel 콜백
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 타이머 콜백 (주기적 피드백 읽기)
  void timerCallback();

  // vx, wz → 좌우 RPM 변환
  void toWheelRPM(
    float vx, float wz,
    float & rpm_left, float & rpm_right);

  // CAN 소켓
  int can_socket_{-1};

  // 파라미터
  int left_motor_id_{1};
  int right_motor_id_{2};
  float track_width_{0.5f};
  float wheel_radius_{0.1f};
  float gear_ratio_{6.0f};

  // 현재 RPM
  float rpm_left_{0.0f};
  float rpm_right_{0.0f};

  // ROS2
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace our_can_driver

#endif  // OUR_CAN_DRIVER__CAN_DRIVER_HPP_