#include "our_can_driver/can_driver.hpp"
#include <cmath>
#include <cstring>

namespace our_can_driver
{

CanDriver::CanDriver()
: Node("can_driver")
{
  // 파라미터 선언
  declare_parameter("can_interface", "can0");
  declare_parameter("left_motor_id", 1);
  declare_parameter("right_motor_id", 2);
  declare_parameter("flipper_motor_id", 3);
  declare_parameter("track_width", 0.4904f);
  declare_parameter("wheel_radius", 0.225f);
  declare_parameter("gear_ratio", 6.0f);
  declare_parameter("flipper_speed", 50.0f);
  declare_parameter("flipper_up_button", 4);
  declare_parameter("flipper_down_button", 5);

  // 파라미터 읽기
  auto can_interface     = get_parameter("can_interface").as_string();
  left_motor_id_         = get_parameter("left_motor_id").as_int();
  right_motor_id_        = get_parameter("right_motor_id").as_int();
  flipper_motor_id_      = get_parameter("flipper_motor_id").as_int();
  track_width_           = get_parameter("track_width").as_double();
  wheel_radius_          = get_parameter("wheel_radius").as_double();
  gear_ratio_            = get_parameter("gear_ratio").as_double();
  flipper_speed_         = get_parameter("flipper_speed").as_double();
  flipper_up_button_     = get_parameter("flipper_up_button").as_int();
  flipper_down_button_   = get_parameter("flipper_down_button").as_int();

  // CAN 초기화
  if (!initCAN(can_interface)) {
    RCLCPP_ERROR(get_logger(), "CAN 초기화 실패!");
    return;
  }

  // /cmd_vel 구독
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&CanDriver::cmdVelCallback, this, std::placeholders::_1));

  // /joy 구독
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&CanDriver::joyCallback, this, std::placeholders::_1));

  // /joint_states 발행
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  // 주기적 피드백 읽기 (100Hz)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&CanDriver::timerCallback, this));

  RCLCPP_INFO(get_logger(), "CAN 드라이버 시작!");
}

CanDriver::~CanDriver()
{
  if (can_socket_ >= 0) {
    close(can_socket_);
  }
}

bool CanDriver::initCAN(const std::string & can_interface)
{
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(get_logger(), "소켓 생성 실패");
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ);
  ioctl(can_socket_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "소켓 바인딩 실패");
    return false;
  }

  RCLCPP_INFO(get_logger(), "CAN 소켓 초기화 완료: %s", can_interface.c_str());
  return true;
}

void CanDriver::sendVelocity(int motor_id, float rpm)
{
  struct can_frame frame;
  frame.can_id  = motor_id;
  frame.can_dlc = 8;

  int32_t speed_val = static_cast<int32_t>(rpm * 360.0f / 60.0f * 100.0f);

  frame.data[0] = 0xA2;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = speed_val & 0xFF;
  frame.data[5] = (speed_val >> 8) & 0xFF;
  frame.data[6] = (speed_val >> 16) & 0xFF;
  frame.data[7] = (speed_val >> 24) & 0xFF;

  write(can_socket_, &frame, sizeof(frame));
}

void CanDriver::readFeedback()
{
  struct can_frame frame;
  fd_set read_fds;
  struct timeval timeout = {0, 1000};

  FD_ZERO(&read_fds);
  FD_SET(can_socket_, &read_fds);

  if (select(can_socket_ + 1, &read_fds, nullptr, nullptr, &timeout) > 0) {
    if (read(can_socket_, &frame, sizeof(frame)) > 0) {
      if (frame.data[0] == 0xA2) {
        int16_t speed_raw = (frame.data[5] << 8) | frame.data[4];
        float rpm = speed_raw / 100.0f * 60.0f / 360.0f;

        if (frame.can_id == left_motor_id_) {
          rpm_left_ = rpm;
        } else if (frame.can_id == right_motor_id_) {
          rpm_right_ = rpm;
        }
      }
    }
  }
}

void CanDriver::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float rpm_left, rpm_right;
  toWheelRPM(msg->linear.x, msg->angular.z, rpm_left, rpm_right);

  sendVelocity(left_motor_id_, rpm_left);
  sendVelocity(right_motor_id_, rpm_right);
}

void CanDriver::joyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // 플리퍼 버튼 제어
  if (msg->buttons[flipper_up_button_] == 1) {
    // 버튼 누르면 위로
    sendVelocity(flipper_motor_id_, flipper_speed_);
    RCLCPP_INFO(get_logger(), "플리퍼 올림");
  } else if (msg->buttons[flipper_down_button_] == 1) {
    // 버튼 누르면 아래로
    sendVelocity(flipper_motor_id_, -flipper_speed_);
    RCLCPP_INFO(get_logger(), "플리퍼 내림");
  } else {
    // 버튼 놓으면 정지
    sendVelocity(flipper_motor_id_, 0.0f);
  }
}

void CanDriver::timerCallback()
{
  readFeedback();

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();
  msg.name     = {"left_wheel", "right_wheel", "flipper"};
  msg.velocity = {rpm_left_, rpm_right_, 0.0f};

  joint_states_pub_->publish(msg);
}

void CanDriver::toWheelRPM(
  float vx, float wz,
  float & rpm_left, float & rpm_right)
{
  float v_left  = vx - wz * track_width_ / 2.0f;
  float v_right = vx + wz * track_width_ / 2.0f;

  rpm_left  = v_left  / (2.0f * M_PI * wheel_radius_) * 60.0f * gear_ratio_;
  rpm_right = v_right / (2.0f * M_PI * wheel_radius_) * 60.0f * gear_ratio_;
}

}  // namespace our_can_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<our_can_driver::CanDriver>());
  rclcpp::shutdown();
  return 0;
}