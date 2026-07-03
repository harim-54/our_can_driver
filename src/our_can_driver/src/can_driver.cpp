#include "our_can_driver/can_driver.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

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
  declare_parameter("track_width", 0.4904);
  declare_parameter("wheel_radius", 0.225);
  declare_parameter("gear_ratio", 6.0);
  declare_parameter("flipper_speed", 50.0);
  declare_parameter("flipper_position_step", 30.0);
  declare_parameter("flipper_max_degree", 90.0);
  declare_parameter("flipper_min_degree", -90.0);

  // 파라미터 읽기
  auto can_interface     = get_parameter("can_interface").as_string();
  left_motor_id_         = get_parameter("left_motor_id").as_int();
  right_motor_id_        = get_parameter("right_motor_id").as_int();
  flipper_motor_id_      = get_parameter("flipper_motor_id").as_int();
  track_width_           = get_parameter("track_width").as_double();
  wheel_radius_          = get_parameter("wheel_radius").as_double();
  gear_ratio_            = get_parameter("gear_ratio").as_double();
  flipper_speed_         = get_parameter("flipper_speed").as_double();
  flipper_position_step_ = get_parameter("flipper_position_step").as_double();
  flipper_max_degree_    = get_parameter("flipper_max_degree").as_double();
  flipper_min_degree_    = get_parameter("flipper_min_degree").as_double();

  // CAN 초기화
  if (!initCAN(can_interface)) {
    RCLCPP_ERROR(get_logger(), "CAN 초기화 실패!");
    return;
  }

  // /cmd_vel 구독 (자율주행 모드에서 MPPI 명령)
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

  // /auto_mode 발행
  auto_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/auto_mode", 10);

  // 주기적 피드백 읽기 (100Hz)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&CanDriver::timerCallback, this));

  RCLCPP_INFO(get_logger(), "CAN 드라이버 시작!");
  RCLCPP_INFO(get_logger(), "모드: 수동 | 플리퍼: 속도 모드");
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
  // RMD-X8-120 속도 제어 명령 (0xA2)
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

void CanDriver::sendPosition(int motor_id, float degree)
{
  // RMD-X8-120 위치 제어 명령 (0xA3)
  // 각도를 0.01도 단위로 변환
  struct can_frame frame;
  frame.can_id  = motor_id;
  frame.can_dlc = 8;

  int32_t angle_val = static_cast<int32_t>(degree * 100.0f);

  frame.data[0] = 0xA3;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = angle_val & 0xFF;
  frame.data[5] = (angle_val >> 8) & 0xFF;
  frame.data[6] = (angle_val >> 16) & 0xFF;
  frame.data[7] = (angle_val >> 24) & 0xFF;

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
  // 자율주행 모드일 때만 처리
  if (drive_mode_ != DriveMode::AUTONOMOUS) {return;}

  float rpm_left, rpm_right;
  toWheelRPM(msg->linear.x, msg->angular.z, rpm_left, rpm_right);

  sendVelocity(left_motor_id_, rpm_left);
  sendVelocity(right_motor_id_, rpm_right);
}

void CanDriver::joyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // =====================
  // 1. 비상 정지 (PS 버튼)
  // =====================
  if (msg->buttons[BTN_PS] == 1 && prev_btn_ps_ == 0) {
    sendVelocity(left_motor_id_, 0.0f);
    sendVelocity(right_motor_id_, 0.0f);
    sendVelocity(flipper_motor_id_, 0.0f);
    drive_mode_ = DriveMode::MANUAL;
    RCLCPP_WARN(get_logger(), "비상 정지! 수동 모드로 전환");

    auto mode_msg = std_msgs::msg::Bool();
    mode_msg.data = false;
    auto_mode_pub_->publish(mode_msg);
  }
  prev_btn_ps_ = msg->buttons[BTN_PS];

  // ==============================
  // 2. 수동/자율주행 전환 (SHARE)
  // ==============================
  if (msg->buttons[BTN_SHARE] == 1 && prev_btn_share_ == 0) {
    if (drive_mode_ == DriveMode::MANUAL) {
      drive_mode_ = DriveMode::AUTONOMOUS;
      RCLCPP_INFO(get_logger(), "자율주행 모드 전환!");
    } else {
      drive_mode_ = DriveMode::MANUAL;
      sendVelocity(left_motor_id_, 0.0f);
      sendVelocity(right_motor_id_, 0.0f);
      RCLCPP_INFO(get_logger(), "수동 모드 전환!");
    }

    auto mode_msg = std_msgs::msg::Bool();
    mode_msg.data = (drive_mode_ == DriveMode::AUTONOMOUS);
    auto_mode_pub_->publish(mode_msg);
  }
  prev_btn_share_ = msg->buttons[BTN_SHARE];

  // ==============================
  // 3. 수동 모드 주행 제어
  // ==============================
  if (drive_mode_ == DriveMode::MANUAL) {
    float vx =  msg->axes[AXIS_LEFT_Y];  // 왼쪽 스틱 상하 → 전진/후진
    float wz =  msg->axes[AXIS_LEFT_X];  // 왼쪽 스틱 좌우 → 좌우 회전

    float rpm_left, rpm_right;
    toWheelRPM(vx, wz, rpm_left, rpm_right);

    sendVelocity(left_motor_id_, rpm_left);
    sendVelocity(right_motor_id_, rpm_right);
  }

  // ==============================
  // 4. 플리퍼 모드 전환 (OPTIONS)
  // ==============================
  if (msg->buttons[BTN_OPTIONS] == 1 && prev_btn_options_ == 0) {
    if (flipper_mode_ == FlipperMode::VELOCITY) {
      flipper_mode_ = FlipperMode::POSITION;
      RCLCPP_INFO(get_logger(), "플리퍼 위치 모드 전환!");
    } else {
      flipper_mode_ = FlipperMode::VELOCITY;
      sendVelocity(flipper_motor_id_, 0.0f);
      RCLCPP_INFO(get_logger(), "플리퍼 속도 모드 전환!");
    }
  }
  prev_btn_options_ = msg->buttons[BTN_OPTIONS];

  // ==============================
  // 5. 플리퍼 제어 (L1/R1)
  // ==============================
  if (flipper_mode_ == FlipperMode::VELOCITY) {
    // 속도 모드: 누르는 동안 회전
    if (msg->buttons[BTN_L1] == 1) {
      sendVelocity(flipper_motor_id_, flipper_speed_);
      RCLCPP_INFO(get_logger(), "플리퍼 올림 (속도 모드)");
    } else if (msg->buttons[BTN_R1] == 1) {
      sendVelocity(flipper_motor_id_, -flipper_speed_);
      RCLCPP_INFO(get_logger(), "플리퍼 내림 (속도 모드)");
    } else {
      sendVelocity(flipper_motor_id_, 0.0f);
    }
  } else {
    // 위치 모드: 버튼 한 번에 목표 각도 이동
    if (msg->buttons[BTN_L1] == 1 && prev_btn_l1_ == 0) {
      // 목표 각도 = 현재 각도 + step (최대 각도 제한)
      flipper_current_degree_ = std::min(
        flipper_current_degree_ + flipper_position_step_,
        flipper_max_degree_);
      sendPosition(flipper_motor_id_, flipper_current_degree_);
      RCLCPP_INFO(
        get_logger(), "플리퍼 위치 이동: %.1f도", flipper_current_degree_);
    } else if (msg->buttons[BTN_R1] == 1 && prev_btn_r1_ == 0) {
      // 목표 각도 = 현재 각도 - step (최소 각도 제한)
      flipper_current_degree_ = std::max(
        flipper_current_degree_ - flipper_position_step_,
        flipper_min_degree_);
      sendPosition(flipper_motor_id_, flipper_current_degree_);
      RCLCPP_INFO(
        get_logger(), "플리퍼 위치 이동: %.1f도", flipper_current_degree_);
    }
  }
  prev_btn_l1_ = msg->buttons[BTN_L1];
  prev_btn_r1_ = msg->buttons[BTN_R1];
}

void CanDriver::timerCallback()
{
  readFeedback();

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();
  msg.name         = {"left_wheel", "right_wheel", "flipper"};
  msg.velocity     = {rpm_left_, rpm_right_, 0.0f};
  msg.position     = {0.0f, 0.0f, flipper_current_degree_};

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