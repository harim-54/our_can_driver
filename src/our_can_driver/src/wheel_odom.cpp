#include "our_can_driver/wheel_odom.hpp"
#include <cmath>

namespace our_can_driver
{

WheelOdom::WheelOdom()
: Node("wheel_odom")
{
  // 파라미터 선언
  declare_parameter("track_width", 0.5f);
  declare_parameter("wheel_radius", 0.1f);
  declare_parameter("gear_ratio", 6.0f);

  track_width_  = get_parameter("track_width").as_double();
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  gear_ratio_   = get_parameter("gear_ratio").as_double();

  last_time_ = now();

  // /joint_states 구독
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&WheelOdom::jointStatesCallback, this, std::placeholders::_1));

  // /wheel_odom 발행
  wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/wheel_odom", 10);

  // TF 브로드캐스터
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "휠 오도메트리 노드 시작!");
}

void WheelOdom::jointStatesCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.size() < 2) {return;}

  float rpm_left  = msg->velocity[0];
  float rpm_right = msg->velocity[1];

  // dt 계산
  auto current_time = now();
  float dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  if (dt <= 0.0f) {return;}

  // RPM → vx, wz
  float vx, wz;
  fromWheelRPM(rpm_left, rpm_right, vx, wz);

  // 위치 업데이트
  x_   += vx * std::cos(yaw_) * dt;
  y_   += vx * std::sin(yaw_) * dt;
  yaw_ += wz * dt;

  // /wheel_odom 발행
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp    = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x  = vx;
  odom.twist.twist.angular.z = wz;

  wheel_odom_pub_->publish(odom);
}

void WheelOdom::fromWheelRPM(
  float rpm_left, float rpm_right,
  float & vx, float & wz)
{
  // RPM → 선속도 (기어비 고려)
  float v_left  = rpm_left  / gear_ratio_ / 60.0f * 2.0f * M_PI * wheel_radius_;
  float v_right = rpm_right / gear_ratio_ / 60.0f * 2.0f * M_PI * wheel_radius_;

  vx = (v_right + v_left) / 2.0f;
  wz = (v_right - v_left) / track_width_;
}

}  // namespace our_can_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<our_can_driver::WheelOdom>());
  rclcpp::shutdown();
  return 0;
}