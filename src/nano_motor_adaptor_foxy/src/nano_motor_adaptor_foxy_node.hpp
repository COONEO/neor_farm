#include <queue>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <serial/serial.h>


using std::placeholders::_1;

#define PI_ 3.1415926
static bool recv_nav_tag_ = false;

// 自定义的存储 线速度和角速度的结构体类型
struct Vel
{
  double linear_ = 0.0;
  double angle_ = 0.0;
};
static Vel Rec_cmd_vel_;

// 姿势协方差
std::array<double, 36>odom_pose_covariance = 
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3};

// 转动协方差
std::array<double, 36>odom_twist_covariance = 
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3};


class MotorAdaptor : public rclcpp::Node
{
public:
  MotorAdaptor();
  ~MotorAdaptor();

  bool calcVelocity(uint8_t *, double &, double &);
  void calcByteToWrite(int16_t &, int16_t &, Vel &);
  uint8_t check_data(uint8_t cmd[], int begin, int end);
  void send_thread();
  void rec_motors();
  void Wheel_odom_Pub();

private:
  void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odom_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Velocity_subscriber_;

  rclcpp::Time lastest_response_time_;
  rclcpp::Time lastest_send_time_;

  // params loaded from launch file.
  std::string serial_port;
  int baudrate_;
  int control_rate_;
  double rear_odom_correct_param_; 

  double diff_send_;

  double lastest_linear_ = 0;
  double lastest_angle_ = 0;
  double odom_linear_x_ = 0;
  double odom_linear_y_ = 0;
  double odom_angle_ = 0;

  double vehicle_linear_;
  double vehicle_angle_;
  int16_t Encoder_right_ = 0;
  int16_t Encoder_left_ = 0;

  // Drive wheel distance (m)
  double wheel_distance_;
  serial::Serial ser_port_;

};