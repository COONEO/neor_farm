#include "nano_motor_adaptor_foxy_node.hpp"


// Construction function Initial.
MotorAdaptor::MotorAdaptor() : Node("nano_motor_adaptor_foxy_node")
{
  //
  Odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 100);
  Velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MotorAdaptor::nav_callback, this, _1));

  // Load params from launch file.
  this->get_parameter_or("serial_port", serial_port, std::string("/dev/ttyUSB0"));
  this->get_parameter_or("baudrate_", baudrate_, 115200);
  this->get_parameter_or("control_rate_", control_rate_, 10);
  this->get_parameter_or("rear_odom_correct_param_", rear_odom_correct_param_, 0.0);
  this->get_parameter_or("wheel_distance_", wheel_distance_, 0.214);

  // test load params success ?
  // std::cout<<serial_port<<" + "<<baudrate_<<" + "<<control_rate_<<" +
  // "<<rear_odom_correct_param_<<" + "<<wheel_distance_<<std::endl;

  // try to open port.
  try
  { 
    ser_port_.setPort(serial_port);
    ser_port_.setBaudrate(baudrate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
    ser_port_.setTimeout(timeout);
    ser_port_.open(); // test close
    RCLCPP_WARN(this->get_logger(), "wheel serial open success!");
  }
  catch (serial::IOException &e)
  {
    // RCLCPP_ERROR(this->get_logger(), "Unable to open serial port " << ser_port_.getPort() << " ");
    return;
  }

  // 这两个线程是分别控制小车运动和接收小车的数据反馈的。
  std::thread s(&MotorAdaptor::send_thread, this);
  s.detach();

  // only test whell contol ,close get wheel encoder
  std::thread e(&MotorAdaptor::rec_motors, this);
  e.detach();

  // Get current ROS2 time.
  lastest_response_time_ = this->now();
  lastest_send_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "nano motor_adaptor Initial successfully!");

} // Finished construction function.

MotorAdaptor::~MotorAdaptor()
{
  ser_port_.close();
  RCLCPP_INFO(this->get_logger(), "exit && close serial port.");

  rclcpp::shutdown();
}

/*************************************************************************************************************************
 * 上位机(ubuntu)------->下位机(stm32)
 * tx[0]  tx[1]	 tx[2]	    tx[3]	    tx[4]	      tx[5] tx[6]
 * tx[7]        tx[8]     tx[9]    tx[10] oxff	  0xfe   tag motorA_H8 motorA_L8
 * motorB_H8	 motorB_L8	 motorA_D	  motorB_D    0x00	   校验
 *  **********************************************************************************************************************/
void MotorAdaptor::calcByteToWrite(int16_t &Encoder_right_, int16_t &Encoder_left_, Vel &v)
{
  double V_right_ = v.linear_ + v.angle_ * wheel_distance_ / 2;
  double V_left_ = v.linear_ - v.angle_ * wheel_distance_ / 2;

  Encoder_left_ = (int16_t)(V_left_ * 1000 + 5000);
  Encoder_right_ = (int16_t)(V_right_ * 1000 + 5000);

  // std::cout<<"Send----> V_left: "<<V_left<<"  V_right:
  // "<<Encoder_right_<<std::endl;
}

/***************************************************************************************************************************************************************************
* 当下位机收到 ”0xff,0xfe,'s'“指令后，会发送:速度 电机温度 驱动器状态的数据
* 帧头  帧头  标志位	 tx[0]	   tx[1]      tx[2]	   tx[3] tx[4] tx[5]
tx[6]   tx[7]     tx[8]    tx[9]      tx[10]   tx[11]     校验     共16字节
* 0xff	0xfc ‘e’     motorA_H8	motorA_L8  motorB_H8 motorB_L8   TemA_H8 TemA_L8
TemB_H   TemB_L8  State_H1  State_H2  State_H3  State_H4   异或校验
* motorA_H8 motorA_L8 motorB_H8 motorB_L8 （这里传输时已经乘以1000
并加上5000了，所以下位机结算时需要除以1000，然后减去5000之后才是每个轮子是多少m/s）
* 左右电机温度（摄氏度）：手册提供的范围是：0-1200度，然后分高低8位置传输。
乘以的目的是保证小数
* 左右电机速度（m/s）：将原始的速度值乘以1000，然后再加上5000后，分高低8位传输。
乘以的目的是保证小数，加上的目的是保证为正数
* 7、电机和驱动器状态：见右边，驱动器一共使用32bit表示状态。因此，传输的时候将这32bit分为4个字节传输。
        其中state_H1,state_H2,state_H3,state_H4,分别代表的是这32bit的
24-32、16-24、8-16、0-8 bit。
 *****************************************************************************************************************************************************************************/
bool MotorAdaptor::calcVelocity(uint8_t *rx, double &vehicle_linear_, double &vehicle_angle_)
{
  //数据帧头正确且校验正确
  if (rx[15] == check_data(rx, 3, 7) && (rx[1] == 0xfc))
  {
    uint16_t U_Velocity_A = 0;
    uint16_t U_Velocity_B = 0;
    int16_t Velocity_A = 0x00;
    int16_t Velocity_B = 0x00;

    double v_left;
    double v_right;

    //存储在无符号16位中间变量里面
    U_Velocity_A = ((U_Velocity_A | rx[3]) << 8) | rx[4];
    U_Velocity_B = ((U_Velocity_B | rx[5]) << 8) | rx[6];

    //存储在有符号16位中间变量里面
    Velocity_A = (int16_t)U_Velocity_A;
    Velocity_B = (int16_t)U_Velocity_B;

    v_left = ((Velocity_A + rear_odom_correct_param_) - 5000.0) / 1000.0;
    v_right = ((Velocity_B + rear_odom_correct_param_) - 5000.0) / 1000.0;

    vehicle_linear_ = (v_left + v_right) / 2;
    vehicle_angle_ = (v_right - v_left) / wheel_distance_;

    // std::cout<<"Get Motor Velocity Left right :"<<v_left<<"  "<<v_right<<"
    // m/s "<<std::endl;   // 按照规则解算之后的左右轮速度值 std::cout<<"Get
    // Motor Velocity Left right :"<<Velocity_A<<"  "<<Velocity_B<<std::endl; //
    // 打印接收到的原始值

    return 1;
  }
  else //检验不正确
  {
    RCLCPP_INFO(this->get_logger(), "Check or Frame header error!");
    return 0;
  }
}

/*************************************
 * 函数名: 异或校验函数
 * 参数:   需要校验的数组 ， 数组长度
 * 返回值： 异或校验数值
 *************************************/
uint8_t MotorAdaptor::check_data(uint8_t cmd[], int begin, int end)
{
  int i;
  uint8_t check = cmd[begin];

  for (i = begin + 1; i < end; i++)
    check ^= cmd[i];

  return check;
}

void MotorAdaptor::send_thread()
{
  uint8_t cmd[] = {0xff, 0xfe, 's', 0, 0, 0, 0, 0};
  uint16_t left_encoder_temp_;
  uint16_t right_encoder_temp_;

  rclcpp::WallRate loop_rate(control_rate_);
  while (rclcpp::ok())
  {
    if (recv_nav_tag_ == true)
    {
      Vel send_ = Rec_cmd_vel_;
      calcByteToWrite(Encoder_right_, Encoder_left_, send_);
      left_encoder_temp_ = (uint16_t)Encoder_left_;
      right_encoder_temp_ = (uint16_t)Encoder_right_;

      cmd[3] = (uint8_t)(left_encoder_temp_ >> 8);  // target_a 高八位
      cmd[4] = (uint8_t)(left_encoder_temp_);       // target_a 低八位
      cmd[5] = (uint8_t)(right_encoder_temp_ >> 8); // target_b 高八位
      cmd[6] = (uint8_t)right_encoder_temp_;        // target_b 低八位
      cmd[7] = check_data(cmd, 3, 7);
      ser_port_.write(cmd, 8);

      left_encoder_temp_ = 0.0;
      right_encoder_temp_ = 0.0;
      recv_nav_tag_ = false;
    }
    else
    {
      cmd[3] = (uint8_t)(5000 >> 8); // 车轮速度 0 ，这里添加了偏移量 5000；
      cmd[4] = (uint8_t)5000;
      cmd[5] = (uint8_t)(5000 >> 8);
      cmd[6] = (uint8_t)5000;
      cmd[7] = check_data(cmd, 3, 7);
      ser_port_.write(cmd, 8);
    }

    Rec_cmd_vel_.linear_ = 0.0;
    Rec_cmd_vel_.angle_ = 0.0;

    loop_rate.sleep();
  }
}

/********************************************************************************************************************************************************
 * 当下位机收到 ”0xff,0xfe,'s'“指令后，会发送:速度
 * 帧头  帧头  标志位	 tx[0]	   tx[1]      tx[2]	   tx[3] 校验 共8字节
 * 0xff	0xfc ‘e’     motorA_H8	motorA_L8  motorB_H8	motorB_L8   异或校验
 * motorA_H8 motorA_L8 motorB_H8 motorB_L8 （这里传输时已经乘以1000
 *并加上5000了，所以下位机结算时需要除以1000，然后减去5000之后才是每个轮子是多少m/s）
 * 左右电机速度（m/s）：将原始的速度值乘以1000，然后再加上5000后，分高低8位传输。
 *乘以的目的是保证小数，加上的目的是保证为正数
 ********************************************************************************************************************************************************/
void MotorAdaptor::rec_motors()
{
  uint8_t resp[8];
  uint8_t encoder_cmd[8] = {0xff, 0xfe, 'e', 0x00, 0x00, 0x00, 0x00, 0x00};
  double vel_x_ = 0.0;
  double vel_y_ = 0.0;
  double diff_ = 0.0;
  double average_linear_ = 0.0;
  double average_angle_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Start read encoder");
  rclcpp::WallRate loop_rate(100); // while 循环执行的频率
  while (rclcpp::ok())
  {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.frame_id = "wheel_odom";
    odom_msg.child_frame_id = "base_link";

    //发送获取编码值标志位，先下发查询指令　0xff 0xfe e
    ser_port_.write(encoder_cmd, 8);
    //读取有效数据位,发送完毕后就去读信息，如果读取到的是16 个字节，则验算，解算
    if (ser_port_.read(resp, 8) == 8)
    {
      // RCLCPP_INFO(this->get_logger(),"encoder recv: %x, %x, %x, %x ,%x, %x, %x, %x", resp[0], resp[1], resp[2],resp[3], resp[4], resp[5],resp[6],resp[7]);

      rclcpp::Time now = this->now();
      diff_ = (now - lastest_response_time_).seconds();
      if (!calcVelocity(resp, vehicle_linear_, vehicle_angle_))
        continue;

      average_linear_ = (lastest_linear_ + vehicle_linear_) / 2;
      average_angle_ = (lastest_angle_ + vehicle_angle_) / 2;

      vel_x_ = vehicle_linear_ * std::cos(average_angle_);
      vel_y_ = vehicle_linear_ * std::sin(average_angle_);

      odom_angle_ += average_angle_ * diff_;
      odom_linear_x_ += (average_linear_ * diff_) * std::cos(odom_angle_);
      odom_linear_y_ += (average_linear_ * diff_) * std::sin(odom_angle_);

      tf2::Quaternion q;
      q.setRPY(0, 0, odom_angle_);

      //保留上一次的速度值，与下一次接收的速度做均值计算
      lastest_linear_ = vehicle_linear_;
      lastest_angle_ = vehicle_angle_;

      odom_msg.pose.pose.position.x = odom_linear_x_;
      odom_msg.pose.pose.position.y = odom_linear_y_;
      odom_msg.pose.pose.position.z = 0;
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();
      odom_msg.pose.covariance = odom_pose_covariance;

      odom_msg.twist.twist.linear.x = vel_x_;
      odom_msg.twist.twist.linear.y = vel_y_;
      odom_msg.twist.twist.linear.z = 0;
      odom_msg.twist.twist.angular.z = vehicle_angle_;
      odom_msg.twist.covariance = odom_twist_covariance;
      odom_msg.header.stamp = this->now();

      lastest_response_time_ = now;

      Odom_publisher_->publish(odom_msg);
    }
    else
    {
    }
    loop_rate.sleep();
  }
}

void MotorAdaptor::nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  recv_nav_tag_ = true;

  // std::cout<<"linear: "<<msg->linear.x<<" angle:
  // "<<msg->angular.z<<std::endl;

  Rec_cmd_vel_.linear_ = msg->linear.x;
  Rec_cmd_vel_.angle_ = msg->angular.z;

  // push.linear_ = msg->linear_.x;                       // linear_ velocity
  // push.angle_ = msg->angular.z;                       // angle_ velocity
  // if(nav_queue_.size() > 1) //若队列满了，就会移除队头元素，然后
  //         nav_queue_.pop();
  // nav_queue_.push(push);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorAdaptor>());
  rclcpp::shutdown();
  return 0;
}