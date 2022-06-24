![](images/neor.jpg)

1. mini_motor_adaptor_foxy.launch.py file explanation.

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            name='nano_motor_adaptor_foxy_node',
            package='nano_motor_adaptor_foxy',
            executable='nano_motor_adaptor_foxy_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/Stm32_PORT',
                'serial_baudrate': 115200, 
                'control_rate_': '10',
                'rear_odom_correct_param_': 0,
                'wheel_distance_': 0.214,
            }],
        ),
    ])
```

serial_port:   Ubuntu 系统下的 STM32 控制板串口重映射 ID 。

serial_baudrate: 波特率

control_rate_: 上位机往下位机 发送速度指令的频率

rear_odom_correct_param_: 速度积分里程计的累积误差

wheel_distance_: 差速类型的两驱动轮之间的间距 （ 单位: m）

2. 串口绑定

```bash
sudo vim /etc/udev/rules.d/neor_mini_arduino.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="....", ATTRS{idProduct}=="....",MODE:="0777", SYMLINK+="Stm32_PORT"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",MODE:="0777", SYMLINK+="IMU_PORT"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",MODE:="0777", SYMLINK+="LIDAR_PORT"
```

然后,重启 udev 规则：

```
sudo service udev reload
sudo service udev restart
```

重新插拔 IMU LIDAR Stm32 即可生效

3. 编译

因为需要对串口进行操作，这里使用的 通过 ROS1 serial 改写过来的 ROS2 serial 功能包，必须和 nano_motor_adaptor_foxy 放置在同一 src 文件夹下才能成功编译。

```bash
cd nano_ws/
colcon build
source install/setup.bash
ros2 launch nano_motor_adaptor_foxy mini_motor_adaptor_foxy.launch.py
```
