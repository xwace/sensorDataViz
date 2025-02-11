## 该分支的作用
1. 用于将扫地机的一些关键信息发送到ROS进行可视化，辅助开发和调试。
2. 目前可显示：
- 祛除畸变后的原始激光雷达点云(红色)
- 转换为360°后robot中大量使用的激光雷达点云(绿色)
- 原始线激光点云(橙色)
- 线激光检测的障碍物点云(深紫色)
- 里程计
以上显示颜色可在rviz中自行调整。

## 使用方法
1. 修改*src/ros_debug/protobuf_rtps/sensor_data_service_impl.hpp*中的IP地址为机器人的对应IP
2. 修改*src/ros_debug/yx_ws/src/debug_robot/launch/start.launch*中**server_ip**为机器人IP和端口号**port**（注意，不要改源码中的IP和端口号）
3. 编译robot.svc并更新到机器人内运行
4. 编译并运行ROS包
```shell
cd src/ros_debug/yx_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
roslaunch debug_robot start.launch
```
如果打印了以下信息，则说明连接正常，否则请确认IP和端口号一致
```txt
server_ip: 10.10.35.228, port: 6558
Connecting to server [10.10.35.228:6558] ...
Connected to server. Start receiving data..
```
5. 通过MobaXterm连接服务器后可打开rviz进行数据可视化
```shell
rviz -d robotlbp/src/ros_debug/yx_ws/src/debug_robot/rviz/nav.rviz
```


---
另：如有疑问，请企业微信@徐保来(Ryan)