开源：https://blog.csdn.net/weixin_52612260/article/details/134124028

# 1.build the map 构建地图
# 方法一：方便，但是pgm图不完整，部分障碍物无法完全识别
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_build_map.launch
rosrun map_server map_saver map:=/projected_map -f /home/sentry/RM/auto_sentry/src/sentry_slam/FAST_LIO/PCD/scans
# 查在fast_lio/PCD下中保存的2d地图scans.yaml,确保其中参数origin[x,y,yaw]不能是nan，如果yaw是nan的话，将其设置为0.

# 方法二：先保存PCD图，然后通过pcd2pgm功能包转成二维珊格图，地图比较完整
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch
roslaunch pcd2pgm run.launch
rosrun map_server map_saver -f /home/sentry/RM/auto_sentry/src/sentry_slam/FAST_LIO/PCD/scans

# 2.navigation 导航

roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_localize.launch
# 用rviz发布初始位姿或者 rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# 英雄 rosrun fast_lio_localization publish_initial_pose.py 0.217 0 0.129 0 0 0
roslaunch sentry_nav sentry_movebase.launch
# 用rviz发布目标点
rosrun sentry_send sentry_send
# 如果使用串口通信将serial_send.cpp中的 /dev/ttyACM0 改为 /dev/ttyUSB0
# 如果提示Unable to open port 使用 sudo chmod 666 /dev/ttyUSB0

roslaunch sentry_control sentry_control.launch

