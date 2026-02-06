本工作空间包含livox驱动、双雷达点云合并
# 使用方法：
## 配置：
进入src/livox_ros_driver2/multi_config.json 修改ip地址  
进入src/lidar_merge/launch/merge_lidar.launch.py 修改tf  
进入src/lidar_merge/src/merge.cpp 修改主雷达  
## 编译：
在~/Desktop/merged_lidar_ws/src/livox_ros_driver2目录下输入  
`source /opt/ros/humble/setup.sh`  
`./build.sh humble`  
## 运行：
在~/Desktop/merged_lidar_ws目录下  
`sudo ptp4l -i enp1s0 -S -l 6 -m`（启动ptp主时钟）   
`source install/setup.bash`  
`ros2 launch livox_ros_driver2 multi_msg_launch.py`（启动两个雷达与imu数据话题）  
`ros2 launch lidar_merge merge_lidar.launch.py`（启动两雷达相对tf，启动merge节点）  
