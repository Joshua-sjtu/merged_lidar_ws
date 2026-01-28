import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id_1      = 'livox_frame_192_168_1_187'
frame_id_2      = 'livox_frame_192_168_1_198'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path_1 = os.path.join(cur_config_path, 'MID360_config1.json')
user_config_path_2 = os.path.join(cur_config_path, 'MID360_config2.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params_1 = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id_1},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path_1},
    {"cmdline_input_bd_code": cmdline_bd_code}
]
livox_ros2_params_2 = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id_2},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path_2},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver_1 = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_1_publisher',
        output='screen',
        parameters=livox_ros2_params_1
        )
    livox_driver_2 = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_2_publisher',
        output='screen',
        parameters=livox_ros2_params_2
        )

    return LaunchDescription([
        livox_driver_1,
        livox_driver_2,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
