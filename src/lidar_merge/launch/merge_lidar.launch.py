from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 可选参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    return LaunchDescription([


        # 1. 静态 TF：lidar2 相对于 lidar1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar2_to_lidar1',
            arguments=[
                '0.0', '-0.1', '0',      # x y z (米，实际标定值)
                '0.0', '0.0', '0.0', '1.0',  # 四元数
                'livox_frame_192_168_1_187',   # parent
                'livox_frame_192_168_1_198'    # child
            ],
            output='screen',
        ),

        # 2. 启动 merge 节点
        Node(
            package='lidar_merge',
            executable='merge_point_node',
            name='merge_point_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])