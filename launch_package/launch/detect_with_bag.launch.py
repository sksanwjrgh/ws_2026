from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from datetime import datetime
import os


def generate_launch_description():
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.expanduser(f"~/rosbags/detect_bag_{now}")

    detect_node = Node(
        package='imagery_processing',
        executable='marker_recognition',
        name='marker_recognition',
        output='screen'
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,

            # 영상 토픽
            '/landing/video/compressed',

            # PX4 / landing 관련 토픽
            '/fmu/out/vehicle_odometry',
            '/fmu/out/vehicle_status_v1',
            '/fmu/in/trajectory_setpoint',
            '/fmu/in/vehicle_command',

            # detection / lidar 관련 토픽
            '/landing/coordinates',
            '/fmu/in/distance_sensor',
        ],
        output='screen'
    )

    return LaunchDescription([
        detect_node,
        rosbag_record,
    ])
