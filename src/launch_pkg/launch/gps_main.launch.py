import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- 패키지 경로 설정 ---
    decision_making_pkg = get_package_share_directory('decision_making_pkg')
    launch_pkg = get_package_share_directory('launch_pkg')
    ublox_dgnss_pkg = get_package_share_directory('ublox_dgnss')

    # --- GPS 연동을 위한 노드들 ---

    # 1. ublox_dgnss GPS 드라이버 실행
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_dgnss_pkg, 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py')
        )
    )

    # 2. robot_localization (좌표 변환) 실행
    ekf_config_path = os.path.join(launch_pkg, 'config', 'ekf.yaml')

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/gps/fix', '/fix'),
            ('/odometry/filtered', '/odometry/gps')
        ]
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/vehicle_state')]
    )

    # --- h-mobility 자율주행 노드들 ---

    path_planner_node = Node(
        package='decision_making_pkg',
        executable='path_planner',
        name='path_planner_node',
        output='screen'
    )

    motion_planner_node = Node(
        package='decision_making_pkg',
        executable='motion_planner',
        name='motion_planner_node',
        output='screen'
    )

    return LaunchDescription([
        # GPS 관련 노드들
        ublox_launch,
        navsat_transform_node,
        ekf_filter_node,

        # 자율주행 핵심 노드들
        path_planner_node,
        motion_planner_node
    ])