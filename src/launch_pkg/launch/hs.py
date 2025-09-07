import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # --- 기존 h-mobility 패키지 경로 설정 (수정 없음) ---
    camera_perception_pkg = get_package_share_directory('camera_perception_pkg')
    lidar_perception_pkg = get_package_share_directory('lidar_perception_pkg')
    decision_making_pkg = get_package_share_directory('decision_making_pkg')
    serial_communication_pkg = get_package_share_directory('serial_communication_pkg')
    debug_pkg = get_package_share_directory('debug_pkg')
    # launch_pkg 경로 추가
    launch_pkg = get_package_share_directory('launch_pkg')

    # --- 기존 다중 카메라 실행 로직 (수정 없음) ---
    # Update this list to match your camera numbers
    cam_numbers = [6] 
    
    camera_nodes = []
    for i in cam_numbers:
        camera_nodes.extend([
            Node(
                package='camera_perception_pkg',
                executable='image_publisher_node',
                name=f'image_publisher_node_{i}',
                output='screen',
                parameters=[
                    {'data_source': 'camera'},
                    {'cam_num': i}, 
                    {'pub_topic': f'image_raw_{i}'}
                ]
            ),
            Node(
                package='camera_perception_pkg',
                executable='yolov8_node',
                name=f'yolov8_node_{i}',
                output='screen',
                remappings=[
                    ('image_raw', f'image_raw_{i}'),
                    ('detections', f'detections_{i}')
                ]
            ),
            Node(
                package='camera_perception_pkg',
                executable='lane_info_extractor_node',
                name=f'lane_info_extractor_node_{i}',
                output='screen',
                parameters=[
                    {'sub_detection_topic': f'detections_{i}'},
                    {'roi_pub_topic': f'roi_image_{i}'},
                    {'cam_num': i}
                ]
            ),
            Node(
                package='debug_pkg',
                executable='yolov8_visualizer_node',
                name=f'yolov8_visualizer_node_{i}',
                output='screen',
                remappings=[
                    ('image_raw', f'image_raw_{i}'),
                    ('detections', f'detections_{i}'),
                    ('yolov8_visualized_img', f'yolov8_visualized_img_{i}')
                ]
            )
        ])
        
    # ==================== GPS 연동을 위해 새로 추가하는 부분 ====================

    # 1. ublox_dgnss GPS 드라이버 노드 실행
    # Rover(자율주행차)에 가장 적합한 launch 파일을 포함시킵니다.
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ublox_dgnss'), 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py')
        )
    )

    # 2. robot_localization (좌표 변환 및 필터링) 노드 실행
    # 이전에 생성한 ekf.yaml 설정 파일의 경로를 지정합니다.
    ekf_config_path = os.path.join(launch_pkg, 'config', 'ekf.yaml')

    # NavSatFix(위도/경도) -> Odometry(지역좌표) 변환 노드
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            # ublox_dgnss가 발행하는 /fix 토픽을 입력으로 받습니다.
            ('/gps/fix', '/fix'),
            # 변환된 Odometry를 EKF 노드의 입력으로 보냅니다.
            ('/odometry/filtered', '/odometry/gps') 
        ]
    )

    # EKF(확장 칼만 필터) 노드
    # GPS 데이터를 필터링하여 최종 위치를 추정하고, h-mobility 코드가 사용하는 /vehicle_state 토픽으로 발행합니다.
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            # 핵심: EKF의 최종 출력(/odometry/filtered)을 /vehicle_state로 이름을 바꿔줍니다.
            ('/odometry/filtered', '/vehicle_state')
        ]
    )

    # =======================================================================
    
    # h-mobility 자율주행 노드들을 실행 목록에 추가합니다.
    # path_planner_node는 /vehicle_state 토픽을 구독하여 경로를 계획합니다.
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

    # 실행할 모든 노드들을 LaunchDescription에 담아 반환합니다.
    ld = LaunchDescription()
    
    # 카메라 노드들 추가
    for node in camera_nodes:
        ld.add_action(node)
        
    # GPS 관련 노드들 추가
    ld.add_action(ublox_launch)
    ld.add_action(navsat_transform_node)
    ld.add_action(ekf_filter_node)
    
    # 자율주행 핵심 노드들 추가
    ld.add_action(path_planner_node)
    ld.add_action(motion_planner_node)

    return ld
