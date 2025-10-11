# launch/opp_centerline.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    csv_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='/home/moon/sim_ws/maps/0927_speed_3.csv',
        description='Centerline CSV 경로'
    )

    return LaunchDescription([
        csv_arg,

        # 1) 센터라인 퍼블리셔 (맵 프레임 기준 경로를 뿌려줌)
        Node(
            package='centerline_planner',
            executable='centerline_pub',
            name='centerline_pub_opp',
            parameters=[{
                'csv_path': LaunchConfiguration('csv_path'),
                'frame_id': 'map',
                'publish_rate': 1.0,
            }],
            output='screen'
        ),

        # 2) Pure Pursuit (상대차의 오돔을 구독하고, 상대차 드라이브 토픽으로 명령 발행)
        Node(
            package='centerline_planner',
            executable='path_pure_pursuit',
            name='path_pure_pursuit_opp',
            parameters=[{
                'wheelbase': 0.33,
                'lookahead_base': 1.0,
                'lookahead_k': 0.3,
                'lookahead_min': 0.7,
                'lookahead_max': 3.0,
                'max_steer': 0.4,
                'speed_min': 1.5,
                'speed_max': 3.0,
            }],
            # ★ 여기서 상대차 토픽으로 리매핑 ★
            remappings=[
                # 노드 내부가 /ego_racecar/odom을 구독한다면 → /opp_racecar/odom 으로
                ('/ego_racecar/odom', '/opp_racecar/odom'),
                # 드라이브 명령을 /drive로 발행한다면 → /opp_drive 로
                ('/drive', '/opp_drive'),
                # 혹시 상대차 로컬 오돔 토픽 이름이 'odom'이라면 아래도 시도 가능:
                ('odom', '/opp_racecar/odom'),
            ],
            output='screen'
        ),
    ])
