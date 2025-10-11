from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 원하는 centerline csv 지정
    csv = '/home/moon/sim_ws/maps/0927_speed_3.csv'  # 바꿔도 됨

    return LaunchDescription([
        Node(
            package='centerline_planner',
            executable='centerline_pub',
            name='centerline_pub',
            parameters=[{
                'csv_path': csv,
                'frame_id': 'map',
                'publish_rate': 1.0,
            }],
            output='screen'
        ),
        Node(
            package='centerline_planner',
            executable='path_pure_pursuit',
            name='path_pure_pursuit',
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
            output='screen'
        ),
    ])
