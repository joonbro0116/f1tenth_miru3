from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bubble_memory',
            executable='bubble_memory',
            name='bubble_memory',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'map_topic': '/map',
                'odom_topic': '/ego_racecar/odom',
                'roi_deg': 70.0,
                'max_dist': 10.0,
                'margin': 0.30,
                'bubble_radius': 0.35,     # 버블 반경 (m)
                'decay_rate': 0.90,        # 프레임마다 곱해지는 감쇠(0.9~0.99 권장)
                'add_value': 0.7,          # 새 버블 찍을 때 올릴 값(0~1)
                'publish_rate_hz': 15.0,   # 퍼블리시 주기
                'occ_threshold': 0.25       # 이 값 이상이면 점유(>=50)로 변환
            }]
        )
    ])
