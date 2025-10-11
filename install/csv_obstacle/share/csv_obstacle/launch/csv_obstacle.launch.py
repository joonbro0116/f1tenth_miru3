from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csv_obstacle',
            executable='csv_obstacle',
            name='csv_obstacle',
            output='screen',
            parameters=[{
                'outer_csv': '/home/moon/sim_ws/bounds_out/outer_bound_world.csv',
                'inner_csv': '/home/moon/sim_ws/bounds_out/inner_bound_world.csv',
                'bubble_topic': '/bubble_grid',
                'odom_topic': '/ego_racecar/odom',
                'occ_threshold': 50,
                'marker_stride': 2,
                'lane_half_width': 0.6,
                'block_dist': 8.0,
                'publish_blocked': True,
            }],
        )
    ])
