import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_stack')

    # config 파일 경로
    joy_node_config      = os.path.join(pkg, 'config', 'joy.yaml')
    joy_teleop_config    = os.path.join(pkg, 'config', 'joy_teleop.yaml')
    vesc_config          = os.path.join(pkg, 'config', 'vesc.yaml')
    sensors_config       = os.path.join(pkg, 'config', 'sensors.yaml')
    mux_config           = os.path.join(pkg, 'config', 'mux.yaml')
    carto_config         = os.path.join(pkg, 'config', 'backpack_2d.lua')

    # Launch Arguments 선언
    joy_node_la = DeclareLaunchArgument(
        'joy_node_config',
        default_value=joy_node_config,
        description='Path to joy driver config (joy.yaml)'
    )
    joy_teleop_la = DeclareLaunchArgument(
        'joy_teleop_config',
        default_value=joy_teleop_config,
        description='Path to joy_teleop config (joy_teleop.yaml)'
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Path to VESC config (vesc.yaml)'
    )
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Path to sensors config (sensors.yaml)'
    )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Path to ackermann_mux config (mux.yaml)'
    )
    carto_la = DeclareLaunchArgument(
        'carto_config',
        default_value=carto_config,
        description='Path to Cartographer Lua config'
    )

    # LaunchDescription 생성 및 인자 추가
    ld = LaunchDescription([
        joy_node_la,
        joy_teleop_la,
        vesc_la,
        sensors_la,
        mux_la,
        carto_la,
    ])

    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_node_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_teleop_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=[
            '0.27', '0.0', '0.11',
            '0.0', '0.0', '0.0',
            'base_link', 'laser'
        ]
    )
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('carto_config'),
            {'use_sim_time': False}
        ],
        remappings=[('LaserScan', '/scan'), ('odom', '/odom')]
    )

    occ_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            LaunchConfiguration('carto_config'),
            {'use_sim_time': False}
        ]
    )

    # Static transform: map -> odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=[
            '0', '0', '0',    # x, y, z
            '0', '0', '0',    # roll, pitch, yaw
            'map',            # parent frame
            'odom'            # child frame
        ]
    )

    # LaunchDescription에 노드 추가
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(carto_node)
    ld.add_action(occ_node)
    ld.add_action(static_map_to_odom)

    return ld

