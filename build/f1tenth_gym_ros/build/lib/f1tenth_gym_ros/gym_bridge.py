# MIT License
# Copyright (c) 2020 Hongrui Zheng
#
# Modified for Gym 0.26+ / f110_gym recent API (reset(options=...), step -> 5 returns)

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler


class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # -------- Parameters --------
        self.declare_parameter('ego_namespace')
        self.declare_parameter('ego_odom_topic')
        self.declare_parameter('ego_opp_odom_topic')
        self.declare_parameter('ego_scan_topic')
        self.declare_parameter('ego_drive_topic')

        self.declare_parameter('opp_namespace')
        self.declare_parameter('opp_odom_topic')
        self.declare_parameter('opp_ego_odom_topic')
        self.declare_parameter('opp_scan_topic')
        self.declare_parameter('opp_drive_topic')

        self.declare_parameter('scan_distance_to_base_link')
        self.declare_parameter('scan_fov')
        self.declare_parameter('scan_beams')

        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext')
        self.declare_parameter('num_agent')

        self.declare_parameter('sx')
        self.declare_parameter('sy')
        self.declare_parameter('stheta')

        self.declare_parameter('sx1')
        self.declare_parameter('sy1')
        self.declare_parameter('stheta1')

        self.declare_parameter('kb_teleop')

        num_agents = self.get_parameter('num_agent').value
        if not isinstance(num_agents, int) or num_agents < 1 or num_agents > 2:
            raise ValueError('num_agent should be int and either 1 or 2.')

        # -------- Make environment (Gym 0.26+) --------
        self.env = gym.make(
            'f110_gym:f110-v0',
            map=self.get_parameter('map_path').value,
            map_ext=self.get_parameter('map_img_ext').value,
            num_agents=num_agents,
            lidar_dist=self.get_parameter('scan_distance_to_base_link').value,
            disable_env_checker=True,  # turn off passive checker warnings
        )

        # -------- Common state --------
        sx = float(self.get_parameter('sx').value)
        sy = float(self.get_parameter('sy').value)
        stheta = float(self.get_parameter('stheta').value)
                # -------- Common state --------
                
        # 내가 추가
        sx1 = float(self.get_parameter('sx1').value)
        sy1 = float(self.get_parameter('sy1').value)
        stheta1 = float(self.get_parameter('stheta1').value)
        
        
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        
        # 내가 추가
        self.opp_pose = [sx1, sy1, stheta1]
        self.opp_speed = [0.0, 0.0, 0.0]
        self.opp_requested_speed = 0.0
        self.opp_steer = 0.0
        
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        
        # 내가 추가
        self.opp_namespace = self.get_parameter('opp_namespace').value
        opp_scan_topic = self.get_parameter('opp_scan_topic').value
        opp_drive_topic = self.get_parameter('opp_drive_topic').value
        opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('opp_odom_topic').value

        scan_fov = float(self.get_parameter('scan_fov').value)
        scan_beams = int(self.get_parameter('scan_beams').value)
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = float(self.get_parameter('scan_distance_to_base_link').value)

        # -------- Opponent (optional) --------
        self.has_opp = (num_agents == 2)
        if self.has_opp:
            poses = np.array([[sx, sy, stheta], [sx1, sy1, stheta1]], dtype=float)
            self.obs, _ = self._compat_reset(poses)
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])
        else:
            poses = np.array([[sx, sy, stheta]], dtype=float)
            self.obs, _ = self._compat_reset(poses)
            self.ego_scan = list(self.obs['scans'][0])

        # -------- ROS timers --------
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)  # physics
        self.timer = self.create_timer(0.004, self.timer_callback)             # pub topics

        # -------- TF Broadcaster --------
        self.br = TransformBroadcaster(self)

        # -------- Publishers --------
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False

        if self.has_opp:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            # self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            # self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        # -------- Subscribers --------
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10
        )
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10
        )
        if self.has_opp:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped, opp_drive_topic, self.opp_drive_callback, 10
            )
            self.opp_reset_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self.opp_reset_callback, 10
            )

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist, '/cmd_vel', self.teleop_callback, 10
            )

    # ------------ Callbacks ------------

    def drive_callback(self, drive_msg: AckermannDriveStamped):
        self.ego_requested_speed = float(drive_msg.drive.speed)
        self.ego_steer = float(drive_msg.drive.steering_angle)
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg: AckermannDriveStamped):
        self.opp_requested_speed = float(drive_msg.drive.speed)
        self.opp_steer = float(drive_msg.drive.steering_angle)
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg: PoseWithCovarianceStamped):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            poses = np.array([[rx, ry, rtheta], opp_pose], dtype=float)
        else:
            poses = np.array([[rx, ry, rtheta]], dtype=float)
        self.obs, _ = self._compat_reset(poses)
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg: PoseStamped):
        if not self.has_opp:
            return
        rx = pose_msg.pose.position.x
        ry = pose_msg.pose.position.y
        rqx = pose_msg.pose.orientation.x
        rqy = pose_msg.pose.orientation.y
        rqz = pose_msg.pose.orientation.z
        rqw = pose_msg.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        poses = np.array([list(self.ego_pose), [rx, ry, rtheta]], dtype=float)
        self.obs, _ = self._compat_reset(poses)
        self._update_sim_state()

    def teleop_callback(self, twist_msg: Twist):
        if not self.ego_drive_published:
            self.ego_drive_published = True
        self.ego_requested_speed = float(twist_msg.linear.x)
        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    # ------------ Timers ------------

    def drive_timer_callback(self):
        if self.ego_drive_published and not self.has_opp:
            actions = np.array([[self.ego_steer, self.ego_requested_speed]], dtype=float)
            self.obs = self._compat_step(actions)
            self._update_sim_state()
        elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
            actions = np.array([
                [self.ego_steer, self.ego_requested_speed],
                [self.opp_steer, self.opp_requested_speed]
            ], dtype=float)
            self.obs = self._compat_step(actions)
            self._update_sim_state()


    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # publish scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_max)
        scan.angle_increment = float(self.angle_inc)
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = list(map(float, self.ego_scan))
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = float(self.angle_min)
            opp_scan.angle_max = float(self.angle_max)
            opp_scan.angle_increment = float(self.angle_inc)
            opp_scan.range_min = 0.0
            opp_scan.range_max = 30.0
            opp_scan.ranges = list(map(float, self.opp_scan))
            self.opp_scan_pub.publish(opp_scan)

        # publish TF + odom
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    # ------------ Helpers ------------

    def _update_sim_state(self):
        # scans
        self.ego_scan = list(self.obs['scans'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])

        # ego state
        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])

        # opp state
        if self.has_opp:
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_q = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_q[1]
        ego_odom.pose.pose.orientation.y = ego_q[2]
        ego_odom.pose.pose.orientation.z = ego_q[3]
        ego_odom.pose.pose.orientation.w = ego_q[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_q = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_q[1]
            opp_odom.pose.pose.orientation.y = opp_q[2]
            opp_odom.pose.pose.orientation.z = opp_q[3]
            opp_odom.pose.pose.orientation.w = opp_q[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            # self.opp_odom_pub.publish(opp_odom)
            # # cross-publish for ego<->opp odom topics if used by other nodes
            # self.opp_ego_odom_pub.publish(ego_odom)
            # self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        # ego base_link in map
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_q = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_q[1]
        ego_t.rotation.y = ego_q[2]
        ego_t.rotation.z = ego_q[3]
        ego_t.rotation.w = ego_q[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_q = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_q[1]
            opp_t.rotation.y = opp_q[2]
            opp_t.rotation.z = opp_q[3]
            opp_t.rotation.w = opp_q[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        # steer angle applied to front hinges -> wheels
        ego_wheel_ts = TransformStamped()
        ego_wheel_q = euler.euler2quat(0.0, 0.0, self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_q[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_q[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_q[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_q[0]
        ego_wheel_ts.header.stamp = ts

        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)

        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_q = euler.euler2quat(0.0, 0.0, self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_q[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_q[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_q[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_q[0]
            opp_wheel_ts.header.stamp = ts

            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)

            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.0
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.0
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

        # ---- Compat helpers for different f110_gym / Gym versions ----
    def _compat_reset(self, poses_np):
        """Return (obs, info), set self.done."""
        # 1) Gym>=0.26 스타일 (options=...)
        try:
            obs, info = self.env.reset(options={"poses": poses_np})
            self.done = False
            return obs, info
        except TypeError:
            pass

        # 2) 키워드 전용 시그니처 reset(poses=...)
        try:
            out = self.env.reset(poses=poses_np)
        except TypeError:
            # 3) 혹시 ndarray 형 변환 문제 있으면 np.asarray로 한 번 더
            try:
                out = self.env.reset(poses=np.asarray(poses_np))
            except TypeError:
                # 4) 극단적으로 list로
                out = self.env.reset(poses=np.asarray(poses_np).tolist())

        # 반환 형태 정규화
        if isinstance(out, tuple):
            if len(out) == 2:
                obs, info = out
                self.done = False
                return obs, info
            if len(out) == 4:
                obs, _, done, info = out
                self.done = bool(done)
                return obs, info

        # obs만 반환하는 형태
        self.done = False
        return out, {}


    def _compat_step(self, actions_np):
        """Return obs dict, set self.done, ignore reward/info outside."""
        # Gym>=0.26: 5개 반환
        try:
            obs, reward, terminated, truncated, info = self.env.step(actions_np)
            self.done = bool(terminated) or bool(truncated)
            return obs
        except ValueError:
            # 구버전: 4개 반환
            obs, reward, done, info = self.env.step(actions_np)
            self.done = bool(done)
            return obs

def main(args=None):
    rclpy.init(args=args)
    node = GymBridge()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
