#!/usr/bin/env python3
# -- coding: utf-8 --
#
# Pygame-based keyboard teleop for F1TENTH (ROS 2 Foxy, Python 3.8)
#   - 속도(v)는 “저속 구간(|v| ≤ low_speed_thresh)”에서만 완만한 가·감속(gentle_accel) 적용
#   - 조향(steering)은 기존 램프를 그대로 사용 → 즉각성 유지
#   - 퍼블리시 토픽 : /drive  (AckermannDriveStamped)

import os
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped


class VelocityRamp:
    """
    current → target 으로 접근할 때 가속도를 제한해주는 램프.
    - 저속 영역(|v| ≤ low_thresh)에서는 gentle_accel을 사용
    - 그 외에는 normal_accel을 사용
    """

    def __init__(
        self,
        normal_accel: float,
        hz: float,
        gentle_accel: float = None,
        low_speed_thresh: float = 0.0,
    ):
        self.normal_accel = normal_accel
        self.gentle_accel = gentle_accel if gentle_accel is not None else normal_accel
        self.low_thresh = low_speed_thresh
        self.dt = 1.0 / hz

    def _select_accel(self, current: float, target: float) -> float:
        if abs(current) <= self.low_thresh and abs(target) <= self.low_thresh:
            return self.gentle_accel
        return self.normal_accel

    def step(self, current: float, target: float) -> float:
        accel = self._select_accel(current, target)
        if current < target:
            return min(current + accel * self.dt, target)
        elif current > target:
            return max(current - accel * self.dt, target)
        return current


class PygameTeleop(Node):
    def __init__(self):
        super().__init__("pygame_key_teleop")

        # ───────────────────── ROS Publisher (/drive) ──────────────────────
        self._pub = self.create_publisher(
            AckermannDriveStamped, "drive", qos_profile_system_default
        )

        # ─────────────────────── Parameters ───────────────────────────────
        self._hz = self.declare_parameter("hz", 30.0).value

        # 속도 / 조향 제한
        self._max_forward = self.declare_parameter("forward_rate", 2.0).value
        self._max_backward = self.declare_parameter("backward_rate", 2.0).value
        self._max_steer = self.declare_parameter("rotation_rate", 0.36).value

        # 램프 가속도 (normal)
        self._lin_accel = self.declare_parameter("linear_acceleration", 4.0).value
        self._ang_accel = self.declare_parameter("angular_acceleration", 1.0).value

        # 저속 구간 전용 램프
        self._gentle_accel = self.declare_parameter(
            "gentle_linear_accel", 0.2
        ).value  # m/s²
        self._low_speed_th = self.declare_parameter(
            "low_speed_thresh", 0.2
        ).value  # m/s

        # ─────────────────────── State vars ───────────────────────────────
        self._current_speed = 0.0
        self._current_steer = 0.0
        self._target_speed = 0.0
        self._target_steer = 0.0

        # ─────────────────────── Ramps ────────────────────────────────────
        self._speed_ramp = VelocityRamp(
            normal_accel=self._lin_accel,
            gentle_accel=self._gentle_accel,
            low_speed_thresh=self._low_speed_th,
            hz=self._hz,
        )
        self._steer_ramp = VelocityRamp(
            normal_accel=self._ang_accel, hz=self._hz
        )  # gentle = normal

    # ---------------------------------------------------------------------

    def publish_cmd(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "key_teleop"
        msg.drive.speed = self._current_speed
        msg.drive.steering_angle = self._current_steer
        self._pub.publish(msg)


# ══════════════════════════════════════════════════════════════════════════
def main():
    # 헤드리스 환경에서도 실행 가능하도록 더미 드라이버 설정
    if not os.environ.get("DISPLAY"):
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    rclpy.init()
    node = PygameTeleop()

    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("Ackermann Teleop")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    hz = node._hz
    running = True

    while running and rclpy.ok():
        # ─── 이벤트 처리 ───
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # 전·후진 키
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            node._target_speed = node._max_forward
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            node._target_speed = -node._max_backward
        else:
            node._target_speed = 0.0

        # 좌·우 조향 키
        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            node._target_steer = node._max_steer
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            node._target_steer = -node._max_steer
        else:
            node._target_steer = 0.0

        # 램프 적용
        node._current_speed = node._speed_ramp.step(
            node._current_speed, node._target_speed
        )
        node._current_steer = node._steer_ramp.step(
            node._current_steer, node._target_steer
        )

        node.publish_cmd()

        # ─── HUD ───
        screen.fill((30, 30, 30))
        txt1 = font.render(f"Speed: {node._current_speed:.2f} m/s", True, (255, 255, 255))
        txt2 = font.render(f"Steer: {node._current_steer:.2f} rad", True, (255, 255, 255))
        screen.blit(txt1, (20, 60))
        screen.blit(txt2, (20, 100))
        pygame.display.flip()

        clock.tick(hz)

    # 종료
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
