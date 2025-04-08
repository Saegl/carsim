from __future__ import annotations

import math
import pygame
import sys


SCALE = 25.0


class Inputs:
    def __init__(self):
        self.left = 0
        self.right = 0
        self.throttle = 0
        self.brake = 0
        self.ebrake = 0


def clamp(x, minval, maxval):
    return min(maxval, max(minval, x))


def sign(x):
    if x >= 0.0:
        return 1.0
    else:
        return -1.0


def apply_smooth_steer(steer, steer_input, dt):
    new_steer = 0
    steer_active = steer_input != 0.0
    if steer_active:
        new_steer = clamp(steer + steer_input * dt * 2.0, -1.0, 1.0)
    else:
        if steer > 0:
            new_steer = max(steer - dt, 0)
        elif steer < 0:
            new_steer = min(steer + dt, 0)

    return new_steer


def apply_safe_steer(steer_input, abs_vel):
    """
    Safe Steering
    Limit the steering angle by the speed of the car.
    Prevents oversteer at expense of more understeer.
    """
    avel = min(abs_vel, 250.0)
    steer = steer_input * (1.0 - (avel / 280.0))
    return steer


class Car:
    def __init__(self):
        self.inputs = Inputs()

        self.heading = 0.0  # angle car is pointed at (radians)
        self.position = pygame.Vector2(10, 10)  # in meters (world coords)
        self.velocity = pygame.Vector2()  # m/s (world coords)
        self.velocity_c = pygame.Vector2()  # (car coords)
        self.accel = pygame.Vector2()  # (world coords)
        self.accel_c = pygame.Vector2()  # (car coords)
        self.abs_vel = 0.0  # absolute velocity aka speed
        self.yaw_rate = 0.0  # angular velocity (radians)
        self.steer = 0.0  # steering input [-1:1]
        self.steer_angle = 0.0  # actual steer angle

        self.smooth_steer = True
        self.safe_steer = True

        # Will be computed from config
        self.inertia = 0.0  # mass
        self.wheel_base = 0.0  # from axle to CG
        self.axle_weight_ratio_front = 0.0  # % car weight on the front axle
        self.axle_weight_ratio_rear = 0.0  # % car weight on the rear axle

        # config
        self.gravity = 9.8  # m/s^2
        self.mass = 900  # kg
        self.intertia_scale = 1.0  # multiply by mass for inertia
        self.half_width = 0.8  # center to side of chassis (meters)
        self.cg_to_front = 2.0  # center of gravity to front of chassis
        self.cg_to_rear = 2.0  # center of gravity to rear of chassis
        self.cg_to_front_axle = 1.25
        self.cg_to_rear_axle = 1.25
        self.cg_to_height = 0.55  # center of gravity height
        self.wheel_radius = 0.3  # including tire (also represents height of axle)
        self.wheel_width = 0.2  # for render only
        self.tire_grip = 2.0  # how much grip tires have
        self.lock_grip = 0.7  # % of grip available when wheel is locked

        self.engine_torque = 250.0
        self.gear_ratio = 4.0
        self.diff_ratio = 3.5
        self.transmission_eff = 0.85

        self.brake_force = 12000.0  # Newtons
        self.ebrake_force = self.brake_force / 2.5
        self.weight_transfer = 0.2  # how much weight transferred during accel/brake
        self.max_steer = 0.6  # maximum steering angle
        self.corner_stiffness_front = 5.0
        self.corner_stiffness_rear = 5.2
        self.air_ressist = 2.5
        self.roll_ressist = 8.0

        # set config
        self.inertia = self.mass * self.intertia_scale
        self.wheel_base = self.cg_to_front_axle + self.cg_to_rear_axle
        self.axle_weight_ratio_rear = self.cg_to_rear_axle / self.wheel_base
        self.axle_weight_ratio_front = self.cg_to_front_axle / self.wheel_base

        self._create_surfaces()

    def _create_surfaces(self):
        body_length = (self.cg_to_front + self.cg_to_rear) * SCALE
        body_width = self.half_width * 2 * SCALE
        self.body_surface = pygame.Surface((body_length, body_width), pygame.SRCALPHA)
        pygame.draw.rect(
            self.body_surface,
            pygame.Color("#1166BB"),
            pygame.Rect(0, 0, body_length, body_width),
        )
        pygame.draw.rect(
            self.body_surface,
            pygame.Color("#222222"),
            pygame.Rect(0, 0, body_length, body_width),
            width=1,
        )

        wheel_w = self.wheel_radius * 2 * SCALE
        wheel_h = self.wheel_width * SCALE
        self.wheel_surface = pygame.Surface((wheel_w, wheel_h), pygame.SRCALPHA)
        pygame.draw.rect(
            self.wheel_surface,
            pygame.Color("#444444"),
            pygame.Rect(0, 0, wheel_w, wheel_h),
        )
        pygame.draw.rect(
            self.wheel_surface,
            pygame.Color("#111111"),
            pygame.Rect(0, 0, wheel_w, wheel_h),
            width=1,
        )

    def update(self, dt: float, keys: pygame.key.ScancodeWrapper):
        self.inputs = Inputs()
        if keys[pygame.K_w]:
            self.inputs.throttle = 1
        if keys[pygame.K_a]:
            self.inputs.left = 1
        if keys[pygame.K_s]:
            self.inputs.brake = 1
        if keys[pygame.K_d]:
            self.inputs.right = 1
        if keys[pygame.K_SPACE]:
            self.inputs.ebrake = 1

        # Steer input smoothing
        steer_input = self.inputs.right - self.inputs.left
        if self.smooth_steer:
            self.steer = apply_smooth_steer(self.steer, steer_input, dt)
        else:
            self.steer = steer_input

        if self.safe_steer:
            self.steer = apply_safe_steer(self.steer, self.abs_vel)

        self.steer_angle = self.max_steer * self.steer

        self.update_physics(dt)

    def update_physics(self, dt):
        sn = math.sin(self.heading)
        cs = math.cos(self.heading)

        self.velocity_c.x = cs * self.velocity.x + sn * self.velocity.y
        self.velocity_c.y = cs * self.velocity.y - sn * self.velocity.x

        axle_weight_front = (
            self.mass * self.axle_weight_ratio_front * self.gravity
            - self.weight_transfer
            * self.accel_c.x
            * self.cg_to_height
            / self.wheel_base
        )
        axle_weight_rear = (
            self.mass * self.axle_weight_ratio_rear * self.gravity
            - self.weight_transfer
            * self.accel_c.x
            * self.cg_to_height
            / self.wheel_base
        )

        yaw_speed_front = self.cg_to_front_axle * self.yaw_rate
        yaw_speed_rear = -self.cg_to_rear_axle * self.yaw_rate

        slip_angle_front = (
            math.atan2(self.velocity_c.y + yaw_speed_front, abs(self.velocity_c.x))
            - sign(self.velocity_c.x) * self.steer_angle
        )
        slip_angle_rear = math.atan2(
            self.velocity_c.y + yaw_speed_rear, abs(self.velocity_c.x)
        )

        tire_grip_front = self.tire_grip
        tire_grip_rear = self.tire_grip * (
            1.0 - self.inputs.ebrake * (1.0 - self.lock_grip)
        )

        friction_force_front_cy = (
            clamp(
                -self.corner_stiffness_front * slip_angle_front,
                -tire_grip_front,
                tire_grip_front,
            )
            * axle_weight_front
        )
        friction_force_rear_cy = (
            clamp(
                -self.corner_stiffness_rear * slip_angle_rear,
                -tire_grip_rear,
                tire_grip_rear,
            )
            * axle_weight_rear
        )

        brake = min(
            self.inputs.brake * self.brake_force
            + self.inputs.ebrake * self.ebrake_force,
            self.brake_force,
        )

        drive_force = (
            self.engine_torque
            * self.gear_ratio
            * self.diff_ratio
            * self.transmission_eff
            / self.wheel_radius
        )
        throttle = self.inputs.throttle * drive_force

        traction_force_cx = throttle - brake * sign(self.velocity_c.x)
        traction_force_cy = 0

        drag_force_cx = (
            -self.roll_ressist * self.velocity_c.x
            - self.air_ressist * self.velocity_c.x * abs(self.velocity_c.x)
        )
        drag_force_cy = (
            -self.roll_ressist * self.velocity_c.y
            - self.air_ressist * self.velocity_c.y * abs(self.velocity_c.y)
        )

        total_force_cx = drag_force_cx + traction_force_cx
        total_force_cy = (
            drag_force_cy
            + traction_force_cy
            + math.cos(self.steer_angle) * friction_force_front_cy
            + friction_force_rear_cy
        )

        self.accel_c.x = total_force_cx / self.mass
        self.accel_c.y = total_force_cy / self.mass

        self.accel.x = cs * self.accel_c.x - sn * self.accel_c.y
        self.accel.y = sn * self.accel_c.x + cs * self.accel_c.y

        self.velocity.x += self.accel.x * dt
        self.velocity.y += self.accel.y * dt

        self.abs_vel = self.velocity.length()

        angular_torque = (
            friction_force_front_cy + traction_force_cy
        ) * self.cg_to_front_axle - friction_force_rear_cy * self.cg_to_rear_axle

        if abs(self.abs_vel) < 0.5 and not throttle:
            self.velocity.x = 0
            self.velocity.y = 0
            self.abs_vel = 0
            angular_torque = 0
            self.yaw_rate = 0

        angular_accel = angular_torque / self.inertia

        self.yaw_rate += angular_accel * dt
        self.heading += self.yaw_rate * dt

        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt

    def draw(self, surf: pygame.Surface, game: Game):
        pos_px = self.position * SCALE

        # Draw car body
        body_rotated = pygame.transform.rotozoom(
            self.body_surface, -math.degrees(self.heading), 1
        )
        body_rect = body_rotated.get_rect()
        body_rect.center = pos_px
        surf.blit(body_rotated, body_rect)

        # Rear wheel
        rear_offset = pygame.Vector2(-self.cg_to_rear_axle * SCALE, 0).rotate_rad(
            self.heading
        )
        rear_pos = pos_px + rear_offset
        rear_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading), 1
        )
        rear_rect = rear_rot.get_rect(center=rear_pos)
        surf.blit(rear_rot, rear_rect)

        # Front wheel
        front_offset = pygame.Vector2(self.cg_to_front_axle * SCALE, 0).rotate_rad(
            self.heading
        )
        front_pos = pos_px + front_offset
        front_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading + self.steer_angle), 1
        )
        front_rect = front_rot.get_rect(center=front_pos)
        surf.blit(front_rot, front_rect)

        hud_lines = [
            f"fps         = {game.current_fps:.2f}",
            f"accel       = ({self.accel.x:.2f}, {self.accel.y:.2f})",
            f"accel_c     = ({self.accel_c.x:.2f}, {self.accel_c.y:.2f})",
            f"velocity    = ({self.velocity.x:.2f}, {self.velocity.y:.2f})",
            f"velocity_c  = ({self.velocity_c.x:.2f}, {self.velocity_c.y:.2f})",
            f"speed       = {self.abs_vel * 60 * 60 / 1000:.2f} km/h",
            f"yaw_rate    = {self.yaw_rate:.2f} rad/s",
            f"heading     = {self.heading:.2f}",
        ]

        x, y = 10, 10
        for line in hud_lines:
            text_surf = game.font.render(line, True, (255, 255, 255))
            surf.blit(text_surf, (x, y))
            y += text_surf.get_height() + 2


class Game:
    def __init__(self, width, height, fps):
        pygame.init()
        self.fps = fps
        self.current_fps = 0.0
        self.screen = pygame.display.set_mode((width, height))
        self.running = False
        self.car = Car()

    def update(self, dt: float):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        keys = pygame.key.get_pressed()
        self.car.update(dt, keys)

        if keys[pygame.K_ESCAPE]:
            self.running = False

    def draw(self, surf: pygame.Surface, game: Game):
        surf.fill((50, 50, 50))  # background
        self.car.draw(surf, game)

        pygame.display.flip()

    def run(self):
        pygame.display.set_caption("Race sim")
        clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Courier", 24)

        self.running = True
        while self.running:
            dt = clock.tick(self.fps) / 1000  # delta time in seconds
            self.current_fps = clock.get_fps()
            self.update(dt)
            self.draw(self.screen, self)

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    game = Game(1920, 1200, fps=60)
    game.run()
