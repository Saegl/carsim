from __future__ import annotations

import time
import math
import socket
import struct
import pygame
import sys


SCALE = 25.0

WHITE = pygame.Color(255, 255, 255)
BLACK = pygame.Color(0, 0, 0)

GREY = pygame.Color(100, 100, 100)
DARK_GREY = pygame.Color(30, 30, 30)

RED = pygame.Color(200, 0, 0)
GREEN = pygame.Color(0, 200, 0)
BLUE = pygame.Color(0, 0, 200)


class Inputs:
    def __init__(self):
        self.left = 0
        self.right = 0
        self.throttle = 0
        self.brake = 0
        self.ebrake = 0
        self.gear = -1  # -1 = no change, 0-5 = gear index


def clamp(x, minval, maxval):
    return min(maxval, max(minval, x))


def sign(x):
    if x >= 0.0:
        return 1.0
    else:
        return -1.0


def get_car_corners(car):
    """Get the four corners of a car in world coordinates"""
    corners = [
        pygame.Vector2(car.cg_to_front, car.half_width),
        pygame.Vector2(car.cg_to_front, -car.half_width),
        pygame.Vector2(-car.cg_to_rear, -car.half_width),
        pygame.Vector2(-car.cg_to_rear, car.half_width),
    ]
    return [car.position + c.rotate_rad(car.heading) for c in corners]


def point_in_obb(point, car):
    """Check if a point is inside a car's OBB, return penetration depth and normal"""
    diff = point - car.position
    sn = math.sin(-car.heading)
    cs = math.cos(-car.heading)
    local_x = cs * diff.x - sn * diff.y
    local_y = sn * diff.x + cs * diff.y

    # Check if inside
    if (
        -car.cg_to_rear <= local_x <= car.cg_to_front
        and -car.half_width <= local_y <= car.half_width
    ):
        # Calculate penetration on each axis
        pen_left = local_x + car.cg_to_rear
        pen_right = car.cg_to_front - local_x
        pen_bottom = local_y + car.half_width
        pen_top = car.half_width - local_y

        min_pen = min(pen_left, pen_right, pen_bottom, pen_top)

        # Determine normal based on smallest penetration
        if min_pen == pen_left:
            local_normal = pygame.Vector2(-1, 0)
        elif min_pen == pen_right:
            local_normal = pygame.Vector2(1, 0)
        elif min_pen == pen_bottom:
            local_normal = pygame.Vector2(0, -1)
        else:
            local_normal = pygame.Vector2(0, 1)

        # Transform normal to world space
        sn = math.sin(car.heading)
        cs = math.cos(car.heading)
        world_normal = pygame.Vector2(
            cs * local_normal.x - sn * local_normal.y,
            sn * local_normal.x + cs * local_normal.y,
        )

        return min_pen, world_normal

    return 0, None


def resolve_car_collision(car1, car2, restitution=0.3):
    """Check and resolve collision between two cars."""
    # Check if car1's corners penetrate car2
    corners1 = get_car_corners(car1)
    max_pen = 0
    collision_normal = None
    collision_point = None

    for corner in corners1:
        pen, normal = point_in_obb(corner, car2)
        if pen > max_pen:
            max_pen = pen
            collision_normal = normal
            collision_point = corner

    # Also check car2's corners against car1
    corners2 = get_car_corners(car2)
    for corner in corners2:
        pen, normal = point_in_obb(corner, car1)
        if pen > max_pen:
            max_pen = pen
            collision_normal = -normal if normal else None
            collision_point = corner

    if max_pen > 0 and collision_normal and collision_point:
        # Resolve overlap - push cars apart
        car1.position += collision_normal * (max_pen * 0.5)
        car2.position -= collision_normal * (max_pen * 0.5)

        # Calculate moment arms (from car center to collision point)
        r1 = collision_point - car1.position
        r2 = collision_point - car2.position

        # Relative velocity at collision point (including angular velocity)
        v1_at_point = car1.velocity + pygame.Vector2(
            -car1.yaw_rate * r1.y, car1.yaw_rate * r1.x
        )
        v2_at_point = car2.velocity + pygame.Vector2(
            -car2.yaw_rate * r2.y, car2.yaw_rate * r2.x
        )
        rel_vel = v1_at_point - v2_at_point
        vel_along_normal = rel_vel.dot(collision_normal)

        # Only resolve if moving towards each other
        if vel_along_normal < 0:
            # Cross product in 2D: r x n = r.x * n.y - r.y * n.x
            r1_cross_n = r1.x * collision_normal.y - r1.y * collision_normal.x
            r2_cross_n = r2.x * collision_normal.y - r2.y * collision_normal.x

            # Impulse denominator includes rotational inertia
            denom = (
                (1 / car1.mass)
                + (1 / car2.mass)
                + (r1_cross_n * r1_cross_n) / car1.inertia
                + (r2_cross_n * r2_cross_n) / car2.inertia
            )

            j = -(1 + restitution) * vel_along_normal / denom

            impulse = collision_normal * j
            car1.velocity += impulse / car1.mass
            car2.velocity -= impulse / car2.mass

            # Apply angular impulse
            car1.yaw_rate += r1_cross_n * j / car1.inertia
            car2.yaw_rate -= r2_cross_n * j / car2.inertia


class Ball:
    """
    A ball with physics that can collide with the car
    """

    def __init__(self):
        self.pos = pygame.Vector2()
        self.velocity = pygame.Vector2()
        self.radius = 1.0  # meters
        self.mass = 500.0  # kg
        self.friction = 0.98  # velocity damping per frame
        self.restitution = 0.8  # bounciness (1.0 = perfectly elastic)

    def collide_with_car(self, car):
        """Check and resolve collision with a single car."""
        # Circle vs OBB (oriented bounding box) collision
        # Transform ball position to car's local coordinate system
        diff = self.pos - car.position
        sn = math.sin(-car.heading)
        cs = math.cos(-car.heading)
        local_x = cs * diff.x - sn * diff.y
        local_y = sn * diff.x + cs * diff.y

        # Car rectangle bounds in local space (centered at CG)
        half_length_front = car.cg_to_front
        half_length_rear = car.cg_to_rear
        half_width = car.half_width

        # Find closest point on rectangle to ball center
        closest_x = clamp(local_x, -half_length_rear, half_length_front)
        closest_y = clamp(local_y, -half_width, half_width)

        # Distance from ball center to closest point
        dx = local_x - closest_x
        dy = local_y - closest_y
        dist_sq = dx * dx + dy * dy

        if dist_sq < self.radius * self.radius:
            # Collision detected
            dist = math.sqrt(dist_sq) if dist_sq > 0 else 0.001

            # Normal in local space (from closest point to ball center)
            local_normal_x = dx / dist
            local_normal_y = dy / dist

            # Transform normal back to world space
            sn = math.sin(car.heading)
            cs = math.cos(car.heading)
            normal = pygame.Vector2(
                cs * local_normal_x - sn * local_normal_y,
                sn * local_normal_x + cs * local_normal_y,
            )

            # Resolve overlap
            overlap = self.radius - dist
            self.pos += normal * overlap

            # Relative velocity
            rel_vel = self.velocity - car.velocity

            # Velocity along collision normal
            vel_along_normal = rel_vel.dot(normal)

            # Only resolve if objects are moving towards each other
            if vel_along_normal < 0:
                # Elastic collision with restitution
                j = -(1 + self.restitution) * vel_along_normal
                j /= (1 / self.mass) + (1 / car.mass)

                impulse = normal * j
                self.velocity += impulse / self.mass
                car.velocity -= impulse / car.mass

    def update_physics(self, dt: float, cars: list | None = None):
        """Update ball physics. If cars provided, check collisions with them."""
        if cars:
            for car in cars:
                self.collide_with_car(car)

        # Apply friction/drag
        self.velocity *= self.friction

        # Update position
        self.pos += self.velocity * dt

    def update(self, dt: float, game: Game):
        self.update_physics(dt, game.cars)

    def draw(self, surf: pygame.Surface, game: Game):
        pos_in_camera = game.camera.convert(self.pos)
        pygame.draw.circle(surf, GREEN, pos_in_camera, self.radius * SCALE)


class Camera:
    def __init__(self, scale: float):
        self.pos = pygame.Vector2()
        self.scale = scale

    def convert(self, vec: pygame.Vector2):
        """
        Convert world coordinates to camera coordinates
        """
        return (vec - self.pos) * self.scale


class Grid:
    def __init__(self, tile_size: int):
        self.tile_size = tile_size

    def draw(self, surf: pygame.Surface, game: Game):
        width = surf.get_width()
        height = surf.get_height()

        # Draw vertical lines
        first_x_world = (game.camera.pos.x // self.tile_size) * self.tile_size
        vert_lines_count = width // self.tile_size

        for i in range(vert_lines_count):
            x_in_world = first_x_world + i * self.tile_size
            x_in_camera = (x_in_world - game.camera.pos.x) * game.camera.scale
            pygame.draw.line(surf, GREY, (x_in_camera, 0), (x_in_camera, height))

        # Draw horizontal lines
        first_y_world = (game.camera.pos.y // self.tile_size) * self.tile_size
        horz_lines_count = height // self.tile_size

        for i in range(horz_lines_count):
            y_in_world = first_y_world + i * self.tile_size
            y_in_camera = (y_in_world - game.camera.pos.y) * game.camera.scale
            pygame.draw.line(surf, GREY, (0, y_in_camera), (width, y_in_camera))


class HUD:
    def draw_debug_text(self, surf: pygame.Surface, game: Game, car: Car):
        upd_budget = game.update_time / (1 / game.fps)
        hud_lines = [
            f"fps         = {game.current_fps:.2f}",
            f"update time = {game.update_time:.4f}",
            f"upd budget  = {upd_budget:.1%}",
            "---",
            f"position    = ({car.position.x:.2f}, {car.position.y:.2f})",
            f"accel       = ({car.accel.x:.2f}, {car.accel.y:.2f})",
            f"accel_c     = ({car.accel_c.x:.2f}, {car.accel_c.y:.2f})",
            f"velocity    = ({car.velocity.x:.2f}, {car.velocity.y:.2f})",
            f"velocity_c  = ({car.velocity_c.x:.2f}, {car.velocity_c.y:.2f})",
            f"abs accel   = {car.accel.length():.2f} m/s^2",
            f"speed       = {car.abs_vel * 60 * 60 / 1000:.2f} km/h",
            "---",
            f"yaw_rate    = {car.yaw_rate:.2f} rad/s",
            f"heading     = {car.heading:.2f}",
            f"torque      = {car.engine_torque:.2f} Nm",
            f"rpm         = {car.rpm:.2f}",
            f"HP          = {car.engine_torque * car.rpm / 5252:.2f}",
            "---",
            f"gear        = {car.current_gear_index + 1}",
            f"gear ratio  = {car.gear_ratios[car.current_gear_index]}",
        ]

        x, y = 10, 10
        for line in hud_lines:
            text_surf = game.debug_font.render(line, True, (255, 255, 255))
            surf.blit(text_surf, (x, y))
            y += text_surf.get_height() + 2

    def draw_speedometer(self, surf: pygame.Surface, game: Game, car: Car):
        speed = car.abs_vel * 60 * 60 / 1000

        radius = 120
        padding = 50
        line_len = 100
        line_width = 5
        max_value = 300
        value = speed

        height = surf.get_height()
        circle_center = pygame.Vector2(radius + padding, height - padding - radius)

        start_angle = -0.8
        full_angle_length = 2 * math.pi - 1.6

        pygame.draw.circle(surf, WHITE, circle_center, radius)

        # Draw tick marks
        tick_interval = 50
        for tick_value in range(0, max_value + 1, tick_interval):
            tick_progress = tick_value / max_value
            tick_angle = start_angle - tick_progress * full_angle_length
            direction = pygame.Vector2(math.sin(tick_angle), math.cos(tick_angle))

            tick_outer = circle_center + direction * (radius - 5)
            tick_inner = circle_center + direction * (radius - 20)
            pygame.draw.line(surf, BLACK, tick_outer, tick_inner, 2)

        # Draw needle
        progress = value / max_value
        angle = start_angle - progress * full_angle_length

        line_end = (
            circle_center + pygame.Vector2(math.sin(angle), math.cos(angle)) * line_len
        )

        pygame.draw.line(
            surf,
            RED,
            circle_center,
            line_end,
            line_width,
        )

        text_surf = game.font.render(str(int(speed)), True, BLACK)
        surf.blit(
            text_surf,
            (circle_center.x - text_surf.get_width() / 2, circle_center.y + 50),
        )

    def draw_tachometer(self, surf: pygame.Surface, game: Game, car: Car):
        radius = 120
        padding = 50
        line_len = 100
        line_width = 5
        max_value = 8000
        value = car.rpm

        height = surf.get_height()
        circle_center = pygame.Vector2(
            3 * radius + 2 * padding, height - padding - radius
        )

        start_angle = -0.8
        full_angle_length = 2 * math.pi - 1.6

        pygame.draw.circle(surf, WHITE, circle_center, radius)

        # Draw tick marks
        tick_interval = 1000
        for tick_value in range(0, max_value + 1, tick_interval):
            tick_progress = tick_value / max_value
            tick_angle = start_angle - tick_progress * full_angle_length
            direction = pygame.Vector2(math.sin(tick_angle), math.cos(tick_angle))

            tick_outer = circle_center + direction * (radius - 5)
            tick_inner = circle_center + direction * (radius - 20)
            pygame.draw.line(surf, BLACK, tick_outer, tick_inner, 2)

        # Draw needle
        progress = value / max_value
        angle = start_angle - progress * full_angle_length

        line_end = (
            circle_center + pygame.Vector2(math.sin(angle), math.cos(angle)) * line_len
        )

        pygame.draw.line(
            surf,
            RED,
            circle_center,
            line_end,
            line_width,
        )

        text_surf = game.font.render(str(car.current_gear_index + 1), True, BLACK)
        surf.blit(
            text_surf,
            (circle_center.x - text_surf.get_width() / 2, circle_center.y + 50),
        )

    def draw(self, surf: pygame.Surface, game: Game, car: Car):
        self.draw_debug_text(surf, game, car)
        self.draw_speedometer(surf, game, car)
        self.draw_tachometer(surf, game, car)


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
        self.inertia_scale = 1.0  # multiply by mass for inertia
        self.half_width = 0.8  # center to side of chassis (meters)

        self.cg_to_front = 2.0  # center of gravity to front of chassis
        self.cg_to_rear = 2.0  # center of gravity to rear of chassis
        self.cg_to_front_axle = 1.25
        self.cg_to_rear_axle = 1.25
        self.cg_to_height = 0.55  # center of gravity height

        self.wheel_radius = 0.3  # including tire (also represents height of axle)
        self.wheel_width = 0.2  # for render only
        self.tire_grip = 2.2  # how much grip tires have
        self.lock_grip = 0.7  # % of grip available when wheel is locked

        self.gear_ratios = [4.23, 2.52, 1.66, 1.23, 1.00, 0.83]
        self.current_gear_index = 0

        self.torque_curve = [
            (800, 200),
            (2000, 280),
            (3000, 340),
            (4000, 355),
            (5000, 360),
            (6000, 355),
            (7000, 340),
            (8000, 300),
        ]

        self.diff_ratio = 3.5
        self.transmission_eff = 0.85
        self.min_rpm = 800
        self.max_rpm = 8100
        self.rpm = self.min_rpm
        self.engine_torque = 0.0  # Calculated dynamically from rpm

        self.brake_force = 15000.0  # Newtons
        self.ebrake_force = self.brake_force / 2.5
        self.weight_transfer = 0.2  # how much weight transferred during accel/brake
        self.max_steer = 0.6  # maximum steering angle
        self.corner_stiffness_front = 5.0
        self.corner_stiffness_rear = 5.2
        self.air_resist = 0.3
        self.roll_resist = 8.0

        # set config
        self.inertia = self.mass * self.inertia_scale
        self.wheel_base = self.cg_to_front_axle + self.cg_to_rear_axle
        self.axle_weight_ratio_rear = self.cg_to_rear_axle / self.wheel_base
        self.axle_weight_ratio_front = self.cg_to_front_axle / self.wheel_base

        self._create_surfaces()

        self.max_tire_length = 100_000
        self.last_tire_index = 0
        self.tire_tracks: list[pygame.Vector2 | None] = [None] * self.max_tire_length

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

    def update(
        self,
        dt: float,
        keys: pygame.key.ScancodeWrapper,
        game: Game,
        bindings: dict | None = None,
    ):
        if bindings is None:
            bindings = {
                "throttle": pygame.K_w,
                "brake": pygame.K_s,
                "left": pygame.K_a,
                "right": pygame.K_d,
                "ebrake": pygame.K_SPACE,
            }

        self.inputs = Inputs()
        if keys[bindings["throttle"]]:
            self.inputs.throttle = 1
        if keys[bindings["left"]]:
            self.inputs.left = 1
        if keys[bindings["brake"]]:
            self.inputs.brake = 1
        if keys[bindings["right"]]:
            self.inputs.right = 1
        if keys[bindings["ebrake"]]:
            self.inputs.ebrake = 1

        if keys[pygame.K_1]:
            self.current_gear_index = 0
        if keys[pygame.K_2]:
            self.current_gear_index = 1
        if keys[pygame.K_3]:
            self.current_gear_index = 2
        if keys[pygame.K_4]:
            self.current_gear_index = 3
        if keys[pygame.K_5]:
            self.current_gear_index = 4
        if keys[pygame.K_6]:
            self.current_gear_index = 5

        # TODO: Implement debounce to make this possible
        # if keys[pygame.K_q]:
        #     self.current_gear_index = max(0, self.current_gear_index - 1)
        #
        # if keys[pygame.K_e]:
        #     self.current_gear_index = min(
        #         len(self.gear_ratios) - 1, self.current_gear_index + 1
        #     )

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

    def add_front_tire_tracks(self):
        tire1 = self.position + pygame.Vector2(
            self.cg_to_front_axle,
            -self.half_width / 2,
        ).rotate_rad(self.heading)

        tire2 = self.position + pygame.Vector2(
            self.cg_to_front_axle,
            +self.half_width / 2,
        ).rotate_rad(self.heading)

        self.tire_tracks[self.last_tire_index] = tire1
        self.last_tire_index = (self.last_tire_index + 1) % self.max_tire_length
        self.tire_tracks[self.last_tire_index] = tire2
        self.last_tire_index = (self.last_tire_index + 1) % self.max_tire_length

    def add_rear_tire_tracks(self):
        tire3 = self.position + pygame.Vector2(
            -self.cg_to_rear_axle,
            -self.half_width / 2,
        ).rotate_rad(self.heading)

        tire4 = self.position + pygame.Vector2(
            -self.cg_to_rear_axle,
            +self.half_width / 2,
        ).rotate_rad(self.heading)

        self.tire_tracks[self.last_tire_index] = tire3
        self.last_tire_index = (self.last_tire_index + 1) % self.max_tire_length
        self.tire_tracks[self.last_tire_index] = tire4
        self.last_tire_index = (self.last_tire_index + 1) % self.max_tire_length

    def get_engine_torque(self, rpm):
        for i in range(len(self.torque_curve) - 1):
            r1, t1 = self.torque_curve[i]
            r2, t2 = self.torque_curve[i + 1]
            if r1 <= rpm <= r2:
                # Linear interpolation
                return t1 + (t2 - t1) * ((rpm - r1) / (r2 - r1))

        # Out of range (too high)
        return 0.0

    def update_physics(self, dt):
        sn = math.sin(self.heading)
        cs = math.cos(self.heading)

        self.velocity_c.x = cs * self.velocity.x + sn * self.velocity.y
        self.velocity_c.y = cs * self.velocity.y - sn * self.velocity.x

        weight_transfered = (
            self.weight_transfer * self.accel_c.x * self.cg_to_height / self.wheel_base
        )

        axle_weight_front = (
            self.mass * self.axle_weight_ratio_front * self.gravity - weight_transfered
        )
        axle_weight_rear = (
            self.mass * self.axle_weight_ratio_rear * self.gravity + weight_transfered
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

        self.engine_torque = self.get_engine_torque(self.rpm)
        gear_ratio = self.gear_ratios[self.current_gear_index]

        drive_force = (
            self.engine_torque
            * gear_ratio
            * self.diff_ratio
            * self.transmission_eff
            / self.wheel_radius
        )
        throttle = self.inputs.throttle * drive_force

        traction_force_cx = throttle - brake * sign(self.velocity_c.x)
        traction_force_cy = 0

        drag_force_cx = (
            -self.roll_resist * self.velocity_c.x
            - self.air_resist * self.velocity_c.x * abs(self.velocity_c.x)
        )
        drag_force_cy = (
            -self.roll_resist * self.velocity_c.y
            - self.air_resist * self.velocity_c.y * abs(self.velocity_c.y)
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

        self.rpm = (
            self.velocity_c.length()
            * gear_ratio
            * self.diff_ratio
            * 60
            / (2 * math.pi * self.wheel_radius)
        )
        self.rpm = clamp(self.rpm, self.min_rpm, self.max_rpm)

        wheel_speed = abs(self.velocity_c.x)
        threshold = 0.01
        angle_threshold = 0.5
        slip_ratio = (wheel_speed - self.abs_vel) / max(self.abs_vel, 0.1)

        if abs(slip_ratio) > threshold or abs(slip_angle_rear) > angle_threshold:
            self.add_rear_tire_tracks()

        if abs(slip_angle_front) > angle_threshold:
            self.add_front_tire_tracks()

        self.heading %= 2 * math.pi  # normalize, it accumulates

    def draw_tire_tracks(self, surf: pygame.Surface, game: Game):
        for wp in self.tire_tracks:
            if wp is not None:
                p = game.camera.convert(wp)
                pygame.draw.circle(surf, DARK_GREY, p, 0.18 * SCALE)

    def draw(self, surf: pygame.Surface, game: Game):
        car_camera_pos = game.camera.convert(self.position)

        # Draw car body
        body_rotated = pygame.transform.rotozoom(
            self.body_surface, -math.degrees(self.heading), 1
        )
        body_rect = body_rotated.get_rect()
        body_rect.center = car_camera_pos
        surf.blit(body_rotated, body_rect)

        # Rear wheel 1
        rear_offset = pygame.Vector2(
            -self.cg_to_rear_axle * game.camera.scale, -10
        ).rotate_rad(self.heading)
        rear_pos = car_camera_pos + rear_offset
        rear_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading), 1
        )
        rear_rect = rear_rot.get_rect(center=rear_pos)
        surf.blit(rear_rot, rear_rect)

        # Rear wheel 2
        rear_offset = pygame.Vector2(
            -self.cg_to_rear_axle * game.camera.scale, 10
        ).rotate_rad(self.heading)
        rear_pos = car_camera_pos + rear_offset
        rear_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading), 1
        )
        rear_rect = rear_rot.get_rect(center=rear_pos)
        surf.blit(rear_rot, rear_rect)

        # Front wheel 3
        front_offset = pygame.Vector2(
            self.cg_to_front_axle * game.camera.scale, -10
        ).rotate_rad(self.heading)
        front_pos = car_camera_pos + front_offset
        front_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading + self.steer_angle), 1
        )
        front_rect = front_rot.get_rect(center=front_pos)
        surf.blit(front_rot, front_rect)

        # Front wheel 4
        front_offset = pygame.Vector2(
            self.cg_to_front_axle * game.camera.scale, +10
        ).rotate_rad(self.heading)
        front_pos = car_camera_pos + front_offset
        front_rot = pygame.transform.rotozoom(
            self.wheel_surface, -math.degrees(self.heading + self.steer_angle), 1
        )
        front_rect = front_rot.get_rect(center=front_pos)
        surf.blit(front_rot, front_rect)


class Game:
    def __init__(self, width, height, fps):
        pygame.init()
        self.fps = fps
        self.current_fps = 0.0
        self.screen = pygame.display.set_mode((width, height))
        self.running = False
        self.update_time = 0.0
        self.running_time = 0.0

        self.camera = Camera(SCALE)
        self.hud = HUD()
        self.grid = Grid(tile_size=10)
        self.ball = Ball()

        # Local mode: two cars on same keyboard
        self.cars = [Car(), Car()]
        self.cars[1].position = pygame.Vector2(20, 10)
        self.my_car_index = 0

    @property
    def my_car(self):
        if self.cars:
            return self.cars[self.my_car_index]
        return None

    def update(self, dt: float, game: Game):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        keys = pygame.key.get_pressed()
        if len(self.cars) > 0:
            self.cars[0].update(dt, keys, game)
        if len(self.cars) > 1:
            arrow_bindings = {
                "throttle": pygame.K_UP,
                "brake": pygame.K_DOWN,
                "left": pygame.K_LEFT,
                "right": pygame.K_RIGHT,
                "ebrake": pygame.K_RSHIFT,
            }
            self.cars[1].update(dt, keys, game, arrow_bindings)

        self.ball.update(dt, game)

        # Resolve collisions between all cars
        for i in range(len(self.cars)):
            for j in range(i + 1, len(self.cars)):
                resolve_car_collision(self.cars[i], self.cars[j])

        # Camera follows player car
        center = pygame.Vector2(self.screen.width / 2, self.screen.height / 2)
        if self.my_car:
            target = self.my_car.position - center / self.camera.scale
            self.camera.pos += (target - self.camera.pos) * 5.0 * dt

        if keys[pygame.K_ESCAPE]:
            self.running = False

    def draw(self, surf: pygame.Surface, game: Game):
        surf.fill((50, 50, 50))  # background

        self.grid.draw(surf, game)
        for car in self.cars:
            car.draw_tire_tracks(surf, game)
        for car in self.cars:
            car.draw(surf, game)
        self.ball.draw(surf, game)
        if self.my_car:
            self.hud.draw(surf, game, self.my_car)

        pygame.display.flip()

    def run(self):
        pygame.display.set_caption("Race sim")
        clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", size=24)
        self.debug_font = pygame.font.SysFont("Courier", size=24)

        self.running = True
        while self.running:
            dt = clock.tick(self.fps) / 1000  # delta time in seconds
            self.current_fps = clock.get_fps()
            self.running_time += dt

            update_start = time.time()
            self.update(dt, self)
            # time.sleep(1 / 120)  # Simulate slow update
            self.update_time = time.time() - update_start

            self.draw(self.screen, self)

        pygame.quit()
        sys.exit()


# Network packet formats
# Input: (throttle, brake, left, right, ebrake, gear) = 5 floats + 1 int
INPUT_FORMAT = "!fffffi"
# Car state: (x, y, vx, vy, heading, yaw_rate, steer, steer_angle) = 8 floats
CAR_STATE_FORMAT = "!ffffffff"
# World state: client_id (1 int) + 2 cars (16 floats) + ball (4 floats) = 1 int + 20 floats
WORLD_STATE_FORMAT = "!iffffffffffffffffffff"

MAX_CLIENTS = 2


def pack_inputs(inputs):
    """Pack player inputs to send to server."""
    return struct.pack(
        INPUT_FORMAT,
        float(inputs.throttle),
        float(inputs.brake),
        float(inputs.left),
        float(inputs.right),
        float(inputs.ebrake),
        inputs.gear,
    )


def unpack_inputs(data):
    """Unpack player inputs from network."""
    values = struct.unpack(INPUT_FORMAT, data)
    inputs = Inputs()
    inputs.throttle = int(values[0])
    inputs.brake = int(values[1])
    inputs.left = int(values[2])
    inputs.right = int(values[3])
    inputs.ebrake = int(values[4])
    inputs.gear = values[5]
    return inputs


def pack_world_state(client_id, cars, ball):
    """Pack full world state to send to clients."""
    # Default values for missing cars
    def get_car_state(cars, index):
        if index < len(cars):
            car = cars[index]
            return (car.position.x, car.position.y,
                    car.velocity.x, car.velocity.y,
                    car.heading, car.yaw_rate,
                    car.steer, car.steer_angle)
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    car0 = get_car_state(cars, 0)
    car1 = get_car_state(cars, 1)
    return struct.pack(
        WORLD_STATE_FORMAT,
        client_id,
        *car0,
        *car1,
        ball.pos.x, ball.pos.y,
        ball.velocity.x, ball.velocity.y,
    )


def unpack_world_state(data):
    """Unpack world state, returns (client_id, car_states, ball_state)."""
    values = struct.unpack(WORLD_STATE_FORMAT, data)
    client_id = values[0]
    car0_state = values[1:9]
    car1_state = values[9:17]
    ball_state = values[17:21]
    return client_id, [car0_state, car1_state], ball_state


def lerp(a, b, t):
    """Linear interpolation between a and b by factor t."""
    return a + (b - a) * t


def lerp_angle(a, b, t):
    """Lerp angles handling wraparound."""
    diff = b - a
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return a + diff * t


def apply_car_state(car, state):
    """Apply state directly to car."""
    car.position.x, car.position.y = state[0], state[1]
    car.velocity.x, car.velocity.y = state[2], state[3]
    car.heading, car.yaw_rate = state[4], state[5]
    car.steer, car.steer_angle = state[6], state[7]


def interpolate_car(car, target_state, factor):
    """Interpolate car towards target state."""
    tx, ty, tvx, tvy, theading, tyaw, tsteer, tsteer_angle = target_state
    car.position.x = lerp(car.position.x, tx, factor)
    car.position.y = lerp(car.position.y, ty, factor)
    car.velocity.x = lerp(car.velocity.x, tvx, factor)
    car.velocity.y = lerp(car.velocity.y, tvy, factor)
    car.heading = lerp_angle(car.heading, theading, factor)
    car.yaw_rate = lerp(car.yaw_rate, tyaw, factor)
    car.steer = lerp(car.steer, tsteer, factor)
    car.steer_angle = lerp(car.steer_angle, tsteer_angle, factor)


def apply_ball_state(ball, state):
    """Apply state directly to ball."""
    ball.pos.x, ball.pos.y = state[0], state[1]
    ball.velocity.x, ball.velocity.y = state[2], state[3]


def interpolate_ball(ball, target_state, factor):
    """Interpolate ball towards target state."""
    tx, ty, tvx, tvy = target_state
    ball.pos.x = lerp(ball.pos.x, tx, factor)
    ball.pos.y = lerp(ball.pos.y, ty, factor)
    ball.velocity.x = lerp(ball.velocity.x, tvx, factor)
    ball.velocity.y = lerp(ball.velocity.y, tvy, factor)


class GameServer:
    """
    Headless dedicated server: runs authoritative physics, no display.
    Receives inputs from clients, broadcasts world state.
    """

    # Spawn positions for players
    SPAWN_POSITIONS = [
        pygame.Vector2(10, 10),
        pygame.Vector2(20, 10),
    ]

    def __init__(self, fps, host, port):
        self.fps = fps
        self.running = False

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.setblocking(False)

        self.ball = Ball()

        # Client management
        self.clients = {}  # addr -> client_id
        self.client_inputs = {}  # client_id -> Inputs
        self.cars = []  # Cars created dynamically on connect

        print(f"Server listening on {host}:{port}")
        print("Waiting for clients to connect...")

    def create_car_for_client(self, client_id):
        """Create a new car for a connecting client (headless, no surfaces)."""
        car = Car.__new__(Car)
        # Initialize without calling _create_surfaces
        car.inputs = Inputs()
        car.heading = 0.0
        car.position = self.SPAWN_POSITIONS[client_id % len(self.SPAWN_POSITIONS)].copy()
        car.velocity = pygame.Vector2()
        car.velocity_c = pygame.Vector2()
        car.accel = pygame.Vector2()
        car.accel_c = pygame.Vector2()
        car.abs_vel = 0.0
        car.yaw_rate = 0.0
        car.steer = 0.0
        car.steer_angle = 0.0
        car.smooth_steer = True
        car.safe_steer = True
        car.inertia = 0.0
        car.wheel_base = 0.0
        car.axle_weight_ratio_front = 0.0
        car.axle_weight_ratio_rear = 0.0
        car.gravity = 9.8
        car.mass = 900
        car.inertia_scale = 1.0
        car.half_width = 0.8
        car.cg_to_front = 2.0
        car.cg_to_rear = 2.0
        car.cg_to_front_axle = 1.25
        car.cg_to_rear_axle = 1.25
        car.cg_to_height = 0.55
        car.wheel_radius = 0.3
        car.wheel_width = 0.2
        car.tire_grip = 2.2
        car.lock_grip = 0.7
        car.gear_ratios = [4.23, 2.52, 1.66, 1.23, 1.00, 0.83]
        car.current_gear_index = 0
        car.torque_curve = [
            (800, 200), (2000, 280), (3000, 340), (4000, 355),
            (5000, 360), (6000, 355), (7000, 340), (8000, 300),
        ]
        car.diff_ratio = 3.5
        car.transmission_eff = 0.85
        car.min_rpm = 800
        car.max_rpm = 8100
        car.rpm = car.min_rpm
        car.engine_torque = 0.0
        car.brake_force = 15000.0
        car.ebrake_force = car.brake_force / 2.5
        car.weight_transfer = 0.2
        car.max_steer = 0.6
        car.corner_stiffness_front = 5.0
        car.corner_stiffness_rear = 5.2
        car.air_resist = 0.3
        car.roll_resist = 8.0
        car.inertia = car.mass * car.inertia_scale
        car.wheel_base = car.cg_to_front_axle + car.cg_to_rear_axle
        car.axle_weight_ratio_rear = car.cg_to_rear_axle / car.wheel_base
        car.axle_weight_ratio_front = car.cg_to_front_axle / car.wheel_base
        car.max_tire_length = 100_000
        car.last_tire_index = 0
        car.tire_tracks = [None] * car.max_tire_length
        return car

    def update(self, dt: float):
        # Receive inputs from all clients
        while True:
            try:
                data, addr = self.socket.recvfrom(1024)

                # Assign client ID if new
                if addr not in self.clients:
                    if len(self.clients) < MAX_CLIENTS:
                        client_id = len(self.clients)
                        self.clients[addr] = client_id
                        self.client_inputs[client_id] = Inputs()
                        # Create car for new client
                        car = self.create_car_for_client(client_id)
                        self.cars.append(car)
                        print(f"Client {client_id} connected from {addr}")
                    else:
                        continue  # Ignore, server full

                client_id = self.clients[addr]
                self.client_inputs[client_id] = unpack_inputs(data)

            except BlockingIOError:
                break

        # Update each car based on client inputs
        for client_id, inputs in self.client_inputs.items():
            car = self.cars[client_id]
            car.inputs = inputs

            # Apply gear change
            if inputs.gear >= 0:
                car.current_gear_index = inputs.gear

            # Apply steering
            steer_input = inputs.right - inputs.left
            if car.smooth_steer:
                car.steer = apply_smooth_steer(car.steer, steer_input, dt)
            else:
                car.steer = steer_input
            if car.safe_steer:
                car.steer = apply_safe_steer(car.steer, car.abs_vel)
            car.steer_angle = car.max_steer * car.steer

            car.update_physics(dt)

        # Ball and collisions - server is authoritative
        self.ball.update_physics(dt, self.cars)

        # Resolve collisions between all cars
        for i in range(len(self.cars)):
            for j in range(i + 1, len(self.cars)):
                resolve_car_collision(self.cars[i], self.cars[j])

        # Broadcast world state to all clients
        for addr, client_id in self.clients.items():
            state_data = pack_world_state(client_id, self.cars, self.ball)
            self.socket.sendto(state_data, addr)

    def run(self):
        print("Server running (headless mode)")
        self.running = True
        last_time = time.time()

        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            self.update(dt)

            # Sleep to maintain tick rate
            sleep_time = (1 / self.fps) - dt
            if sleep_time > 0:
                time.sleep(sleep_time)


class GameClient(Game):
    """
    Client: sends inputs to server, receives and renders world state.
    Runs local prediction for smooth visuals.
    """

    def __init__(self, width, height, fps, host, port):
        super().__init__(width, height, fps)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.server_addr = (host, port)

        # Start with empty cars list - will be populated when server responds
        self.cars = []
        self.my_client_id = None
        self.remote_car_states = [None, None]
        self.remote_ball_state = None

        print(f"Client connecting to {host}:{port}")

    @property
    def my_car(self):
        if self.my_client_id is not None and self.my_client_id < len(self.cars):
            return self.cars[self.my_client_id]
        return None

    def update(self, dt: float, game: Game):
        # Receive world state from server
        while True:
            try:
                data, _ = self.socket.recvfrom(1024)
                client_id, car_states, ball_state = unpack_world_state(data)

                if self.my_client_id is None:
                    self.my_client_id = client_id
                    self.my_car_index = client_id
                    print(f"Assigned as Player {client_id + 1}")

                # Ensure we have enough cars for all states
                while len(self.cars) < len(car_states):
                    self.cars.append(Car())

                self.remote_car_states = car_states
                self.remote_ball_state = ball_state

            except BlockingIOError:
                break

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        keys = pygame.key.get_pressed()

        # Collect inputs
        inputs = Inputs()
        if keys[pygame.K_w]:
            inputs.throttle = 1
        if keys[pygame.K_a]:
            inputs.left = 1
        if keys[pygame.K_s]:
            inputs.brake = 1
        if keys[pygame.K_d]:
            inputs.right = 1
        if keys[pygame.K_SPACE]:
            inputs.ebrake = 1

        # Gear selection
        if keys[pygame.K_1]:
            inputs.gear = 0
        elif keys[pygame.K_2]:
            inputs.gear = 1
        elif keys[pygame.K_3]:
            inputs.gear = 2
        elif keys[pygame.K_4]:
            inputs.gear = 3
        elif keys[pygame.K_5]:
            inputs.gear = 4
        elif keys[pygame.K_6]:
            inputs.gear = 5

        # Send inputs to server
        self.socket.sendto(pack_inputs(inputs), self.server_addr)

        # Local prediction: run physics locally
        if self.my_car is not None:
            self.my_car.inputs = inputs

            # Apply gear change locally
            if inputs.gear >= 0:
                self.my_car.current_gear_index = inputs.gear

            steer_input = inputs.right - inputs.left
            if self.my_car.smooth_steer:
                self.my_car.steer = apply_smooth_steer(self.my_car.steer, steer_input, dt)
            else:
                self.my_car.steer = steer_input
            if self.my_car.safe_steer:
                self.my_car.steer = apply_safe_steer(self.my_car.steer, self.my_car.abs_vel)
            self.my_car.steer_angle = self.my_car.max_steer * self.my_car.steer

            self.my_car.update_physics(dt)

        # Update other cars with physics prediction
        for i, car in enumerate(self.cars):
            if i != self.my_client_id:
                car.update_physics(dt)

        # Interpolate all cars towards server state
        for i, state in enumerate(self.remote_car_states):
            if state and i < len(self.cars):
                # Stronger interpolation for remote cars, lighter for own car
                factor = 0.1 if i == self.my_client_id else 0.3
                interpolate_car(self.cars[i], state, factor)

        # Run local ball physics, interpolate towards server
        self.ball.update(dt, game)
        if self.remote_ball_state:
            interpolate_ball(self.ball, self.remote_ball_state, 0.3)

        # Run collisions locally for prediction
        for i in range(len(self.cars)):
            for j in range(i + 1, len(self.cars)):
                resolve_car_collision(self.cars[i], self.cars[j])

        # Camera follows our car
        center = pygame.Vector2(self.screen.width / 2, self.screen.height / 2)
        if self.my_car is not None:
            target = self.my_car.position - center / self.camera.scale
            self.camera.pos += (target - self.camera.pos) * 5.0 * dt

        if keys[pygame.K_ESCAPE]:
            self.running = False


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        mode = sys.argv[1]
        addr = sys.argv[2]

        # Parse host:port format
        if ":" in addr:
            host, port_str = addr.rsplit(":", 1)
            port = int(port_str)
        else:
            print(f"Invalid address format: {addr}")
            print("Expected format: host:port (e.g., 0.0.0.0:4000)")
            sys.exit(1)

        if mode == "server":
            server = GameServer(fps=60, host=host, port=port)
            server.run()
        elif mode == "client":
            game = GameClient(1920, 1200, fps=60, host=host, port=port)
            game.run()
        else:
            print(f"Unknown mode: {mode}")
            print("Usage:")
            print("  python main.py server 0.0.0.0:4000     # Start headless server")
            print("  python main.py client 192.168.1.171:4000  # Connect as player")
            print("  python main.py                         # Local mode")
            sys.exit(1)
    else:
        # Local mode (both cars on same keyboard)
        game = Game(1920, 1200, fps=60)
        game.run()
