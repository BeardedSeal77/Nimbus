"""
Generic PID Controllers for Drone Navigation
Can be copied to nimbus-robotics for real drone use
"""

import math


def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))


class PIDController:
    """Generic single-axis PID controller"""
    def __init__(self, kp, ki, kd, output_min=-float('inf'), output_max=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_max = 1.0

    def update(self, error, dt):
        """Update PID with current error and time delta"""
        p_term = self.kp * error

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral

        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p_term + i_term + d_term
        return clamp(output, self.output_min, self.output_max)

    def reset(self):
        """Reset controller state"""
        self.prev_error = 0.0
        self.integral = 0.0


class AltitudeController:
    """PD controller for altitude with cubic error scaling"""
    def __init__(self, k_p, k_d, offset=0.0):
        self.k_p = k_p
        self.k_d = k_d
        self.offset = offset
        self.prev_altitude = 0.0

    def update(self, current_altitude, target_altitude, dt):
        """
        Calculate vertical thrust adjustment
        Returns: vertical_input to add to base thrust
        """
        altitude_error = target_altitude - current_altitude + self.offset

        altitude_rate = 0.0
        if dt > 0:
            altitude_rate = (current_altitude - self.prev_altitude) / dt
        self.prev_altitude = current_altitude

        clamped_error = clamp(altitude_error, -1.0, 1.0)

        vertical_input = (self.k_p * pow(clamped_error, 3.0) - self.k_d * altitude_rate)

        return vertical_input


class NavigationController:
    """
    Controls drone navigation using delta XY from AI
    Strategy: Yaw to face target, then pitch forward to move
    """
    def __init__(self, heading_kp, heading_kd, distance_kp, distance_kd,
                 heading_threshold_deg=15.0, distance_threshold_m=1.0,
                 max_yaw=1.3, max_pitch=6.0):

        self.heading_pid = PIDController(heading_kp, 0.0, heading_kd, -max_yaw, max_yaw)
        self.distance_pid = PIDController(distance_kp, 0.0, distance_kd, -max_pitch, max_pitch)

        self.heading_threshold = math.radians(heading_threshold_deg)
        self.distance_threshold = distance_threshold_m
        self.max_yaw = max_yaw
        self.max_pitch = max_pitch

        self.heading_locked = False
        self.at_target = False

    def update(self, delta_x, delta_y, dt):
        """
        Calculate control outputs from delta XY

        Args:
            delta_x: Forward distance to target (meters, positive = in front)
            delta_y: Lateral distance to target (meters, positive = right)
            dt: Time delta (seconds)

        Returns:
            dict with:
                'yaw_disturbance': float
                'pitch_disturbance': float
                'roll_disturbance': float (for lateral movement)
                'heading_locked': bool
                'at_target': bool
        """

        target_angle = math.atan2(delta_y, delta_x)

        distance = math.sqrt(delta_x**2 + delta_y**2)

        self.at_target = distance < self.distance_threshold

        self.heading_locked = abs(target_angle) < self.heading_threshold

        yaw_disturbance = 0.0
        if not self.at_target and not self.heading_locked:
            yaw_disturbance = self.heading_pid.update(target_angle, dt)
        elif self.heading_locked:
            self.heading_pid.reset()

        pitch_disturbance = 0.0
        if not self.at_target and self.heading_locked:
            raw_pitch = self.distance_pid.update(distance, dt)

            decel_distance = 5.0
            if distance < decel_distance:
                speed_factor = distance / decel_distance
                speed_factor = max(0.2, speed_factor)
                raw_pitch = raw_pitch * speed_factor

            pitch_disturbance = -raw_pitch

        roll_disturbance = 0.0

        return {
            'yaw_disturbance': yaw_disturbance,
            'pitch_disturbance': pitch_disturbance,
            'roll_disturbance': roll_disturbance,
            'heading_locked': self.heading_locked,
            'at_target': self.at_target,
            'distance': distance,
            'heading_error_deg': math.degrees(target_angle)
        }

    def reset(self):
        """Reset controller state"""
        self.heading_pid.reset()
        self.distance_pid.reset()
        self.heading_locked = False
        self.at_target = False
