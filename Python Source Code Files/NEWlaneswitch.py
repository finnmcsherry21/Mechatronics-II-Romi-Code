"""Dead-reckoning lane-change controller for the Romi differential drive."""

import math
import time


class LaneSwitch:
    """Move one lane sideways while returning to the original heading.

    Lateral position is estimated from both encoder velocities and the BNO055
    heading. The line follower is temporarily paused, but it resumes as soon
    as the maneuver finishes or times out.
    """

    def __init__(self, outer_controller, imu, lane_width=0.20,
                 turn_direction=1, wheel_radius=0.035):
        self.outer_controller = outer_controller
        self.imu = imu
        self.lane_width = float(lane_width)
        self.turn_direction = 1 if turn_direction >= 0 else -1
        self.wheel_radius = float(wheel_radius)

        # Conservative starting values; tune only after a wheels-off-ground
        # direction and encoder-sign check.
        self.heading_limit = 0.45          # rad (about 26 degrees)
        self.lateral_heading_gain = 5.0    # rad / m
        self.heading_gain = 8.0            # wheel-rad/s per rad of error
        self.lateral_tolerance = 0.015     # m
        self.heading_tolerance = 0.08      # rad
        self.timeout_ms = 8000

        self.lane_switching_enabled = False
        self.initial_heading = 0.0
        self.relative_heading = 0.0
        self.d_hat = 0.0
        self.forward_speed = 0.0
        self.previous_time = 0
        self.start_time = 0

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def enable_lane_switching(self):
        if not self.outer_controller.enabled:
            print("Start line following before requesting a lane switch.")
            return False

        self.lane_switching_enabled = True
        self.initial_heading, _, _ = self.imu.euler()
        self.relative_heading = 0.0
        self.d_hat = 0.0
        self.forward_speed = 0.0
        self.previous_time = time.ticks_us()
        self.start_time = time.ticks_ms()
        self.outer_controller.lane_switching = True
        print("Lane switch started.")
        return True

    def disable_lane_switching(self, reason="complete"):
        self.lane_switching_enabled = False
        self.outer_controller.lane_switching = False
        self.outer_controller.line_integral = 0.0
        self.outer_controller.clear_lane_setpoints()
        print("Lane switch {}. Lateral estimate: {:.3f} m".format(
            reason, self.d_hat))

    def _update_dead_reckoning(self):
        current_time = time.ticks_us()
        dt = time.ticks_diff(current_time, self.previous_time) / 1_000_000
        self.previous_time = current_time

        # Ignore a bad scheduler pause rather than integrating a large false
        # motion estimate after a serial disconnect or debugger pause.
        if dt <= 0 or dt > 0.1:
            return

        omega_left = self.outer_controller.left_task.encoder.get_velocity()
        omega_right = self.outer_controller.right_task.encoder.get_velocity()
        self.forward_speed = self.wheel_radius * (omega_left + omega_right) / 2

        current_heading, _, _ = self.imu.euler()
        self.relative_heading = self._wrap_angle(
            current_heading - self.initial_heading)
        self.d_hat += self.forward_speed * math.sin(self.relative_heading) * dt

    def _command_toward_target(self):
        target_lateral = self.turn_direction * self.lane_width
        lateral_error = target_lateral - self.d_hat
        target_heading = self.lateral_heading_gain * lateral_error
        target_heading = max(-self.heading_limit,
                             min(self.heading_limit, target_heading))

        heading_error = self._wrap_angle(target_heading - self.relative_heading)
        max_correction = 0.5 * abs(self.outer_controller.velocity_setpoint)
        correction = self.heading_gain * heading_error
        correction = max(-max_correction, min(max_correction, correction))

        base = self.outer_controller.velocity_setpoint
        self.outer_controller.set_lane_setpoints(base - correction,
                                                  base + correction)
        return lateral_error

    def run(self):
        """Advance the lane-change estimator and update wheel setpoints."""
        if not self.lane_switching_enabled:
            return

        self._update_dead_reckoning()
        lateral_error = self._command_toward_target()

        if abs(lateral_error) <= self.lateral_tolerance and \
                abs(self.relative_heading) <= self.heading_tolerance:
            self.disable_lane_switching()
            return

        if time.ticks_diff(time.ticks_ms(), self.start_time) >= self.timeout_ms:
            self.disable_lane_switching("timed out; returning to line follower")
