import array
import gc
import math
from pyb import Pin, Timer
import pyb
import time


class LineSensorArray:

    def __init__(self):
        self.sensor1 = pyb.ADC(Pin.cpu.C2)
        self.sensor2 = pyb.ADC(Pin.cpu.C3)
        self.sensor3 = pyb.ADC(Pin.cpu.A0)
        self.sensor4 = pyb.ADC(Pin.cpu.A1)
        self.sensor5 = pyb.ADC(Pin.cpu.A4)
        self.sensor6 = pyb.ADC(Pin.cpu.B0)
        self.sensor7 = pyb.ADC(Pin.cpu.C1)
        self.sensor8 = pyb.ADC(Pin.cpu.C0)
        self.sensor9 = pyb.ADC(Pin.cpu.A6)
        self.sensor10 = pyb.ADC(Pin.cpu.C4)
        self.sensor11 = pyb.ADC(Pin.cpu.C5)
        self.sensor12 = pyb.ADC(Pin.cpu.A7)
        self.sensor13 = pyb.ADC(Pin.cpu.B1)

        self.sensors = (
            self.sensor1, self.sensor2, self.sensor3, self.sensor4,
            self.sensor5, self.sensor6, self.sensor7, self.sensor8,
            self.sensor9, self.sensor10, self.sensor11, self.sensor12,
            self.sensor13
        )

        self.sample_count = 5
        self.sensor_buffers = tuple(
            array.array('H', [0] * self.sample_count)
            for sensor in self.sensors
        )
        self.sample_timer = pyb.Timer(4, freq=2500)
        self.left_control_pin = Pin(Pin.cpu.B13, mode=Pin.OUT_PP)
        self.right_control_pin = Pin(Pin.cpu.B14, mode=Pin.OUT_PP)
        self.emitters_enabled = False
        self.disable_emitters()
        self.black_values = None
        self.white_values = None
        self.calibration_black_values = None
        self.calibration_stage = 0
        self.center_sensor = 6.0
        self.line_detect_threshold = 0.10

        data_size = 250
        self.log_time = array.array('f', [0] * data_size)
        self.log_centroid = array.array('f', [0] * data_size)
        self.log_line_error = array.array('f', [0] * data_size)
        self.log_index = 0
        self.logging = False
        self.log_start_time = 0
        gc.collect()

    def read_sensors(self):
        pyb.ADC.read_timed_multi(self.sensors, self.sensor_buffers, self.sample_timer)
        return [sum(buffer) / len(buffer) for buffer in self.sensor_buffers]

    def enable_emitters(self):
        self.left_control_pin.high()
        self.right_control_pin.high()
        self.emitters_enabled = True

    def disable_emitters(self):
        self.left_control_pin.low()
        self.right_control_pin.low()
        self.emitters_enabled = False

    def start_calibration(self):
        print("Place the robot on the black surface and press B.")
        print("Press E to cancel.")
        self.calibration_black_values = None
        self.calibration_stage = 1

    def capture_calibration(self, command):
        if self.calibration_stage == 1:
            if command != "b":
                print("Place the robot on the black surface and press B.")
                return False

            self.calibration_black_values = self.read_sensors()
            self.calibration_stage = 2
            print("Black values:", self.calibration_black_values)
            print("Place the robot on the white surface and press W.")
            return False

        if self.calibration_stage == 2:
            if command != "w":
                print("Place the robot on the white surface and press W.")
                return False

            self.black_values = self.calibration_black_values
            self.white_values = self.read_sensors()
            self.calibration_black_values = None
            self.calibration_stage = 0
            print("White values:", self.white_values)
            return True

        return False

    def start_logging(self):
        self.log_index = 0
        self.logging = True
        self.log_start_time = time.ticks_us()

    def stop_logging(self):
        self.logging = False

    def normalize_values(self, sensor_values):
        if self.black_values is None or self.white_values is None:
            return None

        normalized_values = []

        for index, value in enumerate(sensor_values):
            span = self.black_values[index] - self.white_values[index]

            if span <= 0:
                return None

            normalized_value = (value - self.white_values[index]) / span
            normalized_values.append(max(0.0, min(1.0, normalized_value)))

        return normalized_values

    def find_centroid(self, normalized_values):
        total_weight = sum(normalized_values)

        if total_weight < self.line_detect_threshold:
            return None

        weighted_sum = sum(
            index * value for index, value in enumerate(normalized_values)
        )
        return weighted_sum / total_weight

    def line_detected(self):
        normalized_values = self.normalize_values(self.read_sensors())

        if normalized_values is None:
            return False

        return self.find_centroid(normalized_values) is not None

    def update(self, line_controller):
        if not line_controller.enabled:
            return

        normalized_values = self.normalize_values(self.read_sensors())

        if normalized_values is None:
            line_controller.clear_line_error()
            return

        centroid = self.find_centroid(normalized_values)

        if centroid is None:
            line_controller.clear_line_error()
            return

        line_error = centroid - self.center_sensor
        line_controller.set_line_error(line_error)

        if self.logging and self.log_index < len(self.log_time):
            elapsed_time = time.ticks_diff(time.ticks_us(), self.log_start_time) / 1_000_000
            self.log_time[self.log_index] = elapsed_time
            self.log_centroid[self.log_index] = centroid
            self.log_line_error[self.log_index] = line_error
            self.log_index += 1

        if all(value > 0.25 for value in normalized_values):
            print("Solid black detected. Stopping motors.")
            line_controller.stop()
            self.stop_logging()


class LineController:

    def __init__(self, left_motor_task, right_motor_task,
                 line_sensor_array=None, imu_sensor=None):
        self.left_motor_task = left_motor_task
        self.right_motor_task = right_motor_task
        self.velocity_setpoint = 0.0
        self.line_error = 0.0
        self.line_proportional_gain = 0.0
        self.line_integral_gain = 0.0
        self.line_integral = 0.0
        self.left_velocity_setpoint = 0.0
        self.right_velocity_setpoint = 0.0
        self.lane_switch_active = False
        self.previous_time = 0
        self.enabled = False
        self.line_sensor_array = line_sensor_array
        self.lane_switch_controller = None

        if imu_sensor is not None:
            self.lane_switch_controller = LaneSwitchController(self, imu_sensor)

    def set_velocity_setpoint(self, velocity_setpoint):
        self.velocity_setpoint = max(-25.0, min(25.0, float(velocity_setpoint)))

    def set_gains(self, proportional_gain, integral_gain):
        proportional_gain = float(proportional_gain)
        integral_gain = float(integral_gain)

        if proportional_gain < 0 or integral_gain < 0:
            raise ValueError("Line-controller gains must be non-negative")

        self.line_proportional_gain = proportional_gain
        self.line_integral_gain = integral_gain
        self.line_integral = 0.0

    def set_line_error(self, line_error):
        self.line_error = float(line_error)

    def clear_line_error(self):
        self.line_error = 0.0
        self.line_integral = 0.0

    def set_lane_setpoints(self, left_setpoint, right_setpoint):
        self.left_velocity_setpoint = max(-25.0, min(25.0, float(left_setpoint)))
        self.right_velocity_setpoint = max(-25.0, min(25.0, float(right_setpoint)))

    def clear_lane_setpoints(self):
        self.left_velocity_setpoint = self.velocity_setpoint
        self.right_velocity_setpoint = self.velocity_setpoint

    def enable(self, velocity_setpoint):
        self.set_velocity_setpoint(velocity_setpoint)
        self.previous_time = time.ticks_us()
        self.line_integral = 0.0
        self.line_sensor_array.enable_emitters()
        self.enabled = True

    def disable(self):
        self.enabled = False
        self.line_integral = 0.0
        self.previous_time = time.ticks_us()
        if self.line_sensor_array is not None:
            self.line_sensor_array.disable_emitters()

    def stop(self):
        if self.lane_switch_controller is not None and \
                self.lane_switch_controller.active:
            self.lane_switch_controller.stop("stopped")

        self.disable()
        self.lane_switch_active = False
        self.clear_lane_setpoints()
        self.left_motor_task.stop()
        self.right_motor_task.stop()
        self.line_error = 0.0

    def request_lane_switch(self, turn_direction):
        if self.lane_switch_controller is None:
            print("IMU lane switching is unavailable.")
            return False

        return self.lane_switch_controller.start(turn_direction)

    def update_setpoints(self):
        current_time = time.ticks_us()
        elapsed_time = time.ticks_diff(current_time, self.previous_time) / 1_000_000
        self.previous_time = current_time

        if self.line_integral_gain and elapsed_time > 0:
            self.line_integral += self.line_error * elapsed_time

        correction = self.line_proportional_gain * self.line_error + \
            self.line_integral_gain * self.line_integral
        maximum_correction = 0.5 * abs(self.velocity_setpoint)
        correction = max(-maximum_correction, min(maximum_correction, correction))

        if self.line_integral_gain and abs(correction) >= maximum_correction:
            if (correction > 0 and self.line_error > 0) or \
                    (correction < 0 and self.line_error < 0):
                self.line_integral -= self.line_error * elapsed_time

        self.left_velocity_setpoint = self.velocity_setpoint + correction
        self.right_velocity_setpoint = self.velocity_setpoint - correction

    def run(self):
        while True:
            if self.enabled:
                if self.lane_switch_active:
                    self.lane_switch_controller.update()
                else:
                    self.line_sensor_array.update(self)

                if self.enabled and not self.lane_switch_active:
                    self.update_setpoints()

                if self.enabled:
                    self.left_motor_task.set_setpoint(self.left_velocity_setpoint)
                    self.right_motor_task.set_setpoint(self.right_velocity_setpoint)

            yield


class LaneSwitchController:

    def __init__(self, line_controller, imu_sensor, lane_width=0.20,
                 turn_direction=1, wheel_radius=0.035):
        self.line_controller = line_controller
        self.imu_sensor = imu_sensor
        self.lane_width = float(lane_width)
        self.turn_direction = 1 if turn_direction >= 0 else -1
        self.wheel_radius = float(wheel_radius)
        self.departure_yaw_rate = 0.35
        self.yaw_rate_gain = 2.0
        self.timeout_ms = 8000
        self.DEPARTURE = 1
        self.RETURN = 2
        self.active = False
        self.state = 0
        self.initial_heading = 0.0
        self.relative_heading = 0.0
        self.lateral_position = 0.0
        self.forward_distance = 0.0
        self.previous_time = 0
        self.start_time = 0
        self.next_distance_time = 0

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def start(self, turn_direction):
        if not self.line_controller.enabled:
            print("Start line following before requesting a lane switch.")
            return False

        self.active = True
        self.state = self.DEPARTURE
        self.turn_direction = 1 if turn_direction >= 0 else -1
        self.initial_heading, _, _ = self.imu_sensor.euler()
        self.relative_heading = 0.0
        self.lateral_position = 0.0
        self.forward_distance = 0.0
        self.previous_time = time.ticks_us()
        self.start_time = time.ticks_ms()
        self.next_distance_time = self.start_time
        self.line_controller.lane_switch_active = True
        if self.line_controller.line_sensor_array is not None:
            self.line_controller.line_sensor_array.disable_emitters()
        direction_name = "left" if self.turn_direction > 0 else "right"
        print("Lane switch started to the {}.".format(direction_name))
        return True

    def stop(self, reason="complete"):
        self.active = False
        self.state = 0
        self.line_controller.lane_switch_active = False
        self.line_controller.line_integral = 0.0
        self.line_controller.clear_lane_setpoints()
        if reason == "distance reached" and \
                self.line_controller.line_sensor_array is not None:
            self.line_controller.line_sensor_array.enable_emitters()
        print("Lane switch {}. Lateral estimate: {:.3f} m, forward distance: {:.3f} m".format(
            reason, abs(self.lateral_position), self.forward_distance))

    def update_dead_reckoning(self):
        current_time = time.ticks_us()
        elapsed_time = time.ticks_diff(current_time, self.previous_time) / 1_000_000
        self.previous_time = current_time

        if elapsed_time <= 0 or elapsed_time > 0.1:
            return

        left_velocity = self.line_controller.left_motor_task.wheel_encoder.get_velocity()
        right_velocity = self.line_controller.right_motor_task.wheel_encoder.get_velocity()
        forward_speed = self.wheel_radius * (left_velocity + right_velocity) / 2
        current_heading, _, _ = self.imu_sensor.euler()
        self.relative_heading = self.wrap_angle(current_heading - self.initial_heading)
        print("bingus")
        self.lateral_position += forward_speed * math.sin(self.relative_heading) * elapsed_time
        self.forward_distance += abs(forward_speed * elapsed_time)

    def update_setpoints(self, target_yaw_rate):
        measured_yaw_rate = self.imu_sensor.yaw_rate()
        yaw_rate_error = target_yaw_rate - measured_yaw_rate
        maximum_correction = 0.5 * abs(self.line_controller.velocity_setpoint)
        correction = self.yaw_rate_gain * yaw_rate_error
        correction = max(-maximum_correction, min(maximum_correction, correction))
        velocity = self.line_controller.velocity_setpoint
        self.line_controller.set_lane_setpoints(
            velocity - correction,
            velocity + correction
        )
    def update(self):
        if not self.active:
            return

        self.update_dead_reckoning()

        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.next_distance_time) >= 0:
            self.next_distance_time = time.ticks_add(current_time, 250)
            print("Estimated lateral distance: {:.3f} m, forward distance: {:.3f} m".format(
                abs(self.lateral_position), self.forward_distance))

        if self.state == self.DEPARTURE:
            self.update_setpoints(self.turn_direction * self.departure_yaw_rate)

            if abs(self.lateral_position) >= self.lane_width / 2:
                self.state = self.RETURN
                print("Halfway to target lane. Returning toward target line.")

        elif self.state == self.RETURN:
            self.update_setpoints(-self.turn_direction * self.departure_yaw_rate)

            if abs(self.lateral_position) >= self.lane_width:
                self.stop("distance reached")

        if self.active and \
                time.ticks_diff(time.ticks_ms(), self.start_time) >= self.timeout_ms:
            self.stop("timed out")
            self.line_controller.stop()
