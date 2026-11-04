from pyb import Pin, Timer
from math import pi
import time


class MotorDriver:

    def __init__(self, pwm_pin, direction_pin, sleep_pin, timer, channel,
                 direction=1):
        self.direction_pin = Pin(direction_pin, mode=Pin.OUT_PP)
        self.sleep_pin = Pin(sleep_pin, mode=Pin.OUT_PP)
        self.pwm_channel = timer.channel(channel, pin=pwm_pin, mode=Timer.PWM, pulse_width_percent=0)
        self.direction = direction
        self.disable()

    def enable(self):
        self.sleep_pin.high()

    def disable(self):
        self.sleep_pin.low()

    def set_effort(self, effort):
        effort = self.direction * effort
        effort = max(-100, min(100, effort))

        if effort < 0:
            self.direction_pin.high()
            effort = -effort
        else:
            self.direction_pin.low()

        self.pwm_channel.pulse_width_percent(effort)


class WheelEncoder:

    def __init__(self, timer, direction=1):
        self.timer = timer
        self.direction = direction
        self.position_counts = 0
        self.previous_count = self.timer.counter()
        self.count_change = 0
        self.elapsed_time = 0.0
        self.speed_counts = 0.0
        self.previous_time = time.ticks_us()

    def update(self):
        current_count = self.timer.counter()
        self.count_change = current_count - self.previous_count

        if self.count_change < -32768:
            self.count_change += 65536
        elif self.count_change > 32768:
            self.count_change -= 65536

        current_time = time.ticks_us()
        self.elapsed_time = time.ticks_diff(current_time, self.previous_time) / 1_000_000

        if self.elapsed_time > 0:
            self.speed_counts = self.count_change / self.elapsed_time
        else:
            self.speed_counts = 0.0

        self.position_counts += self.count_change
        self.previous_count = current_count
        self.previous_time = current_time

    def get_position(self):
        return self.direction * self.position_counts * (2 * pi / (12 * 120))

    def get_velocity(self):
        return self.direction * self.speed_counts * (2 * pi / (12 * 120))

    def zero(self):
        self.position_counts = 0
        self.count_change = 0
        self.speed_counts = 0.0
        self.previous_count = self.timer.counter()
        self.previous_time = time.ticks_us()


class VelocityController:

    def __init__(self, proportional_gain=0.0, integral_gain=0.0,
                 minimum_voltage=-7.2, maximum_voltage=7.2):
        self.minimum_voltage = float(minimum_voltage)
        self.maximum_voltage = float(maximum_voltage)

        if self.minimum_voltage >= self.maximum_voltage:
            raise ValueError("minimum_voltage must be less than maximum_voltage")

        self.proportional_gain = 0.0
        self.integral_gain = 0.0
        self.integral_error = 0.0
        self.set_gains(proportional_gain, integral_gain)

    def set_gains(self, proportional_gain, integral_gain):
        proportional_gain = float(proportional_gain)
        integral_gain = float(integral_gain)

        if proportional_gain < 0 or integral_gain < 0:
            raise ValueError("Controller gains must be non-negative")

        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain
        self.integral_error = 0.0

    def reset(self):
        self.integral_error = 0.0

    def update(self, setpoint, measurement, elapsed_time):
        error = float(setpoint) - float(measurement)
        elapsed_time = float(elapsed_time)
        proportional_voltage = self.proportional_gain * error

        candidate_error = self.integral_error + error * elapsed_time
        candidate_voltage = proportional_voltage + self.integral_gain * candidate_error

        if candidate_voltage > self.maximum_voltage:
            allow_integration = error < 0
        elif candidate_voltage < self.minimum_voltage:
            allow_integration = error > 0
        else:
            allow_integration = True

        if allow_integration:
            self.integral_error = candidate_error

        requested_voltage = proportional_voltage + self.integral_gain * self.integral_error
        return max(self.minimum_voltage, min(self.maximum_voltage, requested_voltage))
    
    """
    def discrete_update(self, setpoint, measurement):
    
        self.error = float(setpoint) - float(measurement)
        self.elapsed_time = float(elapsed_time)
        
        self.proportional = self.proportional + self.proportional_gain * self.error
        self.integral = self.integral + self.integral_gain * self.error * self.elapsed_time
        self.derivative = self.derivative + self.derivative_gain * (self.error - self.previous_error) / self.elapsed_time
        
        self.previous_error = self.error
        
        self.output = self.proportional + self.integral + self.derivative
        self.output = max(self.minimum_voltage, min(self.maximum_voltage, self.output))
        
        return self.output
        
    
    """
    


class MotorControlTask:

    def __init__(self, motor_driver, wheel_encoder, log_time, log_position, log_velocity, log_voltage, log_setpoint):
        self.motor_driver = motor_driver
        self.wheel_encoder = wheel_encoder
        self.log_time = log_time
        self.log_position = log_position
        self.log_velocity = log_velocity
        self.log_voltage = log_voltage
        self.log_setpoint = log_setpoint

        self.input_voltage = 0.0
        self.running = False
        self.finished = False
        self.sample_index = 0
        self.test_duration = 5.0
        self.continuous = False
        self.velocity_setpoint = 0.0
        self.velocity_controller = VelocityController()

    def set_gains(self, proportional_gain, integral_gain):
        self.velocity_controller.set_gains(proportional_gain, integral_gain)

    def start(self, velocity_setpoint, continuous=False):
        self.velocity_controller.reset()
        self.wheel_encoder.zero()
        self.start_time = time.ticks_us()
        self.set_setpoint(velocity_setpoint)
        self.input_voltage = 0.0
        self.sample_index = 0
        self.running = True
        self.finished = False
        self.continuous = continuous
        self.motor_driver.set_effort(0)
        self.motor_driver.enable()

    def set_setpoint(self, velocity_setpoint):
        self.velocity_setpoint = max(-25.0, min(25.0, float(velocity_setpoint)))

    def stop(self):
        self.velocity_controller.reset()
        self.motor_driver.set_effort(0)
        self.motor_driver.disable()
        self.running = False
        self.finished = True
        self.continuous = False

    def run(self):
        while True:
            if not self.running:
                yield
                continue

            self.wheel_encoder.update()
            position = self.wheel_encoder.get_position()
            velocity = self.wheel_encoder.get_velocity()
            elapsed_time = time.ticks_diff(time.ticks_us(), self.start_time) / 1_000_000

            self.input_voltage = self.velocity_controller.update(
                self.velocity_setpoint,
                velocity,
                self.wheel_encoder.elapsed_time
            )
            
            """
            self.input_voltage = self.velocity_controller.discrete_update(
                self.velocity_setpoint,
                velocity
                )
            """
            

            effort = 100 * self.input_voltage / 7.2
            self.motor_driver.set_effort(effort)

            if self.sample_index < len(self.log_time):
                self.log_time[self.sample_index] = elapsed_time
                self.log_position[self.sample_index] = position
                self.log_velocity[self.sample_index] = velocity
                self.log_voltage[self.sample_index] = self.input_voltage
                self.log_setpoint[self.sample_index] = self.velocity_setpoint
                self.sample_index += 1

            if not self.continuous and \
                    (elapsed_time >= self.test_duration or self.sample_index >= len(self.log_time)):
                self.stop()

            yield

