

import time
from NEWclosedloop import ClosedLoop

class TaskMotor:

    def __init__(self, motor, encoder, data_time, data_position, data_velocity, data_input_voltage, data_setpoint):

            self.motor = motor
            self.encoder = encoder

            self.data_time = data_time
            #self.data_effort = data_effort
            self.data_position = data_position
            self.data_velocity = data_velocity
            self.data_input_voltage = data_input_voltage
            self.data_setpoint = data_setpoint
            #self.data_kp = data_kp
            #self.data_ki = data_ki

            self.effort = 0
            self.input_voltage = 0.0
            self.running = False
            self.finished = False
            self.state = 1
            self.i = 0
            self.duration = 5
            self.continuous = False
            self.setpoint = 0.0
            self.controller = ClosedLoop(k_p=0.0, k_i=0.0)
 
                   
    def set_gains(self, k_p, k_i):
        self.controller.set_gains(k_p, k_i)


    def start(self, velocity_setpoint, continuous=False):
        self.controller.reset()
        self.encoder.zero()
        self.start_time = time.ticks_us()
        self.setpoint = float(velocity_setpoint)
        self.effort = 0.0
        self.input_voltage = 0.0
        self.i = 0
        self.state = 1
        self.running = True
        self.finished = False
        self.continuous = continuous

        self.motor.set_effort(0)
        self.motor.enable()


    def set_setpoint(self, velocity_setpoint):
        self.setpoint = float(velocity_setpoint)


    def stop(self):
        self.controller.reset()
        self.motor.set_effort(0)
        self.motor.disable()

        self.running = False
        self.finished = True
        self.continuous = False
        self.state = 1


    def RunMethod(self):
        while True:
            if not self.running:
                yield
                continue
            self.encoder.update()
            self.position = self.encoder.get_position()
            self.velocity = self.encoder.get_velocity()
            self.Time = time.ticks_diff(time.ticks_us(), self.start_time) / 1_000_000

            self.input_voltage = self.controller.update(
                self.setpoint,
                self.velocity,
                self.encoder.dt
            )

            # Convert commanded voltage to percent PWM effort. 7.2 V is the
            # nominal battery/motor voltage for the six-cell Romi pack.
            self.effort = 100 * self.input_voltage / 7.2
            self.motor.set_effort(self.effort)

            if self.i < len(self.data_time):
                self.data_time[self.i] = self.Time
                self.data_position[self.i] = self.position
                self.data_velocity[self.i] = self.velocity
                self.data_input_voltage[self.i] = self.input_voltage
                self.data_setpoint[self.i] = self.setpoint
                self.i += 1

            if not self.continuous and \
                    (self.Time >= self.duration or self.i >= len(self.data_time)):
                self.motor.set_effort(0)
                self.motor.disable()
                self.running = False
                self.finished = True
                self.state = 1

            yield
