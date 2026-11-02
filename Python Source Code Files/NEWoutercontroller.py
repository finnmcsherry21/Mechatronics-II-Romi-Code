

from pyb import Pin
import time


class OuterController:

    def __init__(self, imu, left_task=0, right_task=0, kp_line=0.0, ki_line=0.0, velocity_setpoint=0.0,y_hat=0.0):

        self.imu = imu
        self.left_task = left_task
        self.right_task = right_task

        self.velocity_setpoint = float(velocity_setpoint)
        self.y_hat = float(y_hat)
        self.ki_line = float(ki_line)
        self.kp_line = float(kp_line)
        
        self.enabled = False
        self.omega_hat = self.imu.yaw_rate()
        self.e_omega = 0.0
        self.delta_omega = 0.0
        self.omega_sp_L = 0.0
        self.omega_sp_R = 0.0
        self.lane_switching = False
        self.state = 1  # 1: line following, 2: lane switching
        self.previous_time = 0
        self.line_integral = 0.0


    def set_base_velocity(self, velocity_setpoint):
        self.velocity_setpoint = float(velocity_setpoint)

        
    def set_gains(self, kp_line, ki_line):
        kp_line = float(kp_line)
        ki_line = float(ki_line)

        if kp_line < 0:
            raise ValueError("Line Kp must be non-negative")

        if ki_line < 0:
            raise ValueError("Line Ki must be non-negative")

        self.kp_line = kp_line
        self.ki_line = ki_line


    def set_line_error(self, y_hat):
        self.y_hat = float(y_hat)


    def enable(self, velocity_setpoint):
        self.set_base_velocity(velocity_setpoint)
        self.previous_time = time.ticks_us()
        self.line_integral = 0.0

        Pin(Pin.cpu.B13, mode=Pin.OUT_PP).high()
        Pin(Pin.cpu.B14, mode=Pin.OUT_PP).high()
        print("Sensor control pins set high")
        self.enabled = True


    def disable(self):
        self.enabled = False
        self.line_integral = 0.0
        self.previous_time = time.ticks_us()

        Pin(Pin.cpu.B13, mode=Pin.OUT_PP).low()
        Pin(Pin.cpu.B14, mode=Pin.OUT_PP).low() 
        print("Sensor control pins set low")

        
    def stop(self):
        self.disable()

        self.left_task.stop()
        self.right_task.stop()

        self.y_hat = 0.0        

        
    def calculate_setpoints(self):
        turn_tightness = 10
        current_time = time.ticks_us()
        dt = time.ticks_diff(current_time, self.previous_time) / 1_000_000
        self.previous_time = current_time

        if dt > 0:
            self.line_integral += self.y_hat * dt

        line_command = self.kp_line * self.y_hat + self.ki_line * self.line_integral
        delta_omega = turn_tightness * line_command

        max_correction = 0.5 * abs(self.velocity_setpoint)
        delta_omega = max(-max_correction, min(max_correction, delta_omega))

        # Prevent the integral term from continuing to grow while steering is
        # already saturated in the same direction as the line error.
        if self.ki_line and abs(delta_omega) >= max_correction:
            if (delta_omega > 0 and self.y_hat > 0) or (delta_omega < 0 and self.y_hat < 0):
                self.line_integral -= self.y_hat * dt
        

        self.omega_sp_L = self.velocity_setpoint - delta_omega
        self.omega_sp_R = self.velocity_setpoint + delta_omega

        return self.omega_sp_L, self.omega_sp_R   


    # def imu_feedback(self):
    #     r = 0.035
    #     self.e_omega = self.velocity_setpoint - self.omega_hat
    #     self.delta_omega = self.kp_line * self.e_omega
        
    #     self.omega_sp_L = (self.velocity_setpoint/r) - self.delta_omega
    #     self.omega_sp_R = (self.velocity_setpoint/r) + self.delta_omega


    #     self.omega_sp_L = max(-25.0, min(25.0, self.omega_sp_L))
    #     self.omega_sp_R = max(-25.0, min(25.0, self.omega_sp_R))


    def run(self):

        while True:
            if self.enabled == True and not self.lane_switching:
                
                omega_sp_L, omega_sp_R = self.calculate_setpoints()
                self.left_task.set_setpoint(omega_sp_L)
                self.right_task.set_setpoint(omega_sp_R)

            if self.enabled == True and self.lane_switching:
                self.left_task.set_setpoint(self.omega_sp_L)
                self.right_task.set_setpoint(self.omega_sp_R)
                
            # else:
            #     break
        
            yield
