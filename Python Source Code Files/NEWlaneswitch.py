

# Lane Switching file

import time
import math





class LaneSwitch:
    
    def __init__(self, outer_controller, imu):
        self.outer_controller = outer_controller
        self.imu = imu
        self.lane_switching_enabled = False
        self.state = 1
        self.initial_heading = 0.0
        self.d_hat = 0.0
        self.previous_time = 0
        self.relative_heading = 0.0
        self.dt = 0.0
        self.forward_speed = 0.0
        self.d = 0.2  # Lane width in meters
        self.r = 0.035  # Wheel radius in meters
        self.current_heading = 0.0
        self.reversed = False
        self.second_half = False

    def enable_lane_switching(self):
        print("In enable_lane_switching method rn")
        
        self.lane_switching_enabled = True
        self.initial_heading, _, _ = self.imu.euler()
        self.previous_time = time.ticks_us()
        self.d_hat = 0.0
        self.relative_heading = 0.0
        self.second_half = False
        self.outer_controller.lane_switching = True
        self.state = 1
        self.dt = 0.0
        self.current_heading = 0.0
        
        
        
        
    def disable_lane_switching(self):
        self.lane_switching_enabled = False
        self.d_hat = 0.0
        self.second_half = False
        self.outer_controller.lane_switching = False
        self.outer_controller.line_integral = 0.0
        self.state = 1
    
    def run(self):
        
        if self.state == 1:
            self.current_time = time.ticks_us()
            self.dt = time.ticks_diff(self.current_time, self.previous_time) / 1_000_000.0
            self.omega_L = self.outer_controller.left_task.encoder.get_velocity()
            self.omega_R = self.outer_controller.right_task.encoder.get_velocity()
            self.forward_speed = (self.r/2)*(self.omega_L + self.omega_R)        


            self.current_heading, _, _ = self.imu.euler()
            #self.relative_heading = self.current_heading - self.initial_heading
            self.relative_heading = (self.current_heading - self.initial_heading + math.pi) % (2 * math.pi) - math.pi
            self.d_hat = self.d_hat + (self.forward_speed * math.sin(self.relative_heading) * self.dt)
            self.previous_time = self.current_time

        
            if abs(self.d_hat) >= self.d/2:
                self.state = 2
                print("d_hat is halfway")
                self.outer_controller.omega_sp_L, self.outer_controller.omega_sp_R = (self.outer_controller.omega_sp_R,self.outer_controller.omega_sp_L)
                
        if self.state == 2:
            self.current_time = time.ticks_us()
            self.dt = time.ticks_diff(self.current_time, self.previous_time) / 1_000_000.0
            self.omega_L = self.outer_controller.left_task.encoder.get_velocity()
            self.omega_R = self.outer_controller.right_task.encoder.get_velocity()
            self.forward_speed = (self.r/2)*(self.omega_L + self.omega_R)        

            self.current_heading, _, _ = self.imu.euler()
            #self.relative_heading = self.current_heading - self.initial_heading
            self.relative_heading = (self.current_heading - self.initial_heading + math.pi) % (2 * math.pi) - math.pi

            self.d_hat = self.d_hat + (self.forward_speed * math.sin(self.relative_heading) * self.dt)

            
            self.previous_time = self.current_time


            
            if abs(self.d_hat) >= self.d:
                print("Disabling lane switching because d_hat exceeded d")
                self.disable_lane_switching()
                self.state = 1
                self.second_half = False
                self.d_hat = 0.0
                
                
# Gains: 0.25, 0.25, 0.25, 0.25

# Velocity: 7.5
                
                
                

        
        

        
        

    
