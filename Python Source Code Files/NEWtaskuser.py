
# Task user file for the motor control system
import pyb
from pyb import Pin, Timer
import NEWline_calibration
import NEWimu_calibration
import NEWlaneswitch
import time



# import from github: https://github.com/adafruit/Adafruit_BNO055.git



ser = pyb.USB_VCP()  # Create a USB virtual comm port object


class TaskUser:

    def __init__(self, left_task, right_task, outer_controller,
                 data_time, data_position, data_velocity,
                 data_input_voltage, data_setpoint, task_line, imu):
         
        
        self.state = 1
        self.command = None
        self.left_task = left_task
        self.right_task = right_task
        self.outer_controller = outer_controller
        self.task_line = task_line
        self.imu = imu
        self.lane_switch = NEWlaneswitch.LaneSwitch(outer_controller, imu)
        
        self.data_time = data_time
        #self.data_effort = data_effort
        self.data_position = data_position
        self.data_velocity = data_velocity
        self.data_input_voltage = data_input_voltage
        self.data_setpoint = data_setpoint
        #self.data_kp = data_kp
        #self.data_ki = data_ki
        self.value = 0.0
        self.k_p = 0.0
        self.k_i = 0.0
        self.kp_line = 0.0
        self.ki_line = 0.0
        self.setpoint_entered = False
        self.closed_loop_enabled = True
        self.start_calibration = False
        self.manual_speed = 7.5
        self.last_imu_calibration = None
        self.lane_switch.lane_switching_enabled = False
        self.m_pattern_step = 0
        self.m_pattern_start_time = 0
        self.next_imu_status_time = 0
        
        
        
        print("+------------------------------------------------------------------------------+")
        print("| ME 4305 Romi Tuning Interface Help Menu                                      |")
        print("+-----+------------------------------------------------------------------------+")
        print("| h/H | Print help menu                                                        |")
        print("| l/L | Trigger step response sequence on left motor and print results         |")
        print("| t/T | Toggle open/closed loop for both motors                                |")
        print("| v/V | Enter the velocity setpoint for the next PI test (rad/s)               |")
        print("| i/I | Calibrate the IMU sensors                                              |")
        print("| m/M | Enter manual driving mode with W, A, S, D controls                     |")
        print("| r/R | Trigger step response sequence on right motor and print results        |")
        print("| k/K | Enter the proportional and integral gains for closed-loop control      |")
        print("| d/D | Start line following                                                   |")
        print("| c/C | Calibrate the line sensors                                             |")
        print("| e/E | Exit program                                                           |")
        print("+-----+------------------------------------------------------------------------+")
 
        
    def print_data(self, motor_task):
        print("Time(s), Position(rad), Velocity(rad/s), Input Voltage(V), Setpoint(rad/s)")

        for index in range(motor_task.i):
            print("{},{},{},{},{}".format(
                motor_task.data_time[index],
                motor_task.data_position[index],
                motor_task.data_velocity[index],
                motor_task.data_input_voltage[index],
                motor_task.data_setpoint[index],
                #motor_task.data_kp[index],
                #motor_task.data_ki[index]
            ))
            
        print("End of data")
 
    def start_m_pattern(self):
        """
        Drive an approximate letter M using timed velocity-controlled motions.

        The robot starts at the lower-left side of the M and should initially
        face toward the top of the letter.
        """

        self.outer_controller.stop()
        self.task_line.stop_data_collection()

        # Start both wheel controllers in continuous velocity-control mode.
        self.left_task.start(0.0, continuous=True)
        self.right_task.start(0.0, continuous=True)

        self.m_pattern_step = 0
        self.m_pattern_start_time = time.ticks_ms()

        print("Starting M pattern.")
        print("Keep the robot clear of obstacles.")

        # State 9 runs the non-blocking M-pattern state machine.
        self.state = 9     
     
               
    def print_line_data(self):
        print("Time(s), Centroid, Y_hat")

        for index in range(self.task_line.line_data_i):
            print("{},{},{}".format(
                self.task_line.line_data_time[index],
                self.task_line.line_data_centroid[index],
                self.task_line.line_data_y_hat[index]
            ))

        print("End of line data")         


    def set_manual_velocity(self, left_velocity, right_velocity):
        self.left_task.set_setpoint(left_velocity)
        self.right_task.set_setpoint(right_velocity)


    def run(self):
        while True:
            if self.state == 1:
                # Check if at least one character is pending
                if ser.any():
                    # Read the character when ready and decode into a string
                    char_in = ser.read(1).decode()
                    
                    # Process each character appropriately after a state transition
                    if char_in in {"H", "h"}:
                        print("+------------------------------------------------------------------------------+")
                        print("| ME 4305 Romi Tuning Interface Help Menu                                      |")
                        print("+-----+------------------------------------------------------------------------+")
                        print("| h/H | Print help menu                                                        |")
                        print("| l/L | Trigger step response sequence on left motor and print results         |")
                        #print("| c/C | Toggle open/closed loop for both motors                               |")
                        print("| v/V | Enter the velocity setpoint for the next PI test (rad/s)               |")
                        print("| k/K | Enter the proportional and integral gains for closed-loop control      |")
                        print("| r/R | Trigger step response sequence on right motor and print results        |")
                        print("| m/M | Enter manual driving mode with W, A, S, D controls                     |")
                        print("| e/E | Stop driving and return to the command menu                            |")
                        print("+-----+------------------------------------------------------------------------+")
                        self.state = 1                    
                        
                    elif char_in in {"L", "l"}:
                        if not self.setpoint_entered:
                            print("Please enter a velocity setpoint for the next PI test first.")
                            self.state = 1
                            continue
                        self.left_task.start(self.value)
                        self.outer_controller.disable()
                        self.state = 2 
                        
                    
                        
                    elif char_in in {"I", "i"}:
                        self.outer_controller.stop()
                        self.task_line.stop_data_collection()
                        self.imu.operating_mode_configuration()
                        
                        print("Starting IMU calibration.")
                        self.last_imu_calibration = None
                        self.next_imu_status_time = time.ticks_ms()
                        self.state = 7
                        
                    elif char_in in {"D", "d"}:
                        print("D was pressed")
                        if not self.setpoint_entered:
                            print("Please enter a velocity setpoint first.")
                            self.state = 1
                            continue

                        if self.task_line.black_values is None:
                            print("Calibrate the line sensors with C before line following.")
                            self.state = 1
                            continue

                        self.task_line.start_data_collection()

                        self.left_task.start(self.value, continuous=True)
                        self.right_task.start(self.value, continuous=True)
                        
                        print("THIS IS THE VALUE:", self.value)
                        self.outer_controller.enable(self.value)
                        
                        print("Starting line following.")
                        print("Type 's' to lane switch")
                        self.state = 8                     
 
                        
                                               
                    elif char_in in {"C", "c"}:
                        
                        self.outer_controller.stop()
                        self.task_line.stop_data_collection()
                        Pin(Pin.cpu.B13, mode=Pin.OUT_PP).high()
                        Pin(Pin.cpu.B14, mode=Pin.OUT_PP).high()
                        print("Sensor control pins set to high")
                        black_values, white_values = self.task_line.calibrate_sensors()

                        calibration = {
                            "channels": list(range(1, 14)),
                            "black": list(black_values),
                            "white": list(white_values)
                        }

                        try:
                            NEWline_calibration.save_calibration(
                                "NEWline_calibration.json",
                                calibration,
                                calibration["channels"],
                                10
                            )

                            print("Saved calibration to NEWline_calibration.json.")

                        except (OSError, ValueError) as error:
                            print("Calibration was not saved:", error)

                        while ser.any():
                            ser.read(1)

                        Pin(Pin.cpu.B13, mode=Pin.OUT_PP).low()
                        Pin(Pin.cpu.B14, mode=Pin.OUT_PP).low()
                        self.state = 1
                        print("Line sensor calibration complete. User commands enabled.")
                     
                    elif char_in in {"R", "r"}:
                        if not self.setpoint_entered:
                            print("Please enter a velocity setpoint for the next PI test first.")
                            self.state = 1
                            continue
                        self.right_task.start(self.value)
                        self.outer_controller.disable()
                        self.state = 3
                    
                    elif char_in in {"X", "x"}:
                        self.start_m_pattern()
                    
                    elif char_in in {"M", "m"}:
                        self.outer_controller.stop()
                        self.task_line.stop_data_collection()
                        #self.controller.stop()

                        self.left_task.start(0.0, continuous=True)
                        self.right_task.start(0.0, continuous=True)

                        print("Manual driving enabled.")
                        print("W: forward, S: reverse, A: left, D: right, E: stop.")

                        self.state = 6   
                        
                    elif char_in in {"E", "e"}:
                        self.outer_controller.stop()
                        self.task_line.stop_data_collection()

                        print("Driving stopped.")
                        self.print_line_data()

                        self.state = 1
                    
                    elif char_in in {"V", "v"}:
                        print("Enter velocity setpoint for next PI test in range [-25.0, 25.0] (rad/s):")
                        
                            
                        self.state = 4
                        char_buf: list = []
                        digits: set = set(map(str, range(10)))
                        term: set = {"\r", "\n"}
                        done = False
                        char_in = None

                    elif char_in in {"K", "k"}:
                        print("Please enter the proportional gain for the velocity controller.")
                        # Saturate gains to non-negative values and max of 3
                        # if self.k_p > 1:
                        #     self.k_p = 1
                        #     print("Proportional gain cannot exceed 1. Setting to 1.")
                            
                        # if self.k_i > 1:
                        #     self.k_i = 1
                        #     print("Integral gain cannot exceed 1. Setting to 1.")
                        self.state = 5
                        char_buf: list = []
                        digits: set = set(map(str, range(10)))
                        term: set = {"\r", "\n"}
                        done = False
                        char_in = None
                        iteration = 0

                    else:
                        pass
                        
    
            if self.state == 2:
                # Run left state
                
                if self.state == 2 and self.left_task.finished:
                    print("Left motor test complete.")
                    self.print_data(self.left_task)
                    self.state = 1
                          
            if self.state == 3:
                # Run right state
                
                if self.state == 3 and self.right_task.finished:
                    print("Right motor test complete.")
                    self.print_data(self.right_task)
                    self.state = 1
                    
            if self.state == 4:
                
                if ser.any():
                    char_in = ser.read(1).decode()
                    
                    
                    if char_in in digits:
                        ser.write(char_in)
                        char_buf.append(char_in)
                    
                    elif char_in == "-" and len(char_buf) == 0:
                        ser.write(char_in)
                        char_buf.append(char_in)
                    
                    elif char_in == "." and "." not in char_buf:
                        ser.write(char_in)
                        char_buf.append(char_in)
                    
                    elif char_in == "\x7f" and len(char_buf) > 0:
                        ser.write(char_in)
                        char_buf.pop()
                    
                    elif char_in in term:
                                
                        if len(char_buf) == 0:
                            ser.write("\r\n")
                            ser.write("Value not changed\r\n")
                            char_buf = []
                            done = True
                            
                        elif char_buf != ["-"] and char_buf != ["."] and char_buf != ["-","."]:
                            ser.write("\r\n")
                            self.value = float("".join(char_buf))
                            if self.value > 25:
                                self.value = 25
                                print("Velocity setpoint cannot exceed 25.0 rad/s. Setting to 25.0 rad/s.")
                            elif self.value < -25:
                                self.value = -25
                                print("Velocity setpoint cannot be less than -25.0 rad/s. Setting to -25.0 rad/s.")
    
                            ser.write(f"Value set to {self.value}\r\n")
                            self.setpoint_entered = True
                            char_buf = []
                            done = True
                            self.state = 1
                            
                    else:
                        pass
         
            if self.state == 5:
                if ser.any():
                    char_in = ser.read(1).decode()

                    if char_in in digits:
                        ser.write(char_in)
                        char_buf.append(char_in)

                    elif char_in == "-" and len(char_buf) == 0:
                        ser.write(char_in)
                        char_buf.append(char_in)

                    elif char_in == "." and "." not in char_buf:
                        ser.write(char_in)
                        char_buf.append(char_in)

                    elif char_in == "\x7f" and len(char_buf) > 0:
                        ser.write(char_in)
                        char_buf.pop()

                    elif char_in in term:
                        if char_buf not in ([], ["-"], ["."], ["-", "."]):
                            entered_gain = float("".join(char_buf))
                            ser.write("\r\n")

                            if iteration == 0:
                                if entered_gain < 0:
                                    print("Proportional gain must be non-negative.")
                                else:
                                    self.k_p = entered_gain
                                    print("Kp set to {}".format(self.k_p))
                                    print("Please enter the integral gain for the velocity controller.")
                                    iteration = 1

                            else:
                                if iteration == 1:
                                    if entered_gain < 0:
                                        print("Integral gain must be non-negative.")
                                    else:
                                        self.k_i = entered_gain
                                        print("Ki set to {}".format(self.k_i))

                                        # Send identical gains to both wheel controllers.
                                        self.left_task.set_gains(self.k_p, self.k_i)
                                        self.right_task.set_gains(self.k_p, self.k_i)

                                        print("Please enter the proportional gain for the line follower.")
                                        iteration = 2
                                        
                                else:
                                    if iteration == 2:
                                        if entered_gain < 0:
                                            print("Proportional gain must be non-negative.")
                                        else:
                                            self.kp_line = entered_gain
                                            print("Kp set to {}".format(self.kp_line))
                                            print("Please enter the integral gain for the line follower.")
                                            iteration = 3
                                            
                                    else:
                                        if iteration == 3:
                                            if entered_gain < 0:
                                                print("Integral gain must be non-negative.")
                                            else:
                                                self.ki_line = entered_gain
                                                print("Ki set to {}".format(self.ki_line))
        
                                                # Send identical gains to both wheel controllers.
                                                self.outer_controller.set_gains(self.kp_line, self.ki_line)
        
                                                print("Controller gains updated.")
                                                self.state = 1

                            char_buf = []
          
            if self.state == 6:
                if ser.any():
                    char_in = ser.read(1).decode()

                    if char_in in {"W", "w"}:
                        self.set_manual_velocity(
                            self.manual_speed,
                            self.manual_speed
                        )

                    elif char_in in {"S", "s"}:
                        self.set_manual_velocity(
                            -self.manual_speed,
                            -self.manual_speed
                        )

                    elif char_in in {"A", "a"}:
                        self.set_manual_velocity(
                            self.manual_speed,
                            -self.manual_speed
                        )

                    elif char_in in {"D", "d"}:
                        self.set_manual_velocity(
                            -self.manual_speed,
                            self.manual_speed
                        )

                    elif char_in in {"E", "e"}:
                        self.outer_controller.stop()

                        print("Manual driving stopped.")
                        self.state = 1
                        
            if self.state == 7:
                status = self.imu.calibration_status()
                system = (status >> 6) & 0x03
                gyro = (status >> 4) & 0x03
                accel = (status >> 2) & 0x03
                mag = status & 0x03

                now = time.ticks_ms()
                if time.ticks_diff(now, self.next_imu_status_time) >= 0:
                    self.next_imu_status_time = time.ticks_add(now, 500)
                    print("Current IMU calibration status: SYS:{}/3 G:{}/3 A:{}/3 M:{}/3".format(
                        system, gyro, accel, mag))
                

                if system == 3 and gyro == 3 and accel == 3 and mag == 3:
                    print("IMU calibration complete.")
                    
                    NEWimu_calibration.save_calibration(
                        "NEWimu_calibration.json",
                        self.imu.calibration_profile_read(),
                    )
                    print("Saving calibration to NEWimu_calibration.json.")
                    self.state = 1
                    
            if self.state == 8:
                if self.lane_switch.lane_switching_enabled:
                    self.lane_switch.run()

                if not self.outer_controller.enabled and not self.lane_switch.lane_switching_enabled:
                    print("Line following stopped. Returning to the command menu.")
                    self.state = 1
                 
                if ser.any():
                    
                    char_in_s = ser.read(1).decode()
                    if char_in_s in {"S", "s"}:
                        self.lane_switch.enable_lane_switching()
                        
                                           
                    
                    if char_in_s in {"e", "E"}:
                        print("e was pressed in lane switching state.")
                        self.outer_controller.stop()
                        self.task_line.stop_data_collection()
                        self.lane_switch.disable_lane_switching()
                        self.lane_switch.lane_switching_enabled = False
                        print("Driving stopped.")
                        self.print_line_data()

                        self.state = 1
                    
                
            if self.state == 9:
                # Each tuple contains:
                # (left-wheel velocity, right-wheel velocity, duration in milliseconds)
                #
                # Positive velocity drives forward.
                # Opposite wheel velocities rotate the robot in place.
                #
                # Tune the durations for your robot, floor, battery, and wheel speed.
                scale = 1.5
                m_pattern = (
                    # Draw the left vertical stroke upward.
                    (self.manual_speed, self.manual_speed, scale * 1000),

                    # Rotate clockwise approximately 135 degrees.
                    (self.manual_speed, -self.manual_speed, scale * 500),

                    # Drive diagonally down toward the center of the M.
                    (self.manual_speed, self.manual_speed, scale * 700),

                    # Rotate counterclockwise approximately 90 degrees.
                    (-self.manual_speed, self.manual_speed, scale * 400),

                    # Drive diagonally upward toward the upper-right corner.
                    (self.manual_speed, self.manual_speed, scale * 700),

                    # Rotate clockwise approximately 90 degrees.
                    (self.manual_speed, -self.manual_speed, scale * 400),

                    # Draw the right vertical stroke downward.
                    (self.manual_speed, self.manual_speed, scale * 1000),
                )

                if self.m_pattern_step < len(m_pattern):
                    left_velocity, right_velocity, duration = (
                        m_pattern[self.m_pattern_step]
                    )

                    self.set_manual_velocity(left_velocity, right_velocity)

                    elapsed = time.ticks_diff(
                        time.ticks_ms(),
                        self.m_pattern_start_time
                    )

                    if elapsed >= duration:
                        self.m_pattern_step += 1
                        self.m_pattern_start_time = time.ticks_ms()

                else:
                    self.set_manual_velocity(0.0, 0.0)
                    self.left_task.stop()
                    self.right_task.stop()

                    print("M pattern complete.")
                    self.state = 1                        
                    
                 
                        
            yield self.value
        
        
        
