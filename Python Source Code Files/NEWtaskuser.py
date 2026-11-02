import pyb
import NEWline_calibration
import NEWimu_calibration
import time


ser = pyb.USB_VCP()


class TaskUser:

    MENU = 1
    LEFT_TEST = 2
    RIGHT_TEST = 3
    SETPOINT = 4
    GAINS = 5
    MANUAL = 6
    LINE_CALIBRATION = 7
    IMU_CALIBRATION = 8
    LINE_FOLLOWING = 9
    LANE_DIRECTION = 10
    MOTOR_DATA = 11
    LINE_DATA = 12
    IMU_VIEWER = 13

    def __init__(self, left_motor_task, right_motor_task, line_controller,
                 line_sensor_array, imu_sensor):
        self.state = TaskUser.MENU
        self.left_motor_task = left_motor_task
        self.right_motor_task = right_motor_task
        self.line_controller = line_controller
        self.line_sensor_array = line_sensor_array
        self.imu_sensor = imu_sensor

        self.value = 0.0
        self.k_p = 0.0
        self.k_i = 0.0
        self.kp_line = 0.0
        self.ki_line = 0.0
        self.setpoint_entered = False
        self.gains_entered = False
        self.manual_speed = 7.5
        self.next_imu_status_time = 0
        self.next_imu_viewer_time = 0
        self.number_buffer = []
        self.number_invalid = False
        self.gain_step = 0
        self.output_motor_task = None
        self.print_index = -1

    def print_help(self):
        print("+------------------------------------------------------------------------------+")
        print("| ME 4305 Romi Tuning Interface Help Menu                                      |")
        print("+-----+------------------------------------------------------------------------+")
        print("| h/H | Print help menu                                                        |")
        print("| v/V | Enter velocity setpoint                                                |")
        print("| k/K | Enter velocity and line-controller gains                               |")
        print("| l/L | Run left motor test                                                    |")
        print("| r/R | Run right motor test                                                   |")
        print("| c/C | Calibrate line sensors                                                 |")
        print("| i/I | Calibrate IMU                                                          |")
        print("| t/T | Stream IMU orientation to the 3D PC viewer                            |")
        print("| d/D | Start line following                                                   |")
        print("| m/M | Start manual driving                                                   |")
        print("| e/E | Stop driving and return to menu                                        |")
        print("+-----+------------------------------------------------------------------------+")

    def print_data(self, motor_task):
        if self.print_index == -1:
            print("Time(s), Position(rad), Velocity(rad/s), Input Voltage(V), Setpoint(rad/s)")
            self.print_index = 0

        elif self.print_index < motor_task.sample_index:
            index = self.print_index
            print("{},{},{},{},{}".format(
                motor_task.log_time[index],
                motor_task.log_position[index],
                motor_task.log_velocity[index],
                motor_task.log_voltage[index],
                motor_task.log_setpoint[index]
            ))
            self.print_index += 1

        else:
            print("End of data")
            return True

        return False

    def print_line_data(self):
        if self.print_index == -1:
            print("Time(s), Centroid, Y_hat")
            self.print_index = 0

        elif self.print_index < self.line_sensor_array.log_index:
            index = self.print_index
            print("{},{},{}".format(
                self.line_sensor_array.log_time[index],
                self.line_sensor_array.log_centroid[index],
                self.line_sensor_array.log_line_error[index]
            ))
            self.print_index += 1

        else:
            print("End of line data")
            return True

        return False

    def read_command(self):
        if not ser.any():
            return None

        char_in = ser.read(1).decode()
        if char_in in {"\r", "\n"}:
            return None
        return char_in.lower()

    def start_number_input(self):
        self.number_buffer = []
        self.number_invalid = False

    def read_number(self):
        if not ser.any():
            return None

        char_in = ser.read(1).decode()

        if char_in in "0123456789":
            if self.number_buffer == ["0"] or \
                    self.number_buffer == ["-", "0"]:
                print("Leading zeros are not allowed. Use 0 or 0.xxx.")
                self.number_invalid = True
            else:
                ser.write(char_in)
                self.number_buffer.append(char_in)

        elif char_in == "-" and len(self.number_buffer) == 0:
            ser.write(char_in)
            self.number_buffer.append(char_in)

        elif char_in == ".":
            if len(self.number_buffer) == 0 or self.number_buffer == ["-"]:
                print("Enter a zero before the decimal point.")
                self.number_invalid = True
            elif "." in self.number_buffer:
                print("Only one decimal point is allowed.")
                self.number_invalid = True
            else:
                ser.write(char_in)
                self.number_buffer.append(char_in)

        elif char_in == "\x7f":
            if len(self.number_buffer) > 0:
                ser.write(char_in)
                self.number_buffer.pop()
            self.number_invalid = False

        elif char_in in {"\r", "\n"}:
            ser.write("\r\n")

            if len(self.number_buffer) == 0 and not self.number_invalid:
                return "cancel"

            if self.number_invalid or \
                    self.number_buffer == ["-"] or \
                    self.number_buffer[-1] == ".":
                print("Enter a valid signed number or press Enter to cancel.")
                self.number_buffer = []
                self.number_invalid = False
                return None

            number = float("".join(self.number_buffer))
            self.number_buffer = []
            self.number_invalid = False
            return number

        else:
            print("Use numbers, one leading -, one decimal point, Backspace, or Enter.")
            self.number_invalid = True

        return None

    def ready_for_motion(self):
        if not self.setpoint_entered:
            print("Enter a velocity setpoint with V first.")
            return False

        if not self.gains_entered:
            print("Enter all controller gains with K first.")
            return False

        return True

    def stop_driving(self):
        self.line_controller.stop()
        self.line_sensor_array.stop_logging()

    def start_motor_test(self, motor_task, state):
        if not self.ready_for_motion():
            return

        self.stop_driving()
        motor_task.start(self.value)
        self.state = state

    def start_line_following(self):
        if not self.ready_for_motion():
            return

        if self.line_sensor_array.black_values is None:
            print("Calibrate the line sensors with C before line following.")
            return

        self.line_sensor_array.start_logging()
        self.line_controller.enable(self.value)
        self.left_motor_task.start(self.value, continuous=True)
        self.right_motor_task.start(self.value, continuous=True)
        print("Starting line following. Type S to lane switch or E to stop.")
        self.state = TaskUser.LINE_FOLLOWING

    def start_manual_driving(self):
        if not self.gains_entered:
            print("Enter all controller gains with K before manual driving.")
            return

        self.stop_driving()
        self.left_motor_task.start(0.0, continuous=True)
        self.right_motor_task.start(0.0, continuous=True)
        print("Manual driving: W forward, S reverse, A left, D right, E stop.")
        self.state = TaskUser.MANUAL

    def start_line_sensor_calibration(self):
        self.stop_driving()
        self.line_sensor_array.enable_emitters()
        self.line_sensor_array.start_calibration()
        self.state = TaskUser.LINE_CALIBRATION

    def finish_line_sensor_calibration(self):
        saved = False

        try:
            calibration = {
                "channels": list(range(1, 14)),
                "black": list(self.line_sensor_array.black_values),
                "white": list(self.line_sensor_array.white_values)
            }
            NEWline_calibration.save_calibration(
                "NEWline_calibration.json",
                calibration,
                calibration["channels"],
                50
            )
            print("Saved calibration to NEWline_calibration.json.")
            saved = True

        except (OSError, ValueError) as error:
            print("Calibration was not saved:", error)

        self.line_sensor_array.disable_emitters()
        if saved:
            print("Line sensor calibration complete. User commands enabled.")
        else:
            print("Calibration is active until restart. Press C to save it again.")
        self.state = TaskUser.MENU

    def update_line_sensor_calibration(self):
        command = self.read_command()

        if command == "e":
            self.line_sensor_array.disable_emitters()
            self.line_sensor_array.calibration_stage = 0
            print("Line sensor calibration cancelled.")
            self.state = TaskUser.MENU

        elif command is not None and \
                self.line_sensor_array.capture_calibration(command):
            self.finish_line_sensor_calibration()

    def start_imu_calibration(self):
        self.stop_driving()
        self.imu_sensor.operating_mode_imuplus()
        self.next_imu_status_time = time.ticks_add(time.ticks_ms(), 25)
        print("Starting IMU calibration. Move the robot until all values are 3/3.")
        print("Press W to save after the IMU is calibrated, or E to cancel.")
        self.state = TaskUser.IMU_CALIBRATION

    def start_imu_viewer(self):
        self.stop_driving()
        self.next_imu_viewer_time = time.ticks_ms()
        print("Streaming IMU orientation for the PC 3D viewer. Press E to stop.")
        self.state = TaskUser.IMU_VIEWER

    def update_menu(self):
        command = self.read_command()

        if command is None:
            return

        if command == "h":
            self.print_help()

        elif command == "v":
            print("Enter velocity setpoint in range [-25.0, 25.0] rad/s.")
            self.start_number_input()
            self.state = TaskUser.SETPOINT

        elif command == "k":
            print("Enter velocity proportional gain.")
            self.gains_entered = False
            self.gain_step = 0
            self.start_number_input()
            self.state = TaskUser.GAINS

        elif command == "l":
            self.start_motor_test(self.left_motor_task, TaskUser.LEFT_TEST)

        elif command == "r":
            self.start_motor_test(self.right_motor_task, TaskUser.RIGHT_TEST)

        elif command == "c":
            self.start_line_sensor_calibration()

        elif command == "i":
            self.start_imu_calibration()

        elif command == "t":
            self.start_imu_viewer()

        elif command == "d":
            self.start_line_following()

        elif command == "m":
            self.start_manual_driving()

        elif command == "e":
            self.stop_driving()
            print("Driving stopped.")
            self.print_index = -1
            self.state = TaskUser.LINE_DATA

        else:
            print("Unknown command. Type H for the help menu.")

    def update_setpoint(self):
        number = self.read_number()

        if number == "cancel":
            print("Velocity setpoint was not changed.")
            self.state = TaskUser.MENU

        elif number is not None:
            self.value = max(-25.0, min(25.0, number))
            self.setpoint_entered = True
            print("Velocity setpoint is {} rad/s.".format(self.value))
            self.state = TaskUser.MENU

    def update_gains(self):
        number = self.read_number()

        if number == "cancel":
            print("Gain entry cancelled. Enter K to try again.")
            self.state = TaskUser.MENU
            return

        if number is None:
            return

        if number < 0:
            print("Gains must be non-negative. Try again.")
            return

        if self.gain_step == 0:
            self.k_p = number
            print("Velocity Kp set to {}.".format(self.k_p))
            print("Enter velocity integral gain.")
            self.gain_step = 1

        elif self.gain_step == 1:
            self.k_i = number
            self.left_motor_task.set_gains(self.k_p, self.k_i)
            self.right_motor_task.set_gains(self.k_p, self.k_i)
            print("Velocity Ki set to {}.".format(self.k_i))
            print("Enter line proportional gain.")
            self.gain_step = 2

        elif self.gain_step == 2:
            self.kp_line = number
            print("Line Kp set to {}.".format(self.kp_line))
            print("Enter line integral gain.")
            self.gain_step = 3

        else:
            self.ki_line = number
            self.line_controller.set_gains(self.kp_line, self.ki_line)
            self.gains_entered = True
            print("Line Ki set to {}.".format(self.ki_line))
            print("Controller gains updated.")
            self.state = TaskUser.MENU

    def update_manual_driving(self):
        command = self.read_command()

        if command is None:
            return

        if command == "w":
            self.left_motor_task.set_setpoint(self.manual_speed)
            self.right_motor_task.set_setpoint(self.manual_speed)

        elif command == "s":
            self.left_motor_task.set_setpoint(-self.manual_speed)
            self.right_motor_task.set_setpoint(-self.manual_speed)

        elif command == "a":
            self.left_motor_task.set_setpoint(-self.manual_speed)
            self.right_motor_task.set_setpoint(self.manual_speed)

        elif command == "d":
            self.left_motor_task.set_setpoint(self.manual_speed)
            self.right_motor_task.set_setpoint(-self.manual_speed)

        elif command == "e":
            self.stop_driving()
            print("Manual driving stopped.")
            self.state = TaskUser.MENU

        else:
            print("Use W, A, S, D, or E in manual driving mode.")

    def update_imu_calibration(self):
        command = self.read_command()

        if command == "e":
            print("IMU calibration cancelled. IMUPLUS mode remains active.")
            self.state = TaskUser.MENU
            return

        status = self.imu_sensor.calibration_status()
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03

        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_imu_status_time) >= 0:
            self.next_imu_status_time = time.ticks_add(now, 500)
            print("IMU: G:{}/3 A:{}/3".format(gyro, accel))

        if command == "w":
            if gyro != 3 or accel != 3:
                print("Keep moving the robot until gyro and accelerometer are 3/3.")
                return

            try:
                self.imu_sensor.operating_mode_configuration()
                self.imu_sensor.wait_for_mode()
                NEWimu_calibration.save_calibration(
                    "NEWimu_calibration.json",
                    self.imu_sensor.calibration_profile_read()
                )
                print("Saved calibration to NEWimu_calibration.json.")
                self.state = TaskUser.MENU

            except (OSError, ValueError) as error:
                print("Calibration was not saved:", error)

            self.imu_sensor.operating_mode_imuplus()
            self.imu_sensor.wait_for_mode()

        elif command is not None:
            print("Press W to save after the IMU is 3/3, or E to cancel.")

    def update_imu_viewer(self):
        command = self.read_command()

        if command == "e":
            print("IMU orientation streaming stopped.")
            self.state = TaskUser.MENU
            return

        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_imu_viewer_time) >= 0:
            self.next_imu_viewer_time = time.ticks_add(now, 50)
            heading, roll, pitch = self.imu_sensor.euler()
            print("IMU,{},{},{}".format(heading, roll, pitch))

    def update_line_following(self):
        if not self.line_controller.enabled:
            print("Line following stopped. Returning to the command menu.")
            self.state = TaskUser.MENU
            return

        command = self.read_command()

        if command == "s":
            print("Lane direction: L for left, R for right, E to cancel.")
            self.state = TaskUser.LANE_DIRECTION

        elif command == "e":
            self.stop_driving()
            print("Driving stopped.")
            self.print_index = -1
            self.state = TaskUser.LINE_DATA

        elif command is not None:
            print("Use S to lane switch or E to stop line following.")

    def update_lane_direction(self):
        if not self.line_controller.enabled:
            print("Line following stopped. Returning to the command menu.")
            self.state = TaskUser.MENU
            return

        command = self.read_command()

        if command == "l":
            self.line_controller.request_lane_switch(1)
            self.state = TaskUser.LINE_FOLLOWING

        elif command == "r":
            self.line_controller.request_lane_switch(-1)
            self.state = TaskUser.LINE_FOLLOWING

        elif command == "e":
            print("Lane switch cancelled.")
            self.state = TaskUser.LINE_FOLLOWING

        elif command is not None:
            print("Use L for left, R for right, or E to cancel.")

    def run(self):
        while True:
            if self.state == TaskUser.MENU:
                self.update_menu()

            elif self.state == TaskUser.LEFT_TEST:
                command = self.read_command()
                if command == "e":
                    self.left_motor_task.stop()
                    print("Left motor test stopped.")
                    self.output_motor_task = self.left_motor_task
                    self.print_index = -1
                    self.state = TaskUser.MOTOR_DATA
                elif self.left_motor_task.finished:
                    print("Left motor test complete.")
                    self.output_motor_task = self.left_motor_task
                    self.print_index = -1
                    self.state = TaskUser.MOTOR_DATA

            elif self.state == TaskUser.RIGHT_TEST:
                command = self.read_command()
                if command == "e":
                    self.right_motor_task.stop()
                    print("Right motor test stopped.")
                    self.output_motor_task = self.right_motor_task
                    self.print_index = -1
                    self.state = TaskUser.MOTOR_DATA
                elif self.right_motor_task.finished:
                    print("Right motor test complete.")
                    self.output_motor_task = self.right_motor_task
                    self.print_index = -1
                    self.state = TaskUser.MOTOR_DATA

            elif self.state == TaskUser.MOTOR_DATA:
                if self.print_data(self.output_motor_task):
                    self.state = TaskUser.MENU

            elif self.state == TaskUser.LINE_DATA:
                if self.print_line_data():
                    self.state = TaskUser.MENU

            elif self.state == TaskUser.SETPOINT:
                self.update_setpoint()

            elif self.state == TaskUser.GAINS:
                self.update_gains()

            elif self.state == TaskUser.MANUAL:
                self.update_manual_driving()

            elif self.state == TaskUser.LINE_CALIBRATION:
                self.update_line_sensor_calibration()

            elif self.state == TaskUser.IMU_CALIBRATION:
                self.update_imu_calibration()

            elif self.state == TaskUser.IMU_VIEWER:
                self.update_imu_viewer()

            elif self.state == TaskUser.LINE_FOLLOWING:
                self.update_line_following()

            elif self.state == TaskUser.LANE_DIRECTION:
                self.update_lane_direction()

            else:
                self.stop_driving()
                print("Invalid user-task state. Returning to the command menu.")
                self.state = TaskUser.MENU

            yield self.value
