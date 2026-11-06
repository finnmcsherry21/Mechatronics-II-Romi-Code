
"""
Push changes to GitHub from the project folder:

git add .
git commit -m "Describe your changes"
git push origin main


Run the test code in the project folder:


"""

from pyb import Pin, Timer, I2C
import NEWimu_calibration
import NEWgains

import NEWtaskuser
import NEWcotask
import array
import NEWline_calibration
import gc
from NEWimudriver import BNO055
from NEWdrive import MotorDriver, WheelEncoder, MotorControlTask
from NEWlinecontrol import LineSensorArray, LineController
from NEWfinaltrack import FinalTrack


# ----- Motor setup -----
pwm_timer = Timer(1, freq=20_000)
left_motor_driver = MotorDriver(Pin.cpu.A8, Pin.cpu.B7, Pin.cpu.B6, pwm_timer, 1, 1)
right_motor_driver = MotorDriver(Pin.cpu.A9, Pin.cpu.B2, Pin.cpu.A10, pwm_timer, 2, 1)


# ----- Encoder hardware setup -----
left_encoder_timer = Timer(3, prescaler=0, period=65535)
left_encoder_timer.channel(1, pin=Pin.cpu.B4, mode=Timer.ENC_AB)
left_encoder_timer.channel(2, pin=Pin.cpu.B5, mode=Timer.ENC_AB)

right_encoder_timer = Timer(2, prescaler=0, period=65535)
right_encoder_timer.channel(1, pin=Pin.cpu.A15, mode=Timer.ENC_AB)
right_encoder_timer.channel(2, pin=Pin.cpu.B3, mode=Timer.ENC_AB)

# IMU setup
imu_i2c = I2C(2, I2C.MASTER, baudrate=100000)
imu_sensor = BNO055(imu_i2c)
chip_id = imu_sensor.read_chip_id()
if chip_id != 0xA0:
    print("Warning: BNO055 chip ID is incorrect. Check IMU wiring before driving.")


def main():

    left_wheel_encoder = WheelEncoder(left_encoder_timer, direction=-1)
    right_wheel_encoder = WheelEncoder(right_encoder_timer, direction=-1)

    # Preallocate arrays for five seconds of 20 ms motor data.
    size = 250
    left_log_position = array.array('f', [0] * size)
    left_log_velocity = array.array('f', [0] * size)
    left_log_time = array.array('f', [0] * size)
    left_log_voltage = array.array('f', [0] * size)
    left_log_setpoint = array.array('f', [0] * size)

    right_log_position = array.array('f', [0] * size)
    right_log_velocity = array.array('f', [0] * size)
    right_log_time = array.array('f', [0] * size)
    right_log_voltage = array.array('f', [0] * size)
    right_log_setpoint = array.array('f', [0] * size)
      
    gc.collect()  # Run garbage collection to free up memory before starting tasks 

    # Create motor, line, outer-controller, and user-interface objects.
    left_motor_task = MotorControlTask(right_motor_driver, right_wheel_encoder, left_log_time, left_log_position, left_log_velocity, left_log_voltage, left_log_setpoint)
    right_motor_task = MotorControlTask(left_motor_driver, left_wheel_encoder, right_log_time, right_log_position, right_log_velocity, right_log_voltage, right_log_setpoint)
    line_sensor_array = LineSensorArray()
    line_controller = LineController(left_motor_task, right_motor_task, line_sensor_array, imu_sensor)
    user_interface = NEWtaskuser.TaskUser(left_motor_task, right_motor_task, line_controller, line_sensor_array, imu_sensor)
    final_track = FinalTrack()


    try:
        channels = list(range(1, 14))
        min_span = 50
        line_calibration_data = NEWline_calibration.load_calibration("NEWline_calibration.json", channels, min_span)

        line_sensor_array.black_values = line_calibration_data["black"]
        line_sensor_array.white_values = line_calibration_data["white"]
         
        print("Loaded NEWline_calibration.json.")
        print("Black values:", line_sensor_array.black_values)
        print("White values:", line_sensor_array.white_values)
              

    except OSError:
        print("No saved line calibration found. Press C to calibrate.")

    except ValueError as error:
        print("Saved line calibration is invalid:", error)
        print("Press C to calibrate.")
        
    imu_sensor.operating_mode_configuration()
    imu_sensor.wait_for_mode()
    imu_sensor.configure_degree_units()

    try:
        imu_calibration_data = NEWimu_calibration.load_calibration("NEWimu_calibration.json")
        imu_sensor.calibration_profile_write(imu_calibration_data)
        print("Loaded NEWimu_calibration.json.")
        print("IMU profile:", imu_calibration_data)

    except OSError:
        print("No saved IMU calibration found. Press I to calibrate.")

    except ValueError as error:
        print("Saved IMU calibration is invalid:", error)
        print("Press I to recalibrate the IMU.")

    imu_sensor.operating_mode_imuplus()
    imu_sensor.wait_for_mode()

    try:
        optimal_gains = NEWgains.load_gains("NEWoptimal_gains.json")
        user_interface.apply_gains(optimal_gains)
        print("Loaded NEWoptimal_gains.json.")

    except OSError:
        print("No saved gains found. Press G to enter and save gains.")

    except ValueError as error:
        print("Saved gains are invalid:", error)
        print("Press G to enter and save gains.")

    user_interface.print_help()
    
    # Multitasking setup
    NEWcotask.task_list.append(
    NEWcotask.Task(line_controller.run(),
                name="Line/Outer Control",
                priority=3,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(left_motor_task.run(),
                name="Left Motor Task",
                priority=2,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(right_motor_task.run(),
                name="Right Motor Task",
                priority=2,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(user_interface.run(),
                name="User Input Task",
                priority=0,
                period=1,
                profile=True))
    
    NEWcotask.task_list.append(
    NEWcotask.Task(final_track.run(),
                name="Final Track Task",
                priority=1,
                period=20,
                profile=True))
    

    try:
        while True:
            NEWcotask.task_list.pri_sched()


    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Stopping the motors.")

    finally:
        left_motor_driver.set_effort(0)
        right_motor_driver.set_effort(0)
        left_motor_driver.disable()
        right_motor_driver.disable()
        line_sensor_array.disable_emitters()
        print(NEWcotask.task_list.profile())
        

  
    
if __name__ == "__main__":
    main()


    
