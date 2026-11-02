
from pyb import Pin, Timer, I2C
# import utime
import time
import pyb
import NEWimu_calibration

from NEWmotor import MotorDriver
from NEWencoder import Encoder
import NEWtaskmotor
import NEWtaskuser

import NEWcotask
import array
import NEWtaskline
import NEWline_calibration
import NEWoutercontroller
import gc
from NEWimudriver import BNO055


# ----- Motor setup -----
pwm_tim = Timer(1, freq=20_000)
left_motor = MotorDriver(Pin.cpu.A8, Pin.cpu.B7, Pin.cpu.B6, pwm_tim, 1) # Order: PWM, Direction, Enable, Timer, Channel
right_motor = MotorDriver(Pin.cpu.A9, Pin.cpu.B2, Pin.cpu.A10, pwm_tim, 2)


# ----- Encoder hardware setup -----
left_encoder_timer = Timer(3, prescaler=0, period=65535)
left_encoder_timer.channel(1, pin=Pin.cpu.B4, mode=Timer.ENC_AB) # Channel A
left_encoder_timer.channel(2, pin=Pin.cpu.B5, mode=Timer.ENC_AB) # Channel B

right_encoder_timer = Timer(2, prescaler=0, period=65535)
right_encoder_timer.channel(1, pin=Pin.cpu.A15, mode=Timer.ENC_AB) # Channel A
right_encoder_timer.channel(2, pin=Pin.cpu.B3, mode=Timer.ENC_AB) # Channel B

# IMU setup
i2c = I2C(2, I2C.MASTER, baudrate=100000)
imu = BNO055(i2c)
chip_id = imu.read_chip_id()
print(f"IMU chip ID: {hex(chip_id)}")


# ----- Encoder objects -----

def print_imu_heading():
    while True:
        heading, roll, pitch = imu.euler()
        print("IMU heading: {:.2f} degrees".format(heading))
        yield 0
        
def print_yaw_rate():
    while True:
        yaw_rate = imu.yaw_rate()
        print("IMU yaw rate: {:.2f} rad/s".format(yaw_rate))
        yield 0

def main():
    
    left_encoder = Encoder(left_encoder_timer, direction=-1)
    right_encoder = Encoder(right_encoder_timer, direction=-1)

    # preallocate arrays for data storage with a fixed size of 1000 elements
    size = 250
    #left_effort = array.array('f', [0] * size)
    left_position = array.array('f', [0] * size)
    left_velocity = array.array('f', [0] * size)
    left_time = array.array('f', [0] * size)
    left_voltage = array.array('f', [0] * size)
    left_setpoint = array.array('f', [0] * size)
    #left_kp = array.array('f', [0] * size)
    #left_ki = array.array('f', [0] * size)

    #right_effort = array.array('f', [0] * size)
    right_position = array.array('f', [0] * size)
    right_velocity = array.array('f', [0] * size)
    right_time = array.array('f', [0] * size)
    right_voltage = array.array('f', [0] * size)
    right_setpoint = array.array('f', [0] * size)
    #right_kp = array.array('f', [0] * size)
    #right_ki = array.array('f', [0] * size)
    
    velocity_setpoint = 0
    y_hat = 0
    kp_line = 0
    ki_line = 0
      
    gc.collect()  # Run garbage collection to free up memory before starting tasks 

    # Create TaskMotor instances for left and right motors and taskuser instance for user input
    left_task = NEWtaskmotor.TaskMotor(left_motor, left_encoder, left_time, left_position, left_velocity, left_voltage, left_setpoint)
    right_task = NEWtaskmotor.TaskMotor(right_motor, right_encoder, right_time, right_position, right_velocity, right_voltage, right_setpoint)
    task_line = NEWtaskline.TaskLine(imu)
    outer_controller = NEWoutercontroller.OuterController(imu, left_task, right_task, kp_line, ki_line, velocity_setpoint, y_hat)
    task_user = NEWtaskuser.TaskUser(left_task, right_task, outer_controller, left_time, left_position, left_velocity, left_voltage, left_setpoint, task_line, imu)

    try:
        channels = list(range(1, 14))
        min_span = 50
        calibrateline = NEWline_calibration.load_calibration("NEWline_calibration.json", channels, min_span)

        task_line.black_values = calibrateline["black"]
        task_line.white_values = calibrateline["white"]
         
        print("Loaded line calibration from NEWline_calibration.json.")
        print("Black_values:", task_line.black_values)
        print("White_values:", task_line.white_values)
              

    except OSError:
        print("No saved line calibration found. Press C to calibrate.")

    except ValueError as error:
        print("Saved line calibration is invalid:", error)
        print("Press C to calibrate.")
        
    try:
        imucalibration = NEWimu_calibration.load_calibration("NEWimu_calibration.json")
        imu.operating_mode_configuration()
        imu.calibration_profile_write(imucalibration)
        imu.operating_mode_normal()

        print("Saved IMU calibration profile loaded.")
        NEWimu_calibration.display_calibration_status(imu)

    except OSError:
        print("No saved IMU calibration found. Press I to calibrate.")

    except ValueError as error:
        print("Saved IMU calibration is invalid:", error)
        print("Delete NEWimu_calibration.json and press I to recalibrate.")

    
    
    # Multitasking setup
    NEWcotask.task_list.append(
    NEWcotask.Task(outer_controller.run(task_line),
                name="Line/Outer Control",
                priority=3,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(left_task.RunMethod(),
                name="Left Motor Task",
                priority=2,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(right_task.RunMethod(),
                name="Right Motor Task",
                priority=2,
                period=20,
                profile=True))

    NEWcotask.task_list.append(
    NEWcotask.Task(task_user.run(),
                name="User Input Task",
                priority=0,
                period=1,
                profile=True))
    

    try:
        while True:
         NEWcotask.task_list.pri_sched()


    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Stopping the motors.")
        left_motor.set_effort(0)
        right_motor.set_effort(0)
        left_motor.disable()
        right_motor.disable()
        print(NEWcotask.task_list.profile())
        

  
    
if __name__ == "__main__":
    main()


    
