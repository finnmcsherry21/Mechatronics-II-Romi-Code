# Romi Code Reference

This document describes the active project files, runtime classes, variables,
tasks, and state flows. Variables listed below are module globals, saved
configuration, or object attributes that retain a value between method calls.
Short-lived locals are described with the method that uses them. Test-only fake
objects are listed by purpose rather than duplicating every fake attribute.

## Files

Run by pressing 

| File | Purpose |
| --- | --- |
| `Python Source Code Files/main.py` | MicroPython boot file; imports and runs `NEWmain.main()`. |
| `Python Source Code Files/NEWmain.py` | Creates hardware objects, loads calibration files, creates tasks, and starts the scheduler. |
| `Python Source Code Files/NEWdrive.py` | Motor driver, encoder conversion, inner velocity PI controller, and motor task. |
| `Python Source Code Files/NEWlinecontrol.py` | QTR sensor array, centroid calculation, outer line PI controller, and dead-reckoned lane switch. |
| `Python Source Code Files/NEWtaskuser.py` | PuTTY command interface, numeric input, calibration workflow, motor tests, and manual drive. |
| `Python Source Code Files/NEWimudriver.py` | BNO055 register access, modes, heading, yaw rate, and calibration-profile reads/writes. |
| `Python Source Code Files/NEWline_calibration.py` | Validates, reads, and saves 13-channel black/white calibration data. |
| `Python Source Code Files/NEWimu_calibration.py` | Validates, reads, and saves the 22-byte IMUPLUS calibration profile. |
| `Python Source Code Files/NEWline_calibration.json` | Saved black/white readings for all 13 QTR channels. |
| `Python Source Code Files/NEWimu_calibration.json` | Saved BNO055 IMUPLUS offset profile. |
| `Python Source Code Files/NEWcotask.py` | Cooperative task scheduler and profiling support. |
| `PC Tools/NEWPC_auto_plotter.py` | PC-only serial test runner and motor-response plot generator. Do not upload it to Romi. |
| `tests/test_control_logic.py` | Desktop tests for control math, calibration validation, IMU access, lane switching, and motor tasks. |
| `tests/test_scheduler.py` | Desktop tests for task priority and missed-period behavior. |
| `tests/test_user_interface.py` | Desktop simulation of PuTTY commands and user-interface states. |
| `README.md` | Quick start, upload, calibration, and testing instructions. |
| `requirements.txt` | PC-only packages: Matplotlib and PySerial. |
| `.gitignore` | Ignores Python caches, VS Code settings, and generated motor plots/data. |
| `CODE_REFERENCE.md` | This reference document. |

## Classes and variables

### `NEWdrive.py`

| Class | Purpose | Persistent variables |
| --- | --- | --- |
| `MotorDriver` | Converts signed effort into a direction-pin state and PWM percentage. | `direction_pin`, `sleep_pin`, `pwm_channel`: hardware outputs; `direction`: motor-polarity multiplier. |
| `WheelEncoder` | Reads a timer encoder and converts counts to wheel position/velocity. | `timer`, `direction`, `position_counts`, `previous_count`, `count_change`, `elapsed_time`, `speed_counts`, `previous_time`. |
| `VelocityController` | Inner PI loop: wheel-velocity error to requested motor voltage. | `minimum_voltage`, `maximum_voltage`, `proportional_gain`, `integral_gain`, `integral_error`. |
| `MotorControlTask` | Runs one motor's encoder update, velocity PI loop, driver command, and log. | `motor_driver`, `wheel_encoder`, five log arrays, `input_voltage`, `running`, `finished`, `sample_index`, `test_duration`, `continuous`, `velocity_setpoint`, `velocity_controller`, `start_time`. |

Method locals: encoder `current_count/current_time` update measurement; PI `error`, `candidate_error`, `candidate_voltage`, and `requested_voltage` calculate one control pass; motor task `position`, `velocity`, `elapsed_time`, and `effort` are the current sample.

### `NEWlinecontrol.py`

| Class | Purpose | Persistent variables |
| --- | --- | --- |
| `LineSensorArray` | Samples 13 QTR channels, normalizes black/white readings, calculates centroid, logs line data, and controls emitters. | `sensor1` through `sensor13`, `sensors`, `sample_count`, `sensor_buffers`, `sample_timer`, `left_control_pin`, `right_control_pin`, `emitters_enabled`, black/white calibration values, `calibration_stage`, `center_sensor`, `line_detect_threshold`, three line-log arrays, `log_index`, `logging`, `log_start_time`. |
| `LineController` | Outer PI loop: centroid error to left/right wheel velocity setpoints; owns normal line-following state. | motor-task references, `velocity_setpoint`, `line_error`, line PI gains/integral, left/right velocity setpoints, `lane_switch_active`, `previous_time`, `enabled`, `line_sensor_array`, `lane_switch_controller`. |
| `LaneSwitchController` | Uses IMU heading, yaw rate, and wheel encoders to perform a two-concavity dead-reckoned lane shift. | `lane_width` (0.20 m), `turn_direction`, `wheel_radius`, yaw-rate settings, `timeout_ms`, state constants, `active`, `state`, `initial_heading`, `relative_heading`, `lateral_position`, `forward_distance`, `previous_time`, `start_time`, `next_distance_time`. |

Method locals: normalized sensor readings and `centroid` form the line error; outer PI uses `elapsed_time` and `correction`; lane dead reckoning uses left/right velocity, `forward_speed`, and heading; lane yaw control uses `measured_yaw_rate` and `yaw_rate_error`.

### `NEWtaskuser.py`

| Class | Purpose | Persistent variables |
| --- | --- | --- |
| `TaskUser` | Owns all user commands and state transitions. | State constants `MENU` through `LINE_DATA`; object references to motors, line controller/sensors, and IMU; `value` velocity command; velocity/line gains; `setpoint_entered`, `gains_entered`; `manual_speed`; `next_imu_status_time`; number-entry buffer/flag; `gain_step`; `output_motor_task`; `print_index`. |

Module variable `ser` is the MicroPython USB serial connection. Method locals include `command`, entered `number`, `status`, `gyro`, `accel`, and `saved`, each used only during one command update.

### `NEWimudriver.py` and calibration modules

| Class/module | Purpose | Persistent variables |
| --- | --- | --- |
| `BNO055` | Wraps BNO055 I2C registers. | Constants `DEV_ADDR`, `CONFIG_MODE`, `IMUPLUS_MODE`; `reg` register definitions; `_i2c`; reusable `_buf`. |
| `NEWimu_calibration` | Saves/loads a valid 22-byte IMUPLUS profile. | `IMU_MODE = "IMUPLUS"`. |
| `NEWline_calibration` | Saves/loads valid black/white QTR values. | No runtime object state; calibration dictionaries use `channels`, `black`, and `white`. |

### `NEWcotask.py`

| Class | Purpose | Persistent variables |
| --- | --- | --- |
| `RunningStatistics` | Tracks count, average, maximum, and standard deviation for profiling. | `count`, `mean`, `sum_squared_difference`, `maximum`. |
| `Task` | Wraps one generator and schedules it periodically or by trigger. | generator, `name`, `priority`, `period`, `_next_run`, `go_flag`, profiling flags/statistics, run count, and trigger time. |
| `TaskList` | Stores all `Task` objects and runs priority or round-robin scheduling. | task list and scheduler index. |

Module variable `task_list` is the scheduler used by `NEWmain.py`.

### PC tool and tests

`NEWPC_auto_plotter.py` owns PC constants such as serial port, baud rate, gains, setpoint, test command, timeout, and plot filenames. Its temporary variables hold serial lines and parsed motor data. Test files use fake clocks, pins, serial ports, motors, sensors, and IMUs to simulate hardware without Romi.

## `NEWmain.py` runtime objects

| Variable | Purpose |
| --- | --- |
| `pwm_timer` | Timer 1 PWM source for both motors. |
| `left_motor_driver`, `right_motor_driver` | Hardware motor drivers; task mapping accounts for the physical left/right wiring discovered during testing. |
| `left_encoder_timer`, `right_encoder_timer` | Timer 3 and Timer 2 encoder counters. |
| `imu_i2c`, `imu_sensor`, `chip_id` | BNO055 I2C connection, driver object, and startup wiring check. |
| `left_wheel_encoder`, `right_wheel_encoder` | Encoder conversion objects. |
| `left_log_*`, `right_log_*` | Preallocated five-second position, velocity, voltage, setpoint, and time logs. |
| `left_motor_task`, `right_motor_task` | Inner velocity-control tasks for the physical left and right wheels. |
| `line_sensor_array`, `line_controller` | Sensor/outer-control objects. |
| `user_interface` | PuTTY command object. |
| `channels`, `min_span`, `line_calibration_data`, `imu_calibration_data` | Startup calibration-loading variables. |

## Scheduled tasks

| Task | Generator | Period | Priority | Reason |
| --- | --- | ---: | ---: | --- |
| Line/Outer Control | `line_controller.run()` | 20 ms | 3 | Runs first so centroid/lane commands are ready before motor tasks use setpoints. |
| Left Motor Task | `left_motor_task.run()` | 20 ms | 2 | Updates encoder, velocity PI loop, motor effort, and log at 50 Hz. |
| Right Motor Task | `right_motor_task.run()` | 20 ms | 2 | Same 50 Hz closed-loop timing as the left wheel. Equal priority is appropriate because both wheels are one drivetrain. |
| User Input Task | `user_interface.run()` | 1 ms | 0 | Checks PuTTY promptly, but runs after control tasks so terminal handling cannot displace motor control. |

`NEWcotask.Task.ready()` skips missed periods rather than running a delayed task repeatedly. This prevents catch-up behavior from starving lower-priority tasks.

## State lists and transitions

### `TaskUser.state`

| State | Purpose | Exit |
| --- | --- | --- |
| `MENU` | Wait for top-level command. | `V`, `K`, `L`, `R`, `C`, `I`, `D`, `M`, or `E`. |
| `SETPOINT` | Receive signed velocity setpoint. | Enter accepts; blank Enter cancels. |
| `GAINS` | Receive velocity Kp/Ki then line Kp/Ki. | Four valid values complete; blank Enter cancels. |
| `LEFT_TEST`, `RIGHT_TEST` | Run one five-second motor test. | Natural completion or `E`. |
| `MOTOR_DATA` | Print one motor-log row per pass. | End of data. |
| `LINE_CALIBRATION` | Wait for black `B`, then white `W`. | Saves on `W`; `E` cancels. |
| `IMU_CALIBRATION` | Show gyro/accelerometer calibration status. | `W` saves only at 3/3; `E` cancels. |
| `LINE_FOLLOWING` | Normal centroid-to-velocity control. | `S` requests lane direction; `E` stops. |
| `LANE_DIRECTION` | Wait for lane `L` or `R`. | `L/R` starts switch; `E` returns to line following. |
| `MANUAL` | W/A/S/D differential-drive commands. | `E` stops. |
| `LINE_DATA` | Print one line-log row per pass. | End of data. |

### `LineSensorArray.calibration_stage`

| Stage | Purpose | Transition |
| --- | --- | --- |
| `0` | Idle. | `C` starts calibration. |
| `1` | Waiting for black reference. | `B` captures black and moves to stage 2. |
| `2` | Waiting for white reference. | `W` captures white, validates/saves, then returns to 0. |

### `LaneSwitchController.state`

| State | Purpose | Transition |
| --- | --- | --- |
| `0` / inactive | Normal line following. | `S`, then `L/R`, starts a lane switch. |
| `DEPARTURE` | First curve toward requested lane. Sensors are off; yaw-rate control runs. | Absolute lateral dead-reckoning reaches 0.10 m. |
| `RETURN` | Opposite curve toward a parallel heading. Sensors remain off; yaw-rate control runs. | Absolute lateral dead-reckoning reaches 0.20 m, then line following resumes. |
| timeout event | Fails safe if 8 s expires before 0.20 m. | Stops motors and disables line control. |

### `MotorControlTask` lifecycle

| Condition | Meaning |
| --- | --- |
| `running = False` | Driver disabled; generator yields without controlling a motor. |
| `start()` | Zeros encoder/log, resets PI integral, enables driver, and sets `running = True`. |
| `continuous = False` | Stops after five seconds or after the log fills. |
| `continuous = True` | Used by line/manual driving; runs until `stop()`. |
| `stop()` | Commands zero effort, disables driver, resets PI integral, and sets `finished = True`. |

## Important units

| Value | Unit |
| --- | --- |
| Wheel position | radians |
| Wheel velocity setpoint/measurement | radians per second |
| Motor controller output | volts, limited to -7.2 to +7.2 V |
| Motor driver effort | signed PWM percent, limited to -100 to +100 |
| Line centroid | sensor index, 0 through 12; center is 6 |
| Lane lateral/forward estimate | meters |
| Task periods | milliseconds |
