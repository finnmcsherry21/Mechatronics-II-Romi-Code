# Mechatronics II Romi Code

MicroPython firmware and a companion PC data-collection script for a Pololu
Romi differential-drive robot. Firmware module names intentionally retain the
`NEW` prefix.

## Firmware layout

- `main.py` starts `NEWmain.py` on the STM32.
- `NEWdrive.py` contains motor driving, encoder measurement, velocity control,
  and the motor-control task.
- `NEWlinecontrol.py` contains line sensing, line following, and lane
  switching.
- `NEWtaskuser.py` handles PuTTY commands, calibration, and test workflow.
- `NEWimudriver.py` and the two calibration modules handle the BNO055 in
  IMUPLUS mode and saved gyro/accelerometer calibration data.

## Firmware upload

Upload every file in `Python Source Code Files` to the STM32's MicroPython
filesystem, including `main.py`. The small unprefixed `main.py` file is only a
startup wrapper; it imports and runs `NEWmain.main()`. All implementation
modules and calibration files retain their `NEW` prefix.

Before the first powered test, place the robot on blocks with its wheels clear
of the floor. Open PuTTY, set the velocity and controller gains, and run a
single-wheel test. Verify motor direction and encoder sign before using line
following or lane switching.

## PC data collection

Run `PC Tools/NEWPC_auto_plotter.py` on the computer, not on the STM32. It requires
`pyserial` and `matplotlib`; set `SERIAL_PORT` to the COM port assigned to the
Romi's USB virtual COM port.

The script sends gains and a velocity setpoint, triggers left- and right-wheel
tests, then overwrites `latest_left_motor_response.png` and
`latest_right_motor_response.png`. It does not save CSV files.

## Saved controller gains

At the Romi command menu, press `G` to enter the velocity Kp/Ki and line Kp/Ki
gains and save them as `NEWoptimal_gains.json`. Valid saved gains are applied
automatically on the next startup; you still need to enter a velocity setpoint
with `V` before starting a motor test or line following. Press `K` to apply a
temporary set of gains without changing the saved file.

## Desktop checks

After installing the PC requirements, run this from the repository root:

```powershell
python -B -m unittest discover -s tests
```

These tests validate control math, calibration validation, and lane-switch
fail-safe behavior without connecting the Romi. They cannot validate the
physical motor direction, encoder sign, sensor polarity, or final gain tuning.

## Calibration files

`NEWline_calibration.json` and `NEWimu_calibration.json` are firmware inputs
and are intentionally versioned. The included IMU profile is intentionally
invalid until you calibrate with `I`; this prevents placeholder data from being
applied to the BNO055. Recalibrate if the sensor hardware, mounting, surface,
or lighting changes substantially.

## Lane switching

Start line following with `D`. While following a line, press `S`, then `L` for
a left lane switch or `R` for a right lane switch. The line-control task uses
IMU yaw-rate feedback while changing lanes, waits until it has cleared the
departure line before accepting a target line, and stops both motors if the
target is missed before the timeout.

## Power behavior

Both motor drivers start disabled and are enabled only for a requested motor
test, manual drive, or line-following run. QTR emitter controls PB13 and PB14
start low, are enabled only for line calibration or line following, and are
forced low on every stop and scheduler exit.

## Notes

`NEWcotask.py` retains its GPLv3 attribution from the course scheduler source.
Review that file's header before selecting a repository-wide license.
