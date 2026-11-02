# Mechatronics II Romi Code

MicroPython firmware and a companion PC data-collection script for a Pololu
Romi differential-drive robot. Firmware module names intentionally retain the
`NEW` prefix.

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

Run `NEWPC_auto_plotter.py` on the computer, not on the STM32. It requires
`pyserial` and `matplotlib`; set `SERIAL_PORT` to the COM port assigned to the
Romi's USB virtual COM port.

The script sends gains and a velocity setpoint, triggers a left-wheel test,
then saves a CSV, velocity plot, and voltage plot. Generated files are ignored
by Git.

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
and are intentionally versioned. Recalibrate if the sensor hardware, mounting,
surface, or lighting changes substantially.

## Notes

`NEWcotask.py` retains its GPLv3 attribution from the course scheduler source.
Review that file's header before selecting a repository-wide license.
