"""Desktop tests for control logic that does not require the STM32."""

# Run all 16 tests from the project folder:
# python -B -m unittest discover -s tests
#
# 1. test_output_is_bounded
# 2. test_rejects_negative_gains
# 3. test_line_calibration_rejects_missing_keys
# 4. test_imu_calibration_requires_bytes
# 5. test_imuplus_mode_and_yaw_rate_use_expected_units
# 6. test_lane_switch_reaches_dead_reckoned_distance
# 7. test_lane_switch_times_out_safely
# 8. test_control_and_logging_happen_each_scheduled_pass
# 9. test_motor_and_line_setpoints_are_limited
# 10. test_priority_scheduler_runs_ready_tasks_highest_priority_first
# 11. test_periodic_tasks_run_once_per_due_period
# 12. test_motion_commands_require_their_setup_values
# 13. test_help_setpoint_and_gains_flow
# 14. test_motor_tests_and_calibration_commands_return_to_menu
# 15. test_imu_line_following_lane_switch_and_manual_commands
# 16. test_completed_imu_calibration_saves_and_restores_normal_mode

import importlib.util
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
SOURCE_ROOT = os.path.join(PROJECT_ROOT, "Python Source Code Files")
if SOURCE_ROOT not in sys.path:
    sys.path.insert(0, SOURCE_ROOT)


class FakeSerial:
    def any(self):
        return False


class FakePyb:
    Pin = object
    Timer = object

    def USB_VCP(self):
        return FakeSerial()


sys.modules["pyb"] = FakePyb()


class FakeMicroPython:
    def const(self, value):
        return value


sys.modules["micropython"] = FakeMicroPython()


def load_module(name):
    path = os.path.join(SOURCE_ROOT, name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeTime:
    def __init__(self):
        self.microseconds = 0
        self.milliseconds = 0

    def advance(self, milliseconds):
        self.milliseconds += milliseconds
        self.microseconds += milliseconds * 1000

    def ticks_us(self):
        return self.microseconds

    def ticks_ms(self):
        return self.milliseconds

    def ticks_diff(self, new, old):
        return new - old

    def ticks_add(self, value, offset):
        return value + offset


class FakeWheelEncoder:
    def __init__(self):
        self.elapsed_time = 0.02
        self.update_count = 0

    def update(self):
        self.update_count += 1

    def zero(self):
        self.update_count = 0

    def get_position(self):
        return 0.0

    def get_velocity(self):
        return 0.0


class FakeMotorTask:
    def __init__(self):
        self.wheel_encoder = FakeWheelEncoder()


class FakeMotorDriver:
    def __init__(self):
        self.enabled = False
        self.efforts = []

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def set_effort(self, effort):
        self.efforts.append(effort)


class FakeLineController:
    def __init__(self):
        self.enabled = True
        self.lane_switch_active = False
        self.line_integral = 0.0
        self.velocity_setpoint = 7.5
        self.left_motor_task = FakeMotorTask()
        self.right_motor_task = FakeMotorTask()
        self.setpoints = None
        self.cleared = False
        self.stop_count = 0
        self.line_sensor_array = None

    def set_lane_setpoints(self, left_setpoint, right_setpoint):
        self.setpoints = (left_setpoint, right_setpoint)

    def clear_lane_setpoints(self):
        self.cleared = True

    def stop(self):
        self.stop_count += 1
        self.enabled = False


class FakeImu:
    def __init__(self):
        self.heading = 0.0
        self.current_yaw_rate = 0.0

    def euler(self):
        return self.heading, 0.0, 0.0

    def yaw_rate(self):
        return self.current_yaw_rate


class FakeI2c:
    def __init__(self):
        self.writes = []

    def mem_read(self, buffer, address, register):
        if register == 0x18:
            buffer[0] = 16
            buffer[1] = 0

    def mem_write(self, value, address, register):
        self.writes.append((value, address, register))


class FakeLineSensorArray:
    def __init__(self, detections):
        self.detections = detections

    def line_detected(self):
        return self.detections.pop(0)


class VelocityControllerTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("NEWdrive")

    def test_output_is_bounded(self):
        controller = self.module.VelocityController(2.0, 1.0)
        self.assertEqual(controller.update(1.0, 0.0, 0.5), 2.5)
        self.assertEqual(controller.update(1.0, 0.0, 0.5), 3.0)

        controller = self.module.VelocityController(10.0, 10.0)
        self.assertEqual(controller.update(100.0, 0.0, 1.0), 7.2)
        self.assertEqual(controller.update(-100.0, 0.0, 1.0), -7.2)

    def test_rejects_negative_gains(self):
        with self.assertRaises(ValueError):
            self.module.VelocityController(-1.0)


class CalibrationTests(unittest.TestCase):
    def test_gains_round_trip_and_rejects_invalid_data(self):
        module = load_module("NEWgains")
        gains = {
            "velocity_kp": 1.0,
            "velocity_ki": 0.1,
            "line_kp": 0.5,
            "line_ki": 0.05,
        }
        filename = os.path.join(PROJECT_ROOT, "tests", "gains_test.json")
        try:
            module.save_gains(filename, gains)
            loaded_gains = module.load_gains(filename)
        finally:
            if os.path.exists(filename):
                os.remove(filename)

        self.assertEqual(loaded_gains, gains)
        with self.assertRaises(ValueError):
            module.validate_gains({"velocity_kp": 1.0})
        with self.assertRaises(ValueError):
            module.validate_gains(dict(gains, line_ki=-0.1))

    def test_line_calibration_rejects_missing_keys(self):
        module = load_module("NEWline_calibration")
        with self.assertRaises(ValueError):
            module.validate_calibration({}, list(range(1, 14)), 50)

        channels = list(range(1, 14))
        calibration = {
            "channels": channels,
            "black": [1000] * 13,
            "white": [100] * 13
        }

        filename = os.path.join(PROJECT_ROOT, "tests", "line_calibration_test.json")
        try:
            module.save_calibration(filename, calibration, channels, 50)
            loaded_calibration = module.load_calibration(filename, channels, 50)
        finally:
            if os.path.exists(filename):
                os.remove(filename)

        self.assertEqual(loaded_calibration, calibration)

    def test_imu_calibration_requires_bytes(self):
        module = load_module("NEWimu_calibration")
        with self.assertRaises(ValueError):
            module.validate_calibration([0] * 21)
        with self.assertRaises(ValueError):
            module.validate_calibration([256] * 22)
        with self.assertRaises(ValueError):
            module.validate_calibration([0] * 22)

        profile = list(range(1, 23))
        filename = os.path.join(PROJECT_ROOT, "tests", "imu_calibration_test.json")
        try:
            module.save_calibration(filename, profile)
            loaded_profile = module.load_calibration(filename)
        finally:
            if os.path.exists(filename):
                os.remove(filename)

        self.assertEqual(loaded_profile, profile)


class ImuDriverTests(unittest.TestCase):
    def test_imuplus_mode_and_yaw_rate_use_expected_units(self):
        module = load_module("NEWimudriver")
        module.sleep = lambda seconds: None
        i2c = FakeI2c()
        imu = module.BNO055(i2c)

        self.assertAlmostEqual(imu.yaw_rate(), 0.0174532925)
        imu.operating_mode_configuration()
        imu.operating_mode_imuplus()

        self.assertEqual(i2c.writes[-2][0], 0x00)
        self.assertEqual(i2c.writes[-1][0], 0x08)


class LaneSwitchTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("NEWlinecontrol")
        self.clock = FakeTime()
        self.module.time = self.clock
        self.line_controller = FakeLineController()
        self.imu_sensor = FakeImu()
        self.lane_switch = self.module.LaneSwitchController(
            self.line_controller,
            self.imu_sensor
        )

    def test_lane_switch_reaches_dead_reckoned_distance(self):
        self.assertTrue(self.lane_switch.start(1))
        self.clock.advance(20)
        self.lane_switch.update()
        self.assertTrue(self.line_controller.lane_switch_active)
        self.assertIsNotNone(self.line_controller.setpoints)
        self.assertLess(self.line_controller.setpoints[0], 7.5)
        self.assertGreater(self.line_controller.setpoints[1], 7.5)

        self.lane_switch.lateral_position = 0.10
        self.clock.advance(20)
        self.lane_switch.update()
        self.assertTrue(self.line_controller.lane_switch_active)

        self.clock.advance(20)
        self.lane_switch.update()
        self.assertTrue(self.line_controller.lane_switch_active)
        self.assertGreater(self.line_controller.setpoints[0], 7.5)
        self.assertLess(self.line_controller.setpoints[1], 7.5)

        self.lane_switch.lateral_position = 0.20
        self.clock.advance(20)
        self.lane_switch.update()
        self.assertFalse(self.line_controller.lane_switch_active)
        self.assertTrue(self.line_controller.cleared)

    def test_lane_switch_times_out_safely(self):
        self.assertTrue(self.lane_switch.start(-1))
        self.clock.advance(self.lane_switch.timeout_ms + 1)
        self.lane_switch.update()
        self.assertFalse(self.line_controller.lane_switch_active)
        self.assertTrue(self.line_controller.cleared)
        self.assertEqual(self.line_controller.stop_count, 1)


class MotorControlTaskTests(unittest.TestCase):
    def test_control_and_logging_happen_each_scheduled_pass(self):
        module = load_module("NEWdrive")
        clock = FakeTime()
        module.time = clock

        motor_driver = FakeMotorDriver()
        wheel_encoder = FakeWheelEncoder()
        logs = [[0.0] * 4 for _ in range(5)]
        motor_task = module.MotorControlTask(
            motor_driver,
            wheel_encoder,
            *logs
        )
        motor_task.set_gains(1.0, 0.0)
        motor_task.start(5.0)
        runner = motor_task.run()

        clock.advance(20)
        next(runner)
        clock.advance(20)
        next(runner)

        self.assertEqual(wheel_encoder.update_count, 2)
        self.assertEqual(motor_task.sample_index, 2)
        self.assertTrue(motor_driver.enabled)
        self.assertGreater(motor_driver.efforts[-1], 0.0)

    def test_motor_and_line_setpoints_are_limited(self):
        module = load_module("NEWdrive")
        logs = [[0.0] * 4 for _ in range(5)]
        motor_task = module.MotorControlTask(
            FakeMotorDriver(),
            FakeWheelEncoder(),
            *logs
        )
        motor_task.set_setpoint(100.0)
        self.assertEqual(motor_task.velocity_setpoint, 25.0)
        motor_task.set_setpoint(-100.0)
        self.assertEqual(motor_task.velocity_setpoint, -25.0)

        line_module = load_module("NEWlinecontrol")
        clock = FakeTime()
        line_module.time = clock
        line_controller = line_module.LineController(
            FakeMotorTask(),
            FakeMotorTask()
        )
        line_controller.set_velocity_setpoint(8.0)
        line_controller.set_gains(0.1, 0.5)
        line_controller.previous_time = clock.ticks_us()

        line_controller.set_line_error(1.0)
        clock.advance(20)
        line_controller.update_setpoints()
        first_left_setpoint = line_controller.left_velocity_setpoint
        self.assertGreater(first_left_setpoint, 8.0)
        self.assertLess(line_controller.right_velocity_setpoint, 8.0)

        clock.advance(20)
        line_controller.update_setpoints()
        self.assertGreater(line_controller.left_velocity_setpoint, first_left_setpoint)

        line_controller.set_line_error(-1.0)
        clock.advance(20)
        line_controller.update_setpoints()
        self.assertLess(line_controller.left_velocity_setpoint, 8.0)
        self.assertGreater(line_controller.right_velocity_setpoint, 8.0)


if __name__ == "__main__":
    unittest.main()
