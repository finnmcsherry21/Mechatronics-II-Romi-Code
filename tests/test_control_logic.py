"""Desktop tests for control logic that does not require the STM32."""

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

    @staticmethod
    def USB_VCP():
        return FakeSerial()


sys.modules["pyb"] = FakePyb()


class FakeMicroPython:
    @staticmethod
    def const(value):
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

    @staticmethod
    def ticks_diff(new, old):
        return new - old


class FakeWheelEncoder:
    def __init__(self):
        self.elapsed_time = 0.02
        self.update_count = 0

    def update(self):
        self.update_count += 1

    def zero(self):
        self.update_count = 0

    @staticmethod
    def get_position():
        return 0.0

    @staticmethod
    def get_velocity():
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

    def euler(self):
        return self.heading, 0.0, 0.0

    @staticmethod
    def yaw_rate():
        return 0.0


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
        controller = self.module.VelocityController(10.0, 10.0)
        self.assertEqual(controller.update(100.0, 0.0, 1.0), 7.2)
        self.assertEqual(controller.update(-100.0, 0.0, 1.0), -7.2)

    def test_rejects_negative_gains(self):
        with self.assertRaises(ValueError):
            self.module.VelocityController(-1.0)


class CalibrationTests(unittest.TestCase):
    def test_line_calibration_rejects_missing_keys(self):
        module = load_module("NEWline_calibration")
        with self.assertRaises(ValueError):
            module.validate_calibration({}, list(range(1, 14)), 50)

    def test_imu_calibration_requires_bytes(self):
        module = load_module("NEWimu_calibration")
        with self.assertRaises(ValueError):
            module.validate_calibration([0] * 21)
        with self.assertRaises(ValueError):
            module.validate_calibration([256] * 22)
        with self.assertRaises(ValueError):
            module.validate_calibration([0] * 22)


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

    def test_lane_switch_captures_target_after_clearing_departure_line(self):
        self.assertTrue(self.lane_switch.start(1))
        line_sensor_array = FakeLineSensorArray([False, True])
        self.clock.advance(20)
        self.lane_switch.update(line_sensor_array)
        self.assertTrue(self.line_controller.lane_switch_active)
        self.assertIsNotNone(self.line_controller.setpoints)

        self.clock.advance(20)
        self.lane_switch.update(line_sensor_array)
        self.assertFalse(self.line_controller.lane_switch_active)
        self.assertTrue(self.line_controller.cleared)

    def test_lane_switch_times_out_safely(self):
        self.assertTrue(self.lane_switch.start(-1))
        line_sensor_array = FakeLineSensorArray([True])
        self.clock.advance(self.lane_switch.timeout_ms + 1)
        self.lane_switch.update(line_sensor_array)
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
        line_controller.set_gains(0.5, 0.1)
        line_controller.previous_time = clock.ticks_us()

        line_controller.set_line_error(1.0)
        clock.advance(20)
        line_controller.update_setpoints()
        self.assertLess(line_controller.left_velocity_setpoint, 8.0)
        self.assertGreater(line_controller.right_velocity_setpoint, 8.0)

        line_controller.set_line_error(-1.0)
        clock.advance(20)
        line_controller.update_setpoints()
        self.assertGreater(line_controller.left_velocity_setpoint, 8.0)
        self.assertLess(line_controller.right_velocity_setpoint, 8.0)


if __name__ == "__main__":
    unittest.main()
