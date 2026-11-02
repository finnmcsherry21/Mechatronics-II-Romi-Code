"""Desktop tests for logic that does not require the STM32 hardware."""

import importlib.util
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
SOURCE_ROOT = os.path.join(PROJECT_ROOT, "Python Source Code Files")
if SOURCE_ROOT not in sys.path:
    sys.path.insert(0, SOURCE_ROOT)


def load_module(name):
    path = os.path.join(SOURCE_ROOT, name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeTime:
    def __init__(self):
        self.us = 0
        self.ms = 0

    def advance(self, milliseconds):
        self.ms += milliseconds
        self.us += milliseconds * 1000

    def ticks_us(self):
        return self.us

    def ticks_ms(self):
        return self.ms

    @staticmethod
    def ticks_diff(new, old):
        return new - old


class FakeEncoder:
    def __init__(self):
        self.dt = 0.02
        self.update_count = 0

    def update(self):
        self.update_count += 1

    @staticmethod
    def get_position():
        return 0.0

    def get_velocity(self):
        return 0.0


class FakeTask:
    def __init__(self):
        self.encoder = FakeEncoder()


class FakeMotor:
    def __init__(self):
        self.enabled = False
        self.efforts = []

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def set_effort(self, effort):
        self.efforts.append(effort)


class FakeOuterController:
    def __init__(self):
        self.enabled = True
        self.lane_switching = False
        self.line_integral = 0.0
        self.velocity_setpoint = 7.5
        self.left_task = FakeTask()
        self.right_task = FakeTask()
        self.commands = None
        self.cleared = False

    def set_lane_setpoints(self, left, right):
        self.commands = (left, right)

    def clear_lane_setpoints(self):
        self.cleared = True


class FakeImu:
    def __init__(self):
        self.heading = 0.0

    def euler(self):
        return self.heading, 0.0, 0.0


class ClosedLoopTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("NEWclosedloop")

    def test_output_is_bounded(self):
        controller = self.module.ClosedLoop(k_p=10.0, k_i=10.0)
        self.assertEqual(controller.update(100.0, 0.0, 1.0), 7.2)
        self.assertEqual(controller.update(-100.0, 0.0, 1.0), -7.2)

    def test_rejects_negative_gains(self):
        with self.assertRaises(ValueError):
            self.module.ClosedLoop(k_p=-1.0)


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


class LaneSwitchTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("NEWlaneswitch")
        self.clock = FakeTime()
        self.module.time = self.clock
        self.outer = FakeOuterController()
        self.imu = FakeImu()
        self.switch = self.module.LaneSwitch(self.outer, self.imu)

    def test_lane_switch_commands_wheels_then_completes(self):
        self.assertTrue(self.switch.enable_lane_switching())
        self.clock.advance(20)
        self.switch.run()
        self.assertTrue(self.outer.lane_switching)
        self.assertIsNotNone(self.outer.commands)

        self.switch.d_hat = self.switch.lane_width
        self.imu.heading = self.switch.initial_heading
        self.clock.advance(20)
        self.switch.run()
        self.assertFalse(self.outer.lane_switching)
        self.assertTrue(self.outer.cleared)


class TaskMotorTests(unittest.TestCase):
    def test_control_and_logging_happen_each_scheduled_pass(self):
        module = load_module("NEWtaskmotor")
        clock = FakeTime()
        module.time = clock

        motor = FakeMotor()
        encoder = FakeEncoder()
        data = [[0.0] * 4 for _ in range(5)]
        task = module.TaskMotor(motor, encoder, *data)
        task.set_gains(1.0, 0.0)
        task.start(5.0)
        runner = task.RunMethod()

        clock.advance(20)
        next(runner)
        clock.advance(20)
        next(runner)

        self.assertEqual(encoder.update_count, 2)
        self.assertEqual(task.i, 2)
        self.assertTrue(motor.enabled)
        self.assertGreater(motor.efforts[-1], 0.0)

    def test_lane_switch_times_out_safely(self):
        self.assertTrue(self.switch.enable_lane_switching())
        self.clock.advance(self.switch.timeout_ms + 1)
        self.switch.run()
        self.assertFalse(self.outer.lane_switching)
        self.assertTrue(self.outer.cleared)


if __name__ == "__main__":
    unittest.main()
