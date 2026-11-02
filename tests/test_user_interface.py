"""Desktop simulations for the Romi command interface."""

import importlib.util
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
SOURCE_ROOT = os.path.join(PROJECT_ROOT, "Python Source Code Files")
if SOURCE_ROOT not in sys.path:
    sys.path.insert(0, SOURCE_ROOT)


class FakeSerial:

    def __init__(self):
        self.input = []
        self.output = []

    def any(self):
        return len(self.input) > 0

    def read(self, count):
        value = "".join(self.input[:count])
        del self.input[:count]
        return value.encode()

    def write(self, value):
        self.output.append(value)

    def push(self, value):
        self.input.extend(value)


class FakePin:

    OUT_PP = 1

    class cpu:
        B13 = "B13"
        B14 = "B14"

    def __init__(self, pin, mode=None):
        self.pin = pin
        self.mode = mode

    def high(self):
        return None

    def low(self):
        return None


class FakePyb:
    Pin = FakePin
    Timer = object

    def USB_VCP(self):
        return FakeSerial()


class FakeTime:

    def __init__(self):
        self.milliseconds = 0

    def ticks_ms(self):
        return self.milliseconds

    def ticks_us(self):
        return self.milliseconds * 1000

    def ticks_diff(self, new, old):
        return new - old

    def ticks_add(self, value, offset):
        return value + offset


class FakeWheelEncoder:

    def get_velocity(self):
        return 0.0


class FakeMotorTask:

    def __init__(self):
        self.wheel_encoder = FakeWheelEncoder()
        self.finished = False
        self.started = []
        self.setpoints = []
        self.gains = []
        self.stopped = 0
        self.sample_index = 0
        self.log_time = []
        self.log_position = []
        self.log_velocity = []
        self.log_voltage = []
        self.log_setpoint = []

    def set_gains(self, proportional_gain, integral_gain):
        self.gains.append((proportional_gain, integral_gain))

    def start(self, setpoint, continuous=False):
        self.finished = False
        self.started.append((setpoint, continuous))

    def set_setpoint(self, setpoint):
        self.setpoints.append(setpoint)

    def stop(self):
        self.stopped += 1


class FakeLineController:

    def __init__(self, left_motor_task, right_motor_task):
        self.left_motor_task = left_motor_task
        self.right_motor_task = right_motor_task
        self.enabled = False
        self.lane_switch_active = False
        self.line_integral = 0.0
        self.velocity_setpoint = 0.0
        self.gains = []
        self.lane_setpoints = []
        self.stop_count = 0
        self.lane_switch_directions = []

    def enable(self, setpoint):
        self.enabled = True
        self.velocity_setpoint = setpoint
        if hasattr(self, "line_sensor_array"):
            self.line_sensor_array.enable_emitters()

    def stop(self):
        self.enabled = False
        self.stop_count += 1
        self.left_motor_task.stop()
        self.right_motor_task.stop()
        if hasattr(self, "line_sensor_array"):
            self.line_sensor_array.disable_emitters()

    def set_gains(self, proportional_gain, integral_gain):
        self.gains.append((proportional_gain, integral_gain))

    def set_lane_setpoints(self, left_setpoint, right_setpoint):
        self.lane_setpoints.append((left_setpoint, right_setpoint))

    def clear_lane_setpoints(self):
        self.lane_switch_active = False
        self.lane_setpoints.append(None)

    def request_lane_switch(self, direction):
        self.lane_switch_directions.append(direction)
        self.lane_switch_active = True
        return True


class FakeLineSensorArray:

    def __init__(self):
        self.black_values = [1] * 13
        self.white_values = [2] * 13
        self.logging = False
        self.log_index = 0
        self.log_time = []
        self.log_centroid = []
        self.log_line_error = []
        self.calibration_count = 0
        self.calibration_stage = 0
        self.emitters_enabled = False

    def enable_emitters(self):
        self.emitters_enabled = True

    def disable_emitters(self):
        self.emitters_enabled = False

    def start_logging(self):
        self.logging = True

    def stop_logging(self):
        self.logging = False

    def start_calibration(self):
        self.calibration_stage = 1

    def capture_calibration(self, command):
        if self.calibration_stage == 1 and command == "b":
            self.calibration_stage = 2
            return False

        if self.calibration_stage == 2 and command == "w":
            self.calibration_stage = 0
            self.calibration_count += 1
            return True

        return False


class FakeImu:

    def __init__(self):
        self.configuration_calls = 0
        self.imuplus_calls = 0
        self.status = 0
        self.saved_profile = bytes(range(22))

    def operating_mode_configuration(self):
        self.configuration_calls += 1

    def operating_mode_imuplus(self):
        self.imuplus_calls += 1

    def calibration_status(self):
        return self.status

    def calibration_profile_read(self):
        return self.saved_profile

    def euler(self):
        return 0.0, 0.0, 0.0


def load_module(name):
    path = os.path.join(SOURCE_ROOT, name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class UserInterfaceTests(unittest.TestCase):

    def setUp(self):
        sys.modules["pyb"] = FakePyb()
        self.line_module = load_module("NEWlinecontrol")
        self.user_module = load_module("NEWtaskuser")
        self.clock = FakeTime()
        self.user_module.time = self.clock
        self.line_module.time = self.clock
        self.serial = FakeSerial()
        self.user_module.ser = self.serial
        self.saved_line_calibrations = []
        self.saved_imu_calibrations = []
        self.user_module.NEWline_calibration.save_calibration = (
            lambda *args: self.saved_line_calibrations.append(args)
        )
        self.user_module.NEWimu_calibration.save_calibration = (
            lambda *args: self.saved_imu_calibrations.append(args)
        )

        self.left_motor_task = FakeMotorTask()
        self.right_motor_task = FakeMotorTask()
        self.line_controller = FakeLineController(
            self.left_motor_task,
            self.right_motor_task
        )
        self.line_sensor_array = FakeLineSensorArray()
        self.line_controller.line_sensor_array = self.line_sensor_array
        self.imu_sensor = FakeImu()
        self.user_interface = self.user_module.TaskUser(
            self.left_motor_task,
            self.right_motor_task,
            self.line_controller,
            self.line_sensor_array,
            self.imu_sensor
        )
        self.runner = self.user_interface.run()

    def press(self, text):
        self.serial.push(text)
        for _ in text:
            next(self.runner)

    def enter_number(self, number):
        self.press(str(number) + "\r")

    def enter_ready_motion_values(self):
        self.press("v")
        self.enter_number(8.0)
        self.press("k")
        for gain in (1.0, 0.1, 0.5, 0.05):
            self.enter_number(gain)

    def test_motion_commands_require_their_setup_values(self):
        self.press("l")
        self.press("r")
        self.press("d")
        self.press("m")

        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(self.left_motor_task.started, [])
        self.assertEqual(self.right_motor_task.started, [])

    def test_help_setpoint_and_gains_flow(self):
        self.press("h")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)

        self.enter_ready_motion_values()

        self.assertEqual(self.user_interface.value, 8.0)
        self.assertTrue(self.user_interface.setpoint_entered)
        self.assertTrue(self.user_interface.gains_entered)
        self.assertEqual(self.left_motor_task.gains[-1], (1.0, 0.1))
        self.assertEqual(self.right_motor_task.gains[-1], (1.0, 0.1))
        self.assertEqual(self.line_controller.gains[-1], (0.5, 0.05))

        self.press("v")
        self.enter_number("5..57")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.SETPOINT)
        self.assertEqual(self.user_interface.value, 8.0)

        self.enter_number("4.02.4")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.SETPOINT)
        self.assertEqual(self.user_interface.value, 8.0)

        self.enter_number("056")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.SETPOINT)
        self.assertEqual(self.user_interface.value, 8.0)

        self.enter_number("0.56")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(self.user_interface.value, 0.56)

    def test_motor_tests_and_calibration_commands_return_to_menu(self):
        self.enter_ready_motion_values()

        self.press("l")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.LEFT_TEST)
        self.assertEqual(self.left_motor_task.started[-1], (8.0, False))
        stop_count = self.left_motor_task.stopped
        self.press("e")
        self.assertEqual(self.left_motor_task.stopped, stop_count + 1)
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MOTOR_DATA)
        next(self.runner)
        next(self.runner)
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)

        self.press("r")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.RIGHT_TEST)
        self.assertEqual(self.right_motor_task.started[-1], (8.0, False))
        self.right_motor_task.finished = True
        next(self.runner)
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MOTOR_DATA)
        next(self.runner)
        next(self.runner)
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)

        self.press("c")
        self.assertEqual(
            self.user_interface.state,
            self.user_module.TaskUser.LINE_CALIBRATION
        )
        self.assertTrue(self.line_sensor_array.emitters_enabled)
        self.press("b")
        self.press("w")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(self.line_sensor_array.calibration_count, 1)
        self.assertEqual(len(self.saved_line_calibrations), 1)
        self.assertFalse(self.line_sensor_array.emitters_enabled)

        self.press("c")
        self.press("e")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertFalse(self.line_sensor_array.emitters_enabled)
        self.assertEqual(self.line_sensor_array.calibration_stage, 0)

    def test_imu_line_following_lane_switch_and_manual_commands(self):
        self.press("i")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.IMU_CALIBRATION)
        self.assertEqual(self.imu_sensor.imuplus_calls, 1)
        self.press("e")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(self.imu_sensor.imuplus_calls, 1)

        self.enter_ready_motion_values()
        self.press("d")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.LINE_FOLLOWING)
        self.assertTrue(self.line_controller.enabled)
        self.assertTrue(self.line_sensor_array.logging)
        self.assertTrue(self.line_sensor_array.emitters_enabled)
        self.assertEqual(self.left_motor_task.started[-1], (8.0, True))
        self.assertEqual(self.right_motor_task.started[-1], (8.0, True))

        self.press("s")
        self.assertEqual(
            self.user_interface.state,
            self.user_module.TaskUser.LANE_DIRECTION
        )
        self.press("l")
        self.assertEqual(self.line_controller.lane_switch_directions, [1])
        self.assertTrue(self.line_controller.lane_switch_active)
        self.press("e")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.LINE_DATA)
        next(self.runner)
        next(self.runner)
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertFalse(self.line_controller.enabled)
        self.assertFalse(self.line_sensor_array.logging)
        self.assertFalse(self.line_sensor_array.emitters_enabled)

        self.press("mwasde")
        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(self.left_motor_task.setpoints[-4:], [7.5, 7.5, -7.5, -7.5])
        self.assertEqual(self.right_motor_task.setpoints[-4:], [7.5, -7.5, -7.5, 7.5])

    def test_completed_imu_calibration_saves_and_restores_normal_mode(self):
        self.imu_sensor.status = 0xff
        self.press("i")
        self.clock.milliseconds = 25
        next(self.runner)

        self.assertEqual(self.user_interface.state, self.user_module.TaskUser.MENU)
        self.assertEqual(len(self.saved_imu_calibrations), 1)
        self.assertEqual(self.imu_sensor.imuplus_calls, 1)


if __name__ == "__main__":
    unittest.main()
