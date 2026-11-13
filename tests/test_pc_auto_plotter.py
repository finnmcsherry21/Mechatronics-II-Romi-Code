"""Desktop tests for the Romi motor-response plotter."""

import importlib.util
import os
import tempfile
import unittest
from unittest.mock import patch


os.environ.setdefault("MPLBACKEND", "Agg")
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
PLOTTER_PATH = os.path.join(PROJECT_ROOT, "PC Tools", "NEWPC_auto_plotter.py")


def load_plotter():
    spec = importlib.util.spec_from_file_location("NEWPC_auto_plotter_test", PLOTTER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeClock:

    def __init__(self):
        self.seconds = 0.0

    def __call__(self):
        return self.seconds

    def advance(self, seconds):
        self.seconds += seconds


class FakeSerial:

    def __init__(self, lines=(), clock=None, empty_wait=0.0):
        self.lines = list(lines)
        self.clock = clock
        self.empty_wait = empty_wait
        self.writes = []

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        if self.clock is not None:
            self.clock.advance(self.empty_wait)
        return b""

    def write(self, value):
        self.writes.append(value)


class AutoPlotterTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.plotter = load_plotter()

    def test_cleaned_fields_ignores_comments_and_blank_lines(self):
        self.assertEqual(self.plotter.cleaned_fields(" 1, 2 # note"), ["1", "2"])
        self.assertEqual(self.plotter.cleaned_fields(" # note"), [])

    def test_collect_dataset_keeps_only_complete_finite_rows(self):
        serial_port = FakeSerial([
            b"Romi is ready\r\n",
            b"Time(s), Position(rad), Velocity(rad/s), Input Voltage(V), Setpoint(rad/s)\r\n",
            b"0, 0.1, 2.0, 3.0, 2.5\r\n",
            b"1, 0.2, nan, 3.1, 2.5\r\n",
            b"2, 0.3, 2.1, 3.2\r\n",
            b"3, 0.4, 2.2, 3.3, 2.5, extra\r\n",
            b"4, 0.5, 2.3, 3.4, 2.5\r\n",
            b"End of data # completed\r\n",
        ])

        headers, columns = self.plotter.collect_dataset(serial_port)

        self.assertEqual(headers, list(self.plotter.EXPECTED_HEADERS))
        self.assertEqual(columns, [
            [0.0, 4.0], [0.1, 0.5], [2.0, 2.3], [3.0, 3.4], [2.5, 2.5],
        ])

    def test_collect_dataset_times_out_before_a_header(self):
        clock = FakeClock()
        serial_port = FakeSerial(clock=clock, empty_wait=self.plotter.FIRST_RESPONSE_TIMEOUT)

        with self.assertRaisesRegex(TimeoutError, "header"):
            self.plotter.collect_dataset(serial_port, clock=clock)

    def test_collect_dataset_times_out_after_a_partial_response(self):
        clock = FakeClock()
        serial_port = FakeSerial([
            b"Time(s), Position(rad), Velocity(rad/s), Input Voltage(V), Setpoint(rad/s)\r\n",
            b"0, 0.1, 2.0, 3.0, 2.5\r\n",
        ], clock=clock, empty_wait=self.plotter.BETWEEN_LINES_TIMEOUT)

        with self.assertRaisesRegex(TimeoutError, "before End of data"):
            self.plotter.collect_dataset(serial_port, clock=clock)

    def test_send_setup_and_test_commands_match_firmware_protocol(self):
        serial_port = FakeSerial()
        headers = list(self.plotter.EXPECTED_HEADERS)
        columns = [[0.0], [0.0], [0.0], [0.0], [self.plotter.VELOCITY]]

        with patch.object(self.plotter, "collect_dataset", return_value=(headers, columns)), \
             patch.object(self.plotter, "save_response_plot") as save_plot:
            self.plotter.run_motor_tests(serial_port)

        expected_setup = [
            b"k",
            b"0.25\r\n", b"0.05\r\n", b"0.25\r\n", b"0.05\r\n",
            b"v", b"7.5\r\n",
        ]
        self.assertEqual(serial_port.writes, expected_setup + [b"l"] + expected_setup + [b"r"])
        self.assertEqual(save_plot.call_args_list[0].args[0], "left")
        self.assertEqual(save_plot.call_args_list[1].args[0], "right")

    def test_straight_drive_sends_duration_and_saves_plot(self):
        serial_port = FakeSerial()
        headers = list(self.plotter.EXPECTED_HEADERS)
        columns = [[0.0], [0.0], [0.0], [0.0], [self.plotter.VELOCITY]]

        with patch.object(self.plotter, "collect_dataset", return_value=(headers, columns)), \
             patch.object(self.plotter, "save_response_plot") as save_plot:
            self.plotter.run_straight_drive(serial_port, duration=2.5)

        expected_setup = [
            b"k",
            b"0.25\r\n", b"0.05\r\n", b"0.25\r\n", b"0.05\r\n",
            b"v", b"7.5\r\n",
        ]
        self.assertEqual(
            serial_port.writes,
            expected_setup + [b"u", b"2.5\r\n"]
        )
        self.assertEqual(save_plot.call_args.args[0], "straight")

    def test_save_response_plot_creates_a_png(self):
        headers = list(self.plotter.EXPECTED_HEADERS)
        columns = [[0.0, 1.0], [0.0, 0.2], [1.0, 2.0], [2.0, 3.0], [1.5, 1.5]]

        with tempfile.TemporaryDirectory() as directory:
            filename = self.plotter.save_response_plot("left", headers, columns, directory)
            self.assertEqual(filename.name, "latest_left_motor_response.png")
            self.assertTrue(filename.is_file())
            self.assertGreater(filename.stat().st_size, 0)

    def test_plot_rejects_misaligned_columns(self):
        headers = list(self.plotter.EXPECTED_HEADERS)
        columns = [[0.0], [0.0], [0.0, 1.0], [0.0], [0.0]]

        with self.assertRaisesRegex(ValueError, "different lengths"):
            self.plotter.save_response_plot("left", headers, columns)


if __name__ == "__main__":
    unittest.main()
