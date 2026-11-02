"""Desktop timing tests for the cooperative task scheduler."""

import importlib.util
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
SOURCE_ROOT = os.path.join(PROJECT_ROOT, "Python Source Code Files")


class FakeClock:

    def __init__(self):
        self.microseconds = 0

    def advance(self, milliseconds):
        self.microseconds += milliseconds * 1000

    def ticks_us(self):
        return self.microseconds

    @staticmethod
    def ticks_add(value, offset):
        return value + offset

    @staticmethod
    def ticks_diff(new, old):
        return new - old


class FakeMicroPython:

    @staticmethod
    def native(function):
        return function


def load_scheduler(clock):
    sys.modules["micropython"] = FakeMicroPython()
    sys.modules["utime"] = clock
    path = os.path.join(SOURCE_ROOT, "NEWcotask.py")
    spec = importlib.util.spec_from_file_location("NEWcotask_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def task_runner(name, events):
    while True:
        events.append(name)
        yield


class SchedulerTests(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.module = load_scheduler(self.clock)

    def test_priority_scheduler_runs_ready_tasks_highest_priority_first(self):
        events = []
        task_list = self.module.TaskList()
        task_list.append(self.module.Task(task_runner("user", events), priority=0, period=20))
        task_list.append(self.module.Task(task_runner("left", events), priority=2, period=20))
        task_list.append(self.module.Task(task_runner("right", events), priority=2, period=20))
        task_list.append(self.module.Task(task_runner("line", events), priority=3, period=20))

        self.clock.advance(21)
        for _ in range(4):
            task_list.pri_sched()

        self.assertEqual(events[0], "line")
        self.assertCountEqual(events, ["line", "left", "right", "user"])

    def test_periodic_tasks_run_once_per_due_period(self):
        events = []
        task_list = self.module.TaskList()
        task_list.append(self.module.Task(task_runner("line", events), priority=3, period=20))
        task_list.append(self.module.Task(task_runner("motor", events), priority=2, period=20))
        task_list.append(self.module.Task(task_runner("user", events), priority=0, period=1))

        for _ in range(5):
            self.clock.advance(20)
            for _ in range(3):
                task_list.pri_sched()

        self.assertEqual(events.count("line"), 5)
        self.assertEqual(events.count("motor"), 5)
        self.assertEqual(events.count("user"), 5)


if __name__ == "__main__":
    unittest.main()
