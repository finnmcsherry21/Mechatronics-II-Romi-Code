"""Small, MicroPython-compatible running statistics helper for NEWcotask."""


class RunningStats:
    """Compute mean, maximum, and population standard deviation incrementally."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._count = 0
        self._mean = 0.0
        self._m2 = 0.0
        self.max = 0.0

    def update(self, value):
        value = float(value)
        self._count += 1
        delta = value - self._mean
        self._mean += delta / self._count
        self._m2 += delta * (value - self._mean)
        if self._count == 1 or value > self.max:
            self.max = value

    @property
    def mean(self):
        return self._mean

    @property
    def std(self):
        if self._count < 2:
            return 0.0
        return (self._m2 / self._count) ** 0.5
