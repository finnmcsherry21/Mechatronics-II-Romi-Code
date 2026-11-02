from time import sleep

from micropython import const
from struct import calcsize, unpack_from
import math


class BNO055:

    DEV_ADDR = const(0x28)
    CONFIG_MODE = const(0x00)
    IMUPLUS_MODE = const(0x08)

    class reg:
        EUL_DATA_ALL = (const(0x1A), b"<hhh")
        GYRO_DATA_Z = (const(0x18), b"<h")

    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray(22)
        sleep(1)

    def _read_reg(self, reg):
        length = calcsize(reg[1])
        buffer = memoryview(self._buf)[:length]
        self._i2c.mem_read(buffer, self.DEV_ADDR, reg[0])
        return unpack_from(reg[1], buffer)

    def euler(self):
        heading, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        scale = math.pi / (16 * 180)
        return heading * scale, roll * scale, pitch * scale

    def yaw_rate(self):
        yaw_rate, = self._read_reg(BNO055.reg.GYRO_DATA_Z)
        return yaw_rate * math.pi / (16 * 180)

    def read_chip_id(self):
        buffer = bytearray(1)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x00)
        return buffer[0]

    def operating_mode_configuration(self):
        self._i2c.mem_write(BNO055.CONFIG_MODE, self.DEV_ADDR, 0x3D)

    def operating_mode_imuplus(self):
        self._i2c.mem_write(BNO055.IMUPLUS_MODE, self.DEV_ADDR, 0x3D)

    def wait_for_mode(self):
        sleep(0.025)

    def configure_degree_units(self):
        self._i2c.mem_write(0x00, self.DEV_ADDR, 0x3B)

    def calibration_status(self):
        buffer = bytearray(1)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x35)
        return buffer[0]

    def calibration_profile_read(self):
        buffer = bytearray(22)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x55)
        return buffer

    def calibration_profile_write(self, calibration):
        if len(calibration) != 22:
            raise ValueError("Calibration profile must contain 22 bytes")

        self._i2c.mem_write(bytearray(calibration), self.DEV_ADDR, 0x55)
