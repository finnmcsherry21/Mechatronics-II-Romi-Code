from time import sleep

from micropython import const
from struct import calcsize, unpack_from
import math

class BNO055:
    DEV_ADDR = const(0x28)
    

    class reg:
        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")
        GYR_Z = (const(0x18), b"<h")

    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22)))
        sleep(1) # Startup time for BNO055

    def _read_reg(self, reg):
        # Determine number of bytes to read
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)


    def euler(self):
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        head = head * (2 * math.pi / 360)  # Convert to radians
        roll = roll * (2 * math.pi / 360)  # Convert to radians
        pitch = pitch * (2 * math.pi / 360)  # Convert to radians
        return (head / 16, roll / 16, pitch / 16)


    def read_chip_id(self):      
        buffer = bytearray(1)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x00)
        return buffer[0]
    
    
    def operating_mode_configuration(self):
        # Parameters: Data, device address, register address (BNO055 datasheet 4.2)
        self._i2c.mem_write(0x00, self.DEV_ADDR, 0x3D)
        print("BNO055 operating mode set to CONFIG")
        sleep(0.025)
        
    def operating_mode_normal(self):
        # Parameters: Data, device address, register address (BNO055 datasheet 4.2)
        self._i2c.mem_write(0x0C, self.DEV_ADDR, 0x3D)
        print("BNO055 operating mode set back to normal")
        sleep(0.025)
        
    def yaw_rate(self):
        # The BNO055 gyro output is 16 LSB per degree/second at the
        # default 2000 degree/second range.
        gyro_z = self._read_reg(BNO055.reg.GYR_Z)[0]
        gyro_z = gyro_z / 16  * (math.pi / 180)  # Convert to radians
        return gyro_z
        
        
        
       
        
    def heading(self):
        heading_raw = self._read_reg(BNO055.reg.EUL_HEADING)[0]
        return heading_raw * math.pi / (16 * 180)
    
    
    def gyro_component(self):
        return self.yaw_rate()
    
    
    def calibration_status(self):
        buffer = bytearray(1)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x35)
        return buffer[0]
    
    
    def calibration_profile_read(self):
        buffer = bytearray(0x6A - 0x55 + 1)
        self._i2c.mem_read(buffer, self.DEV_ADDR, 0x55)
        #self._i2c.mem_write(buffer, self.DEV_ADDR, 0x55)
        return buffer
    
    def calibration_profile_write(self, calibration):
        if len(calibration) != 0x6A - 0x55 + 1:
            raise ValueError("Calibration profile must contain 22 bytes")

        buffer = bytearray(calibration)
        self._i2c.mem_write(buffer, self.DEV_ADDR, 0x55)
