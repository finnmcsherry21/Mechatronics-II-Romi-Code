
import json



def display_calibration_status(imu):
    """Print and return the BNO055 system, gyro, accel, and magnetometer status.

    Each calibration value ranges from 0 (uncalibrated) to 3 (fully calibrated).
    """
    status = imu.calibration_status()
    system = (status >> 6) & 0x03
    gyro = (status >> 4) & 0x03
    accel = (status >> 2) & 0x03
    mag = status & 0x03
    
    print("\tSYS:{} G:{} A:{} M:{}".format(
        system, gyro, accel, mag
    ))

    return system, gyro, accel, mag


def validate_calibration(calibration):
    # The BNO055 calibration profile is exactly 22 unsigned bytes.
    if not isinstance(calibration, (list, tuple, bytearray)) or len(calibration) != 22:
        raise ValueError("Calibration data must be 22 bytes long")
    for value in calibration:
        if not isinstance(value, int) or not 0 <= value <= 255:
            raise ValueError("Calibration data must contain unsigned bytes")
    return calibration


def save_calibration(filename, calibration):
    calibration = list(calibration)
    validate_calibration(calibration)
    with open(filename, "w") as file:
        json.dump(calibration, file)


def load_calibration(filename):
    # Read and validate; startup code reports errors or repeats calibration.

    with open(filename, "r") as file:
        calibration = json.load(file)

    calibration = list(calibration)
    validate_calibration(calibration)

    return calibration
