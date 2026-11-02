import json


IMU_MODE = "IMUPLUS"


def display_calibration_status(imu):
    status = imu.calibration_status()
    gyro = (status >> 4) & 0x03
    accel = (status >> 2) & 0x03
    print("\tGYRO:{} ACCEL:{}".format(gyro, accel))
    return gyro, accel


def validate_calibration(calibration):
    if not isinstance(calibration, (list, tuple, bytearray)) or len(calibration) != 22:
        raise ValueError("Calibration data must be 22 bytes long")

    for value in calibration:
        if not isinstance(value, int) or not 0 <= value <= 255:
            raise ValueError("Calibration data must contain unsigned bytes")

    if not any(calibration):
        raise ValueError("Calibration data is empty; calibrate the IMU first")

    return list(calibration)


def save_calibration(filename, calibration):
    profile = validate_calibration(calibration)
    data = {"mode": IMU_MODE, "profile": profile}

    with open(filename, "w") as file:
        json.dump(data, file)


def load_calibration(filename):
    with open(filename, "r") as file:
        data = json.load(file)

    if not isinstance(data, dict) or data.get("mode") != IMU_MODE:
        raise ValueError("Calibration file must contain an IMUPLUS profile")

    return validate_calibration(data.get("profile"))
