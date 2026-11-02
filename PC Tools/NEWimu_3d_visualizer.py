"""Show the Romi orientation streamed by NEWtaskuser.py over USB serial."""

from serial import Serial, SerialException
from vpython import box, compound, cross, cylinder, rate, rotate, scene, vector


SERIAL_PORT = "COM10"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.05


def make_romi():
    parts = [
        box(pos=vector(0, 0, 0), size=vector(0.18, 0.035, 0.14), color=vector(0.15, 0.55, 0.85)),
        box(pos=vector(0.02, 0.03, 0), size=vector(0.10, 0.025, 0.10), color=vector(0.95, 0.95, 0.95)),
        cylinder(pos=vector(-0.055, 0, -0.085), axis=vector(0, 0, 0.025), radius=0.037, color=vector(0.12, 0.12, 0.12)),
        cylinder(pos=vector(0.055, 0, -0.085), axis=vector(0, 0, 0.025), radius=0.037, color=vector(0.12, 0.12, 0.12)),
        cylinder(pos=vector(-0.055, 0, 0.06), axis=vector(0, 0, 0.025), radius=0.037, color=vector(0.12, 0.12, 0.12)),
        cylinder(pos=vector(0.055, 0, 0.06), axis=vector(0, 0, 0.025), radius=0.037, color=vector(0.12, 0.12, 0.12)),
    ]
    return compound(parts)


def update_orientation(romi, heading, roll, pitch):
    forward = vector(1, 0, 0)
    up = vector(0, 1, 0)

    forward = rotate(forward, angle=heading, axis=vector(0, 1, 0))
    up = rotate(up, angle=heading, axis=vector(0, 1, 0))

    left = cross(up, forward)
    forward = rotate(forward, angle=pitch, axis=left)
    up = rotate(up, angle=pitch, axis=left)
    up = rotate(up, angle=roll, axis=forward)

    romi.axis = forward
    romi.up = up


def read_orientation(serial_port):
    raw_line = serial_port.readline()
    if not raw_line:
        return None

    fields = raw_line.decode("utf-8", errors="replace").strip().split(",")
    if len(fields) != 4 or fields[0] != "IMU":
        return None

    try:
        return float(fields[1]), float(fields[2]), float(fields[3])
    except ValueError:
        return None


def main():
    scene.title = "Romi IMU 3D Viewer"
    scene.caption = "Press Ctrl-C in the terminal to stop streaming.\n"
    scene.width = 900
    scene.height = 650
    scene.background = vector(0.92, 0.92, 0.92)
    romi = make_romi()

    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as serial_port:
            serial_port.reset_input_buffer()
            serial_port.write(b"t")

            try:
                while True:
                    rate(50)
                    orientation = read_orientation(serial_port)
                    if orientation is not None:
                        update_orientation(romi, *orientation)

            except KeyboardInterrupt:
                pass

            finally:
                serial_port.write(b"e")

    except SerialException as error:
        raise SystemExit("Serial-port error: {}".format(error))


if __name__ == "__main__":
    main()
