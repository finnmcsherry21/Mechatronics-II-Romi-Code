"""Show the Romi orientation streamed by NEWtaskuser.py over USB serial.

To run:
Close PuTTY

python "PC Tools/NEWimu_3d_visualizer.py"



"""


from serial import Serial, SerialException
from vpython import box, compound, cross, cylinder, label, rate, rotate, scene, vector


SERIAL_PORT = "COM10"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.05
orientation_offset = [0.0, 0.0, 0.0]
latest_orientation = [0.0, 0.0, 0.0]


def reset_orientation(event):
    if event.key.lower() == "r":
        orientation_offset[0] = latest_orientation[0]
        orientation_offset[1] = latest_orientation[1]
        orientation_offset[2] = latest_orientation[2]
        print("IMU viewer orientation reset.")


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


def update_orientation(romi, front_label, heading, roll, pitch):
    heading -= orientation_offset[0]
    roll -= orientation_offset[1]
    pitch -= orientation_offset[2]

    forward = vector(1, 0, 0)
    up = vector(0, 1, 0)

    forward = rotate(forward, angle=-heading, axis=vector(0, 1, 0))
    up = rotate(up, angle=-heading, axis=vector(0, 1, 0))

    left = cross(up, forward)
    forward = rotate(forward, angle=pitch, axis=left)
    up = rotate(up, angle=pitch, axis=left)
    up = rotate(up, angle=roll, axis=forward)

    romi.axis = forward
    romi.up = up
    front_label.pos = romi.pos - forward * 0.12 + up * 0.06


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
    scene.caption = "Press R here to set the current Romi pose as level and forward. Press Ctrl-C in the terminal to stop streaming.\n"
    scene.width = 900
    scene.height = 650
    scene.background = vector(0.92, 0.92, 0.92)
    scene.bind("keydown", reset_orientation)
    romi = make_romi()
    front_label = label(text="FRONT", pos=vector(-0.12, 0.06, 0),
                        height=14, box=False, opacity=0, color=vector(0.8, 0, 0))

    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as serial_port:
            serial_port.reset_input_buffer()
            serial_port.write(b"t")

            try:
                while True:
                    rate(50)
                    orientation = read_orientation(serial_port)
                    if orientation is not None:
                        latest_orientation[0] = orientation[0]
                        latest_orientation[1] = orientation[1]
                        latest_orientation[2] = orientation[2]
                        update_orientation(romi, front_label, *orientation)

            except KeyboardInterrupt:
                pass

            finally:
                serial_port.write(b"e")

    except SerialException as error:
        raise SystemExit("Serial-port error: {}".format(error))


if __name__ == "__main__":
    main()
