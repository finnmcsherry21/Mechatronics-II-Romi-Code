from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


SERIAL_PORT = "COM10"
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.25
MOTOR_COMMANDS = ("l", "r")
VELOCITY = 7.5
VELOCITY_KP = 0.25
VELOCITY_KI = 0.05
LINE_KP = 0.25
LINE_KI = 0.05
NUMERIC_LINE_ENDING = "\r\n"
END_MARKER = "End of data"
EXPECTED_HEADERS = (
    "Time(s)",
    "Position(rad)",
    "Velocity(rad/s)",
    "Input Voltage(V)",
    "Setpoint(rad/s)",
)
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0



def cleaned_fields(line):
    content = line.split("#", 1)[0].strip()

    if not content:
        return []

    return [field.strip() for field in content.split(",")]


def collect_dataset(serial_port):
    headers = None
    columns = None
    waiting_since = monotonic()

    while True:
        raw_line = serial_port.readline()

        if not raw_line:
            timeout = FIRST_RESPONSE_TIMEOUT if headers is None else BETWEEN_LINES_TIMEOUT
            if monotonic() - waiting_since >= timeout:
                if headers is None:
                    raise TimeoutError("No motor-response header was received.")
                raise TimeoutError("Motor response ended before End of data.")
            continue

        waiting_since = monotonic()
        line = raw_line.decode("utf-8", errors="replace").strip()

        if line == END_MARKER:
            break

        fields = cleaned_fields(line)

        if not fields:
            continue

        if headers is None:
            if tuple(fields) == EXPECTED_HEADERS:
                headers = fields
                columns = [[] for _ in headers]
            continue

        if len(fields) < len(headers):
            continue

        try:
            values = [float(field) for field in fields[:len(headers)]]
        except ValueError:
            continue

        for column, value in zip(columns, values):
            column.append(value)

    if headers is None or not columns[0]:
        raise ValueError("No valid motor-response data was received.")

    return headers, columns


def save_response_plot(motor_name, headers, columns):
    figure, axes = pyplot.subplots(2, 1, sharex=True)

    axes[0].plot(columns[0], columns[2], label=headers[2])
    axes[0].plot(columns[0], columns[4], label=headers[4])
    axes[0].set_ylabel("Velocity (rad/s)")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(columns[0], columns[3], label=headers[3])
    axes[1].set_xlabel(headers[0])
    axes[1].set_ylabel(headers[3])
    axes[1].grid(True)

    figure.suptitle("{} Motor Response".format(motor_name.title()))
    figure.tight_layout()
    filename = "latest_{}_motor_response.png".format(motor_name)
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)
    print("Updated {}.".format(filename))


def send_setup(serial_port):
    serial_port.write(b"k")
    for value in (VELOCITY_KP, VELOCITY_KI, LINE_KP, LINE_KI):
        serial_port.write((str(value) + NUMERIC_LINE_ENDING).encode("utf-8"))
        print(value)

    serial_port.write(b"v")
    serial_port.write((str(VELOCITY) + NUMERIC_LINE_ENDING).encode("utf-8"))


def main():
    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as serial_port:
            serial_port.reset_input_buffer()

            for command in MOTOR_COMMANDS:
                motor_name = "left" if command == "l" else "right"
                send_setup(serial_port)
                command = command + NUMERIC_LINE_ENDING
                serial_port.write(command.encode("utf-8"))
                print(command)
                headers, columns = collect_dataset(serial_port)
                print(headers, columns)
                save_response_plot(motor_name, headers, columns)

    except SerialException as error:
        raise SystemExit("Serial-port error: {}".format(error))


if __name__ == "__main__":
    main()
