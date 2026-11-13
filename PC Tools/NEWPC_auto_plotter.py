from math import isfinite
from pathlib import Path
from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


SERIAL_PORT = "COM10"
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.25
MOTOR_COMMANDS = ("l", "r")
RUN_STRAIGHT_DRIVE = True
STRAIGHT_DRIVE_DURATION = 5.0
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


def collect_dataset(serial_port, clock=monotonic):
    """Read one complete motor-test response from ``serial_port``.

    Non-data console messages are ignored, but a response must contain the
    expected header, at least one complete finite numeric row, and its end
    marker.  ``clock`` is injectable so the timeout behavior is testable
    without waiting for a real serial port.
    """
    headers = None
    columns = None
    waiting_since = clock()

    while True:
        raw_line = serial_port.readline()

        if not raw_line:
            timeout = FIRST_RESPONSE_TIMEOUT if headers is None else BETWEEN_LINES_TIMEOUT
            if clock() - waiting_since >= timeout:
                if headers is None:
                    raise TimeoutError("No motor-response header was received.")
                raise TimeoutError("Motor response ended before End of data.")
            continue

        waiting_since = clock()
        line = raw_line.decode("utf-8", errors="replace").strip()
        fields = cleaned_fields(line)

        if not fields:
            continue

        # Parse the marker like every other serial line, so trailing comments
        # or incidental whitespace do not cause a false timeout.
        if fields == [END_MARKER]:
            break

        if headers is None:
            if tuple(fields) == EXPECTED_HEADERS:
                headers = list(EXPECTED_HEADERS)
                columns = [[] for _ in headers]
            continue

        if len(fields) != len(headers):
            continue

        try:
            values = [float(field) for field in fields]
        except ValueError:
            continue

        if not all(isfinite(value) for value in values):
            continue

        for column, value in zip(columns, values):
            column.append(value)

    if headers is None or not columns[0]:
        raise ValueError("No valid motor-response data was received.")

    return headers, columns


def validate_dataset(headers, columns):
    """Raise ``ValueError`` unless *columns* match the motor-test protocol."""
    if tuple(headers) != EXPECTED_HEADERS:
        raise ValueError("Unexpected motor-response headers.")

    if len(columns) != len(headers) or not columns or not columns[0]:
        raise ValueError("No valid motor-response data was received.")

    sample_count = len(columns[0])
    if any(len(column) != sample_count for column in columns):
        raise ValueError("Motor-response columns have different lengths.")

    if not all(isfinite(value) for column in columns for value in column):
        raise ValueError("Motor-response data contains a non-finite value.")


def save_response_plot(motor_name, headers, columns, output_directory="."):
    """Save a velocity/voltage plot and return its path."""
    validate_dataset(headers, columns)
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
    filename = Path(output_directory) / "latest_{}_motor_response.png".format(motor_name)
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)
    print("Updated {}.".format(filename))
    return filename


def send_setup(serial_port):
    serial_port.write(b"k")
    for value in (VELOCITY_KP, VELOCITY_KI, LINE_KP, LINE_KI):
        serial_port.write((str(value) + NUMERIC_LINE_ENDING).encode("utf-8"))
        print(value)

    serial_port.write(b"v")
    serial_port.write((str(VELOCITY) + NUMERIC_LINE_ENDING).encode("utf-8"))


def run_motor_tests(serial_port):
    """Configure the Romi and collect plots for every requested motor."""
    for command in MOTOR_COMMANDS:
        motor_name = "left" if command == "l" else "right"
        send_setup(serial_port)
        serial_port.write(command.encode("ascii"))
        print("Starting {} motor test.".format(motor_name))
        headers, columns = collect_dataset(serial_port)
        save_response_plot(motor_name, headers, columns)


def run_straight_drive(serial_port, duration=STRAIGHT_DRIVE_DURATION):
    """Configure the Romi, run ``u``, and plot its printed motor response."""
    send_setup(serial_port)
    serial_port.write(b"u")
    serial_port.write((str(duration) + NUMERIC_LINE_ENDING).encode("utf-8"))
    print("Starting straight drive for {} seconds.".format(duration))
    headers, columns = collect_dataset(serial_port)
    return save_response_plot("straight", headers, columns)


def main():
    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as serial_port:
            serial_port.reset_input_buffer()
            if RUN_STRAIGHT_DRIVE:
                run_straight_drive(serial_port)
            else:
                run_motor_tests(serial_port)

    except SerialException as error:
        raise SystemExit("Serial-port error: {}".format(error))


if __name__ == "__main__":
    main()
