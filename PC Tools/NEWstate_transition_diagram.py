from matplotlib import pyplot


figure, axes = pyplot.subplots(
    4, 1, figsize=(14, 26),
    gridspec_kw={"height_ratios": [12, 5, 3, 3]}
)

user_axis = axes[0]
user_axis.set_title("TaskUser State Transitions", fontsize=16)
user_axis.set_xlim(0, 12)
user_axis.set_ylim(0, 12.5)
user_axis.axis("off")

user_flows = [
    (11, ["MENU", "SETPOINT", "MENU"], ["V", "Enter/cancel"]),
    (10, ["MENU", "GAINS", "MENU"], ["K", "Complete/cancel"]),
    (9, ["MENU", "LEFT TEST", "MOTOR DATA", "MENU"], ["L", "Done/E", "End of data"]),
    (8, ["MENU", "RIGHT TEST", "MOTOR DATA", "MENU"], ["R", "Done/E", "End of data"]),
    (7, ["MENU", "LINE CALIBRATION", "MENU"], ["C", "B, W/E"]),
    (6, ["MENU", "IMU CALIBRATION", "MENU"], ["I", "3/3/E"]),
    (5, ["MENU", "LINE FOLLOWING", "LANE DIRECTION", "LINE FOLLOWING"], ["D", "S", "L/R/E"]),
    (4, ["LINE FOLLOWING", "LINE DATA", "MENU"], ["E", "End of data"]),
    (3, ["LINE FOLLOWING", "MENU"], ["Controller stops"]),
    (2, ["MENU", "MANUAL", "MENU"], ["M", "E"]),
    (1, ["MENU", "LINE DATA", "MENU"], ["E", "End of data"]),
]

for y_position, states, labels in user_flows:
    x_positions = [1.2, 4.2, 7.2, 10.2]

    for index in range(len(states)):
        user_axis.text(
            x_positions[index], y_position, states[index],
            ha="center", va="center", fontsize=9,
            bbox={"boxstyle": "round", "facecolor": "lightblue"}
        )

    for index in range(len(labels)):
        user_axis.annotate(
            "",
            xy=(x_positions[index + 1] - 0.55, y_position),
            xytext=(x_positions[index] + 0.55, y_position),
            arrowprops={"arrowstyle": "->", "color": "black"}
        )
        user_axis.text(
            (x_positions[index] + x_positions[index + 1]) / 2,
            y_position + 0.28,
            labels[index],
            ha="center", va="bottom", fontsize=8
        )

lane_axis = axes[1]
lane_axis.set_title("LaneSwitchController State Transitions", fontsize=16)
lane_axis.set_xlim(0, 12)
lane_axis.set_ylim(0, 7)
lane_axis.axis("off")

lane_flows = [
    (5, ["INACTIVE", "DEPARTURE", "RETURN", "INACTIVE"],
     ["Start: L/R", "Departure line lost", "Target line: 3 detections\nat least 0.10 m away"]),
    (3.5, ["DEPARTURE", "INACTIVE"], ["8 s timeout: stop motors"]),
    (2, ["RETURN", "INACTIVE"], ["8 s timeout: stop motors"]),
]

for y_position, states, labels in lane_flows:
    x_positions = [1.5, 4.5, 7.5, 10.5]

    for index in range(len(states)):
        lane_axis.text(
            x_positions[index], y_position, states[index],
            ha="center", va="center", fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "lightgreen"}
        )

    for index in range(len(labels)):
        lane_axis.annotate(
            "",
            xy=(x_positions[index + 1] - 0.6, y_position),
            xytext=(x_positions[index] + 0.6, y_position),
            arrowprops={"arrowstyle": "->", "color": "black"}
        )
        lane_axis.text(
            (x_positions[index] + x_positions[index + 1]) / 2,
            y_position + 0.28,
            labels[index],
            ha="center", va="bottom", fontsize=8
        )

sensor_axis = axes[2]
sensor_axis.set_title("LineSensorArray Calibration Stages", fontsize=16)
sensor_axis.set_xlim(0, 12)
sensor_axis.set_ylim(0, 4)
sensor_axis.axis("off")

sensor_flows = [
    (2.8, ["IDLE", "BLACK", "WHITE", "IDLE"], ["C", "B", "W: save"]),
    (1.2, ["BLACK or WHITE", "IDLE"], ["E: cancel"]),
]

for y_position, states, labels in sensor_flows:
    x_positions = [1.5, 4.5, 7.5, 10.5]

    for index in range(len(states)):
        sensor_axis.text(
            x_positions[index], y_position, states[index],
            ha="center", va="center", fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "khaki"}
        )

    for index in range(len(labels)):
        sensor_axis.annotate(
            "",
            xy=(x_positions[index + 1] - 0.6, y_position),
            xytext=(x_positions[index] + 0.6, y_position),
            arrowprops={"arrowstyle": "->", "color": "black"}
        )
        sensor_axis.text(
            (x_positions[index] + x_positions[index + 1]) / 2,
            y_position + 0.28,
            labels[index],
            ha="center", va="bottom", fontsize=8
        )

motor_axis = axes[3]
motor_axis.set_title("MotorControlTask Lifecycle", fontsize=16)
motor_axis.set_xlim(0, 12)
motor_axis.set_ylim(0, 3)
motor_axis.axis("off")

motor_states = ["STOPPED", "RUNNING", "STOPPED"]
motor_labels = ["start()", "Test done/E/line stop"]
motor_positions = [1.5, 6, 10.5]

for index in range(len(motor_states)):
    motor_axis.text(
        motor_positions[index], 1.5, motor_states[index],
        ha="center", va="center", fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "plum"}
    )

for index in range(len(motor_labels)):
    motor_axis.annotate(
        "",
        xy=(motor_positions[index + 1] - 0.6, 1.5),
        xytext=(motor_positions[index] + 0.6, 1.5),
        arrowprops={"arrowstyle": "->", "color": "black"}
    )
    motor_axis.text(
        (motor_positions[index] + motor_positions[index + 1]) / 2,
        1.8, motor_labels[index],
        ha="center", va="bottom", fontsize=8
    )

pyplot.tight_layout()
pyplot.savefig("PC Tools/NEWstate_transition_diagram.png", dpi=200)
