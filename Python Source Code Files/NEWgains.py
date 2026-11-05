"""Validation and JSON storage for the Romi velocity and line PI gains."""

import json


REQUIRED_KEYS = (
    "velocity_kp",
    "velocity_ki",
    "line_kp",
    "line_ki",
)


def validate_gains(gains):
    """Return a normalized gain dictionary or raise ``ValueError``."""
    if not isinstance(gains, dict) or set(gains) != set(REQUIRED_KEYS):
        raise ValueError("Gains must contain exactly the four required values")

    normalized = {}
    for name in REQUIRED_KEYS:
        value = gains[name]
        if isinstance(value, bool):
            raise ValueError("{} must be a non-negative finite number".format(name))

        try:
            value = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError("{} must be numeric".format(name)) from error

        if value < 0 or value != value or value in (float("inf"), -float("inf")):
            raise ValueError("{} must be a non-negative finite number".format(name))

        normalized[name] = value

    return normalized


def save_gains(filename, gains):
    """Validate and save gains to *filename*."""
    gains = validate_gains(gains)
    with open(filename, "w") as file:
        json.dump(gains, file)


def load_gains(filename):
    """Load and validate gains from *filename*."""
    with open(filename, "r") as file:
        return validate_gains(json.load(file))
