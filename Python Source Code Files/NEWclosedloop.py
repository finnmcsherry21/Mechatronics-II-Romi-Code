

class ClosedLoop:
    # PI velocity controller with voltage saturation and anti-windup

    def __init__(self, k_p=0.0, k_i=0.0, min_voltage=-7.2, max_voltage=7.2):
        self.min_voltage = float(min_voltage)
        self.max_voltage = float(max_voltage)

        if self.min_voltage >= self.max_voltage:
            raise ValueError("min_voltage must be less than max_voltage")

        self.k_p = 0.0
        self.k_i = 0.0
        self.integral_error = 0.0
        self.set_gains(k_p, k_i)


    def set_gains(self, k_p, k_i):
        print(k_p, k_i)

        k_p = float(k_p)
        k_i = float(k_i)

        if k_p < 0:
            raise ValueError("k_p must be non-negative")

        if k_i < 0:
            raise ValueError("k_i must be non-negative")

        self.k_p = k_p
        self.k_i = k_i

        # Do not retain integral error from an old set of gains.
        self.integral_error = 0.0


    def reset(self):
        self.integral_error = 0.0


    def update(self, setpoint, measurement, dt):
        # setpoint: desired wheel velocity in rad/s
        # measurement: measured wheel velocity in rad/s
        # dt: elapsed time since the last controller update in seconds
        
        error = float(setpoint) - float(measurement)
        dt = float(dt)

        # P action: voltage caused immediately by current velocity error.
        proportional_voltage = self.k_p * error

        candidate_integral_error = self.integral_error + error * dt
        candidate_integral_voltage = self.k_i * candidate_integral_error
        candidate_voltage = proportional_voltage + candidate_integral_voltage

        # Anti-windup
        if candidate_voltage > self.max_voltage:
            allow_integration = error < 0

        elif candidate_voltage < self.min_voltage:
            allow_integration = error > 0

        else:
            allow_integration = True

        if allow_integration:
            self.integral_error = candidate_integral_error

        integral_voltage = self.k_i * self.integral_error
        requested_voltage = proportional_voltage + integral_voltage

        # Enforce the Romi motor-voltage limits.
        if requested_voltage > self.max_voltage:
            output_voltage = self.max_voltage

        elif requested_voltage < self.min_voltage:
            output_voltage = self.min_voltage

        else:
            output_voltage = requested_voltage

        return output_voltage