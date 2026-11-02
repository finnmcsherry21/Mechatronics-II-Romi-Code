import array
import gc

from pyb import Pin, Timer
import pyb
import time

ser = pyb.USB_VCP()


class TaskLine:

    def __init__(self, imu):
        self.sensor1 = pyb.ADC(Pin.cpu.C2)
        self.sensor2 = pyb.ADC(Pin.cpu.C3)
        self.sensor3 = pyb.ADC(Pin.cpu.A0)
        self.sensor4 = pyb.ADC(Pin.cpu.A1)
        self.sensor5 = pyb.ADC(Pin.cpu.A4)
        self.sensor6 = pyb.ADC(Pin.cpu.B0)
        self.sensor7 = pyb.ADC(Pin.cpu.C1)
        self.sensor8 = pyb.ADC(Pin.cpu.C0)
        self.sensor9 = pyb.ADC(Pin.cpu.A6)
        self.sensor10 = pyb.ADC(Pin.cpu.C4)
        self.sensor11 = pyb.ADC(Pin.cpu.C5)
        self.sensor12 = pyb.ADC(Pin.cpu.A7)
        self.sensor13 = pyb.ADC(Pin.cpu.B1)

        self.sensors = (
            self.sensor1, self.sensor2, self.sensor3, self.sensor4,
            self.sensor5, self.sensor6, self.sensor7, self.sensor8,
            self.sensor9, self.sensor10, self.sensor11, self.sensor12,
            self.sensor13
        )

        self.imu = imu
        self.max_samples = 5
        self.sensor_buffers = tuple(array.array('H', [0] * self.max_samples) for _ in range(len(self.sensors)))
        gc.collect()  # Run garbage collection to free up memory before starting tasks

        self.tim = pyb.Timer(4, freq=2500)

        self.black_values = None
        self.white_values = None
        self.center_sensor = 6.0
        
        
        self.line_detect_threshold = 0.20

        gc.collect()  # Run garbage collection to free up memory before starting tasks
        size = 250
        self.line_data_time = array.array('f', [0] * size)
        self.line_data_centroid = array.array('f', [0] * size)
        self.line_data_y_hat = array.array('f', [0] * size)
        gc.collect()  # Run garbage collection to free up memory before starting tasks
        
        self.line_data_i = 0
        self.line_data_running = False
        self.line_start_time = 0
        #self.outer_controller = OuterController(self.imu)


    def read_sensors(self):     
        pyb.ADC.read_timed_multi(self.sensors, self.sensor_buffers, self.tim)

        sensor_values = [
            sum(buffer) / len(buffer)
            for buffer in self.sensor_buffers
        ]

        return sensor_values


    def calibrate_sensors(self):
        
        
        print("Place the robot on the black surface and press 'B'.")

        black_values = None

        while black_values is None:
            if ser.any():
                char_in = ser.read(1).decode()
                gc.collect()  # Run garbage collection to free up memory before reading sensors
                if char_in in {"b", "B"}:
                    black_values = self.read_sensors()
                    print("Black values:", black_values)
                    print("Place the robot on the white surface and press 'W'.")

        white_values = None

        while white_values is None:
            if ser.any():
                char_in = ser.read(1).decode()

                if char_in in {"w", "W"}:
                    white_values = self.read_sensors()
                    print("White values:", white_values)

        self.black_values = black_values
        self.white_values = white_values

        return black_values, white_values

    
    def start_data_collection(self):
        self.line_data_i = 0
        self.line_data_running = True
        self.line_start_time = time.ticks_us()


    def stop_data_collection(self):
        self.line_data_running = False    


    def normalize_sensor_values(self, sensor_values):
        if self.black_values is None or self.white_values is None:
            return None
        line_values = []

        for index, value in enumerate(sensor_values):
            span = self.black_values[index] - self.white_values[index]

            if span <= 0:
                return None

            normalized_value = (
                value - self.white_values[index]
            ) / span

            normalized_value = max(0.0, min(1.0, normalized_value))
            line_values.append(normalized_value)

        return line_values


    def find_centroid(self, sensor_values):
        total_weight = sum(sensor_values)

        if total_weight < self.line_detect_threshold:
            return None

        weighted_sum = sum(
            index * value
            for index, value in enumerate(sensor_values)
        )

        centroid = weighted_sum / total_weight
        return centroid


    def update(self, outer_controller):
        """Sample the line once and update the shared outer controller.

        This is called from the outer-controller task, ensuring that the
        latest centroid is used in the same 20 ms control cycle. Sampling is
        skipped while stopped, avoiding needless ADC/DMA work at the command
        prompt.
        """
        if not outer_controller.enabled:
            return

        sensor_values = self.read_sensors()
        line_values = self.normalize_sensor_values(sensor_values)

        if line_values is None:
            outer_controller.clear_line_error()
            return

        centroid = self.find_centroid(line_values)
        if centroid is None:
            outer_controller.clear_line_error()
            return

        y_hat = centroid - self.center_sensor
        outer_controller.set_line_error(y_hat)

        if self.line_data_running and self.line_data_i < len(self.line_data_time):
            line_time = time.ticks_diff(time.ticks_us(), self.line_start_time) / 1_000_000
            self.line_data_time[self.line_data_i] = line_time
            self.line_data_centroid[self.line_data_i] = centroid
            self.line_data_y_hat[self.line_data_i] = y_hat
            self.line_data_i += 1

        if all(value > 0.25 for value in line_values):
            print("Solid black detected. Stopping motors.")
            outer_controller.stop()
            self.stop_data_collection()


    def run(self, outer_controller):
        """Compatibility generator for standalone sensor-task experiments."""
        while True:
            self.update(outer_controller)
            yield
