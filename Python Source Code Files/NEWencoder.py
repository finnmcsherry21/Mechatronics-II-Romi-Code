
from time import ticks_us, ticks_diff

from math import pi

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''


    def __init__(self, tim, direction=1):
        '''Initializes an Encoder object'''

        self.tim = tim
        self.direction = direction
        self.position = 0
        self.prev_count = self.tim.counter()
        self.delta = 0
        self.dt = 0
        self.speed = 0
        self.prev_time = ticks_us()

    
    def update(self):
        '''Updates encoder position and wheel speed.'''

        AR = 65535
        # Read the current hardware count from this encoder's timer
        current_count = self.tim.counter()

        # Find how many encoder counts occurred since the previous update
        self.delta = current_count - self.prev_count

        # Correct a timer overflow: 65535 -> 0
        if self.delta < -(AR + 1) // 2:
            self.delta += AR + 1

        # Correct a timer underflow: 0 -> 65535
        elif self.delta > (AR + 1) // 2:
            self.delta -= AR + 1

        # Find elapsed real time in seconds
        current_time = ticks_us()
        self.dt = ticks_diff(current_time, self.prev_time) / 1_000_000

        # Calculate wheel speed in encoder counts per second
        if self.dt > 0:
            self.speed = self.delta / (self.dt)
        else:
            self.speed = 0

        # Accumulate total encoder position in counts
        self.position += self.delta

        # Save values for the next update
        self.prev_count = current_count
        self.prev_time = current_time

            
    def get_position(self):
        return self.direction * self.position * (2 * pi / (12*120))

            
    def get_velocity(self):
        velocity = self.direction * self.speed * (2 * pi / (12*120))
        return self.direction * self.speed * (2 * pi / (12*120))

    
    def zero(self):
        self.position = 0
        self.delta = 0
        self.speed = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
        
        

