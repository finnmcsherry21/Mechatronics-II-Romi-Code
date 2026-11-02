from pyb import Pin, Timer

class MotorDriver:

    def __init__(self, pwm_pin: Pin, dir_pin: Pin, nslp_pin: Pin, tim: Timer, chan: int):

        # Store a copy of each input parameter as an attribute
        self._dir_pin = Pin(dir_pin, mode=Pin.OUT_PP)
        self._nslp_pin = Pin(nslp_pin, mode=Pin.OUT_PP)
        self._pwm_chan = tim.channel(chan, pin=pwm_pin, mode=Timer.PWM, pulse_width_percent=0)
        self.disable()
        
        self.run_left = False
        self.run_right = False
      
                                        
    def enable(self):
        self._nslp_pin.high()
    
        
    def disable(self):
        self._nslp_pin.low()

        
    def set_effort(self, effort):
        effort = max(-100, min(100, effort))
        
        if effort < 0:
            self._dir_pin.high()
            effort = -effort
            
        else:
            self._dir_pin.low()
        self._pwm_chan.pulse_width_percent(effort)

