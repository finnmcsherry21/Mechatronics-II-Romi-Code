
from pyb import Pin, Timer, I2C
import time


"""

States:

0. Blue button is pressed --> State 1
1. Line follow (until _ meters)
2. Lane change (direction)
3. Line follow (until all white is detected)
4. Dead reckoning (distance)
5. Rotate (90 degrees CW)
6. Ultrasonic (distance from wall)
7. Line follow (until all black is detected)
8. Rotate (90 degrees CCW)
9. Dead reckoning (distance)
10. Dead reckoning (-distance)
11. Rotate (90 degrees CW)
12. Dead reckoning (distance)
13. Rotate (90 degrees CW)
14. Line follow (until all headings are detected)
15. Dead reckoning (distance)
16. Line follow (until all white is detected)
17. Rotate (90 degrees CW)
18. Dead reckoning (distance)
19. Rotate (720 degrees CW)
20. End

"""



class FinalTrack:
    
    
    
    def __init__(self):
        self.state = 0
        self.button_pin = Pin(Pin.cpu.C13, mode=Pin.IN, pull=Pin.PULL_UP)
        


    def run(self):
        while True:
            if self.state == 0:
                # Wait for blue button to be pressed
                if self.button_pin.value() == 0:
                    self.state = 1
                    print("Blue button pressed. Starting final track sequence.")
                    time.sleep(3)  # Debounce delay
                
            
            if self.state == 1:
                # Line follow (until _ meters)
                
                
                pass
            
            if self.state == 2:
                # Lane change (direction)
                pass
            
            if self.state == 3:
                # Line follow (until all white is detected)
                pass
            
            if self.state == 4:
                # Dead reckoning (distance)
                pass
            
            if self.state == 5:
                # Rotate (90 degrees CW)
                pass
            
            if self.state == 6:
                # Ultrasonic (distance from wall)
                pass
            
            if self.state == 7:
                # Line follow (until all black is detected)
                pass
            
            if self.state == 8:
                # Rotate (90 degrees CCW)
                pass
            
            if self.state == 9:
                # Dead reckoning (distance)
                pass
            
            if self.state == 10:
                # Dead reckoning (-distance)
                pass    
        
            if self.state == 11:
                # Rotate (90 degrees CW)
                pass
            
            if self.state == 12:
                # Dead reckoning (distance)
                pass
            
            if self.state == 13:
                # Rotate (90 degrees CW)
                pass
            
            if self.state == 14:
                # Line follow (until all headings are detected)
                pass
            
            if self.state == 15:
                # Dead reckoning (distance)
                pass
            
            if self.state == 16:
                # Line follow (until all white is detected)
                pass
            
            if self.state == 17:
                # Rotate (90 degrees CW)
                pass
            
            if self.state == 18:
                # Dead reckoning (distance)
                pass
            
            if self.state == 19:
                # Rotate (720 degrees CW)
                pass
            
            if self.state == 20:
                # End
                pass
            
            yield