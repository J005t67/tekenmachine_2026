import math

from machine import Pin, PWM

# Deze waarden bepalen welk PWM-bereik hoort bij
# ongeveer 0 graden en 180 graden.
MIN_DUTY = 1670
MAX_DUTY = 8024

class Servo:
    
    def __init__(self, pin_nr):
        self.pin_nr = pin_nr
        self.pwm = PWM(Pin(self.pin_nr))
        self.pwm.freq(50)
                
    def __angle_to_duty(self, angle):
        duty = int(MIN_DUTY + (angle / 180) * (MAX_DUTY - MIN_DUTY))
        return duty

    def set_angle_degrees(self, angle_degrees):
        self.pwm.duty_u16(self.__angle_to_duty(angle_degrees))
        
    def set_angle_radians(self, angle_radians):
        self.set_angle_degrees(math.degrees(angle_radians))       
        