

from machine import PWM, Pin
from ulab import numpy as np

class servo_class:
    # =========================================================
    # SERVO BEREIK
    # =========================================================
    # Deze waarden bepalen welk PWM-bereik hoort bij
    # ongeveer 0 graden en 180 graden.
    MIN_DUTY = 1670
    MAX_DUTY = 8024

    def __init__(self, ID, Pin_num, frequency, xp, fp, offset):
        self.ID = ID
        self.PWM = PWM(Pin(Pin_num))
        self.freq = self.PWM.freq(frequency)
        self.xp = xp
        self.fp = fp
        self.offset = offset  # graden

    def angle_to_duty(self, angle):
        duty = int(self.MIN_DUTY + (angle / 180.) * (self.MAX_DUTY - self.MIN_DUTY))
        return duty

    def duty_to_angle(self, duty):
        return (duty - self.MIN_DUTY) / (self.MAX_DUTY - self.MIN_DUTY) * 180.

    def set_servo_angle(self, angle):
        self.PWM.duty_u16(self.angle_to_duty(angle))

    def angle_to_servo_angle(self, angle):
        # see xy_calibration.ods
        servo_angle = np.interp(angle, self.fp, self.xp)[0] + self.offset
        # print(f"servo {self.ID} angle: {servo_angle}")
        return servo_angle




