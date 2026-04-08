# =========================================================
# PROGRAMMA VPC TEKENROBOT
# =========================================================
# Dit programma bestuurt een robotarm met:
# - schouder
# - elleboog
# - penmechanisme
#
# Op het OLED-scherm verschijnt tekst.
# Met knoppen kan elke servo bewogen worden.


# =========================================================
# BIBLIOTHEKEN
# =========================================================
from machine import Pin, PWM, I2C
# Pin → knoppen en digitale signalen
# PWM → servo aansturen
# I2C → communicatie met OLED-scherm

import time
# time → pauzes in het programma

import framebuf
# framebuf → beeld in geheugen opbouwen voor OLED

import math
# Nodig voor goniometrische berekeningen


# =========================================================
# OLED INSTELLING
# =========================================================
ADDR = 0x3C
i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=20000)


class SSD1306_I2C:

    def __init__(self, width, height, i2c, addr=0x3C):
        self.width = width
        self.height = height
        self.i2c = i2c
        self.addr = addr
        self.pages = self.height // 8

        self.buffer = bytearray(self.pages * self.width)

        self.fb = framebuf.FrameBuffer(
            self.buffer,
            self.width,
            self.height,
            framebuf.MONO_VLSB
        )

        self.init_display()

    def writeto_retry(self, data, tries=8, delay_ms=5):
        for _ in range(tries):
            try:
                self.i2c.writeto(self.addr, data)
                return
            except OSError:
                time.sleep_ms(delay_ms)

        # Laatste poging
        self.i2c.writeto(self.addr, data)

    def cmd(self, c):
        self.writeto_retry(bytes([0x00, c]))
        time.sleep_ms(2)

    def init_display(self):
        for c in (
            0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
            0x8D, 0x14, 0x20, 0x00,
            #0xA0,  # horizontaal omdraaien
            #0xC0,  # verticaal omdraaien
            0xA1,  # 180° horizontaal
            0xC8,  # 180° verticaal
            0xDA, 0x12,
            0x81, 0xCF, 0xD9, 0xF1,
            0xDB, 0x40, 0xA4, 0xA6, 0xAF
        ):
            self.cmd(c)

        self.fill(0)
        self.show()

    def fill(self, col):
        self.fb.fill(col)

    def text(self, s, x, y):
        self.fb.text(s, x, y)

    def show(self):
        self.cmd(0x21)
        self.cmd(0)
        self.cmd(self.width - 1)
        self.cmd(0x22)
        self.cmd(0)
        self.cmd(self.pages - 1)

        # De buffer in kleine stukken versturen
        # Dit is stabieler dan alles in één keer sturen
        chunk = 32
        for i in range(0, len(self.buffer), chunk):
            self.writeto_retry(b"\x40" + self.buffer[i:i + chunk])
            time.sleep_ms(1)


oled = SSD1306_I2C(128, 64, i2c, addr=ADDR)


def oled_message(l1="", l2="", l3=""):
    oled.fill(0)
    oled.text(l1, 0, 0)
    oled.text(l2, 0, 16)
    oled.text(l3, 0, 32)
    oled.show()


# =========================================================
# KNOPPEN
# =========================================================
button_schouder = Pin(13, Pin.IN, Pin.PULL_DOWN)
button_elleboog = Pin(14, Pin.IN, Pin.PULL_DOWN)
button_pen = Pin(15, Pin.IN, Pin.PULL_DOWN)


# =========================================================
# SERVO'S
# =========================================================
servo_schouder = PWM(Pin(18))
servo_schouder.freq(50)

servo_elleboog = PWM(Pin(19))
servo_elleboog.freq(50)

servo_pen = PWM(Pin(20))
servo_pen.freq(50)


# =========================================================
# SERVO BEREIK
# =========================================================
# Deze waarden bepalen welk PWM-bereik hoort bij
# ongeveer 0 graden en 180 graden.
MIN_DUTY = 1638
MAX_DUTY = 8192


def angle_to_duty(angle):
    return int(MIN_DUTY + (angle / 180) * (MAX_DUTY - MIN_DUTY))


def set_servo_angle(servo, angle):
    servo.duty_u16(angle_to_duty(angle))

# =========================================================
# Functies die xy-coördinaten omzetten naar servohoeken
# =========================================================
def bereken_ellebooghoek(x, y):
    """
    Berekent de hoek van de elleboogservo (∠E)
    op basis van het punt (x, y).
    """
    waarde = (100 - x**2 - y**2) / 96
    hoek_E = 180 - math.degrees(math.acos(waarde))
    return hoek_E

def bereken_schouderhoek(x, y):
    """
    Berekent de hoek van de schouderservo (∠S)
    op basis van het punt (x, y).
    """
    L = math.sqrt(x**2 + y**2)

    waarde_a = (x**2 + y**2 + 28) / (16 * L)
    hoek_a = math.degrees(math.acos(waarde_a))

    waarde_b = x / L
    hoek_b = math.degrees(math.acos(waarde_b))

    hoek_S = hoek_a + hoek_b
    return hoek_S


# =========================================================
# OPSTARTTEST
# =========================================================
oled_message("VPC Tekenrobot", "Opstarten", "")
set_servo_angle(servo_pen, 0)
servoSchouderStart = 90
servoElleboogStart = 90
set_servo_angle(servo_schouder, servoSchouderStart)
time.sleep(2)
set_servo_angle(servo_elleboog, servoElleboogStart)
time.sleep(2)
set_servo_angle(servo_pen, 90)

oled_message("VPC Tekenrobot", f"Schouder: {servoSchouderStart}", f"Elleboog: {servoElleboogStart}")

x = 9
y = 10

schouderhoek = bereken_schouderhoek(x, y)
ellebooghoek = bereken_ellebooghoek(x, y)

print("Schouder:", schouderhoek)
print("Elleboog:", ellebooghoek)

set_servo_angle(servo_schouder, schouderhoek)
time.sleep(1)
set_servo_angle(servo_elleboog,ellebooghoek)
oled_message(f"x: {x}  y: {y}", f"Schouder: {schouderhoek}", f"Elleboog: {ellebooghoek}")


# =========================================================
# HOOFDLUS
# =========================================================
while True:

    if button_schouder.value():

        oled_message("Servo", "Schouder", "")

        set_servo_angle(servo_schouder, 90 + 15)
        time.sleep(0.5)
        
        set_servo_angle(servo_schouder, 90)
        time.sleep(0.5)

        set_servo_angle(servo_schouder, 90 - 15)
        time.sleep(0.5)

        set_servo_angle(servo_schouder, 90)
        time.sleep(0.5)

        oled_message("VPC Tekenrobot", "Gereed", "Druk knop")

        while button_schouder.value():
            time.sleep(0.01)

    if button_elleboog.value():

        oled_message("Servo", "Elleboog", "")

        set_servo_angle(servo_elleboog, 90 + 15)
        time.sleep(0.5)

        set_servo_angle(servo_elleboog, 90)
        time.sleep(0.5)
        
        set_servo_angle(servo_elleboog, 90 - 15)
        time.sleep(0.5)

        set_servo_angle(servo_elleboog, 90)
        time.sleep(0.5)

        oled_message("VPC Tekenrobot", "Gereed", "Druk knop")

        while button_elleboog.value():
            time.sleep(0.01)

    if button_pen.value():

        oled_message("Servo", "Pen", "")

        set_servo_angle(servo_pen, 90 + 45)
        time.sleep(0.5)

        set_servo_angle(servo_pen, 90)
        time.sleep(0.5)


        oled_message("VPC Tekenrobot", "Gereed", "Druk knop")

        while button_pen.value():
            time.sleep(0.01)

    time.sleep(0.01)