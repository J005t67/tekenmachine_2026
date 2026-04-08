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

import math

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
MIN_DUTY = 1670
MAX_DUTY = 8024

# =========================================================
# OLED INSTELLING
# =========================================================
ADDR = 0x3C
i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)

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
        # OLED 180° draaien (display zit ondersteboven)
        for c in (
                0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
                0x8D, 0x14, 0x20, 0x00,
                0xA1,  # horizontaal omdraaien
                0xC8,  # verticaal omdraaien
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

class Vector2D:
    """A two-dimensional vector with Cartesian coordinates."""
    """https://scipython.com/books/book2/chapter-4-the-core-python-language-ii/examples/a-2d-vector-class/"""

    def __init__(self, x, y):
        self.x, self.y = x, y

    def __str__(self):
        """Human-readable string representation of the vector."""
        return '{:g}i + {:g}j'.format(self.x, self.y)

    def __repr__(self):
        """Unambiguous string representation of the vector."""
        return repr((self.x, self.y))

    def dot(self, other):
        """The scalar (dot) product of self and other. Both must be vectors."""

        if not isinstance(other, Vector2D):
            raise TypeError('Can only take dot product of two Vector2D objects')
        return self.x * other.x + self.y * other.y
    # Alias the __matmul__ method to dot so we can use a @ b as well as a.dot(b).
    __matmul__ = dot

    def __sub__(self, other):
        """Vector subtraction."""
        return Vector2D(self.x - other.x, self.y - other.y)

    sub = __sub__

    def __add__(self, other):
        """Vector addition."""
        return Vector2D(self.x + other.x, self.y + other.y)

    def __mul__(self, scalar):
        """Multiplication of a vector by a scalar."""
        if isinstance(scalar, int) or isinstance(scalar, float):
            return Vector2D(self.x*scalar, self.y*scalar)
        raise NotImplementedError('Can only multiply Vector2D by a scalar')

    def __rmul__(self, scalar):
        """Reflected multiplication so vector * scalar also works."""
        return self.__mul__(scalar)

    def __neg__(self):
        """Negation of the vector (invert through origin.)"""
        return Vector2D(-self.x, -self.y)

    def __truediv__(self, scalar):
        """True division of the vector by a scalar."""
        return Vector2D(self.x / scalar, self.y / scalar)

    def __mod__(self, scalar):
        """One way to implement modulus operation: for each component."""
        return Vector2D(self.x % scalar, self.y % scalar)

    def __abs__(self):
        """Absolute value (magnitude) of the vector."""
        return math.sqrt(self.x**2 + self.y**2)

    norm = __abs__

    def distance_to(self, other):
        """The distance between vectors self and other."""
        return abs(self - other)

    def to_polar(self):
        """Return the vector's components in polar coordinates."""
        return self.__abs__(), math.atan2(self.y, self.x)

    """Volgende methods toegevoegd tbv tekenproject"""

    def vecs2angle(self, other):
        # berekent hoek tussen self en other
        return math.acos(self.dot(other) / (self.__abs__() * other.__abs__()))


"""Hier worden de lengtes van de onder- en bovenarm gedefinieerd"""
Anorm = 80  # lengte bovenarm
Bnorm = 60  # lengte onderarm


def C2AB(C_vector):
    # Construeert de A en B vectoren op basis van vector C
    X = Vector2D(1, 0) # Hulpvector
    Cnorm = C_vector.norm()  # lengte van vector C
    beta = math.acos((Bnorm ** 2 - Anorm ** 2 - Cnorm ** 2) / (-2 * Anorm * Cnorm))  # cosinus regel
    delta = C_vector.vecs2angle(X)
    phi = beta + delta
    # Construct A_vector in Cartesian coordinates from polar coordinates
    A_vector = Vector2D(Anorm * math.cos(phi), Anorm * math.sin(phi))
    B_vector = C_vector.sub(A_vector)
    return A_vector, B_vector

def AB2phigamma(A_vector, B_vector):
    # berekent de hoeken phi en gamma
    X = Vector2D(1, 0)
    phi = A_vector.vecs2angle(X)
    gamma = A_vector.vecs2angle(B_vector)
    return phi, gamma

def angle_to_duty(angle):
    duty = int(MIN_DUTY + (angle / 180) * (MAX_DUTY - MIN_DUTY))
    return duty

def duty_to_angle(duty):
    return (duty - MIN_DUTY) / (MAX_DUTY - MIN_DUTY) * 180

def set_servo_angle(servo, angle):
    servo.duty_u16(angle_to_duty(angle))

def set_xy(x, y):
    C_vector = Vector2D(x, y)
    A_vector, B_vector = C2AB(C_vector)
    phi, gamma = AB2phigamma(A_vector, B_vector)
    set_servo_angle(servo_schouder, math.degrees(phi))
    set_servo_angle(servo_elleboog, math.degrees(gamma))
    time.sleep(0.1)

def pen_up():
    set_servo_angle(servo_pen, 90)

def pen_down():
    set_servo_angle(servo_pen, 90 + 60)

def home():
    pen_up()
    set_xy(60, 80)

oled = SSD1306_I2C(128, 64, i2c, addr=ADDR)

def oled_message(l1="", l2="", l3=""):
    oled.fill(0)
    oled.text(l1, 0, 0)
    oled.text(l2, 0, 16)
    oled.text(l3, 0, 32)
    oled.show()

oled_message("VPC-Tekenrobot", "Gereed", "Druk knop SW1")
home()




# =========================================================
# HOOFDLUS
# =========================================================

# "cirkel" tekenen

while True:
    if button_schouder.value():
        r = 30
        theta_step = 2 * math.pi / 180
        theta = 0
        while theta <= 2 * math.pi:
            x = r * math.cos(theta) + 20
            y = r * math.sin(theta) + 80
            set_xy(x, y)
            if theta == 0:
                time.sleep(1)
                pen_down()
                time.sleep(1)
            time.sleep(0.03)
            theta += theta_step
        time.sleep(1)
        pen_up()
        time.sleep(1)
        home()

        while button_schouder.value():
            time.sleep(0.01)




