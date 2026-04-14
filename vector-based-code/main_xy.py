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
from machine import Pin, I2C
# Pin → knoppen en digitale signalen
# PWM → servo aansturen
# I2C → communicatie met OLED-scherm

import time
# time → pauzes in het programma

import framebuf
from ulab import numpy as np
# import Vector2D
import brachi

# framebuf → beeld in geheugen opbouwen voor OLED


# =========================================================
# KNOPPEN
# =========================================================
button_schouder = Pin(13, Pin.IN, Pin.PULL_DOWN)
button_elleboog = Pin(14, Pin.IN, Pin.PULL_DOWN)
button_pen = Pin(15, Pin.IN, Pin.PULL_DOWN)

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

brachiograph = brachi.brachi_class()

oled = SSD1306_I2C(128, 64, i2c, addr=ADDR)

def oled_message(l1="", l2="", l3=""):
    oled.fill(0)
    oled.text(l1, 0, 0)
    oled.text(l2, 0, 16)
    oled.text(l3, 0, 32)
    oled.show()

oled_message("VPC-Tekenrobot", "Gereed", "Druk knop SW1")
brachiograph.home()

# =========================================================
# HOOFDLUS
# =========================================================
# set_xy(60, 80)
# time.sleep(5)
# while True:
#     if button_schouder.value():
#         x0 = -40
#         dx = 20
#         y0 = 50
#         dy = 20
#         x = x0
#         pen_up()
#         cnt = 0
#         stop = False
#         while x < 140:
#             if stop:
#                 print("Drawing stopped.")
#                 break
#             y = y0
#             while y < 141:
#                 if button_pen.value():  # break
#                     stop = True
#                     break
#                 cnt += 1
#                 set_xy(x, y)
#                 print(f"cnt = {cnt}; x = {x}; y = {y}")
#                 time.sleep(1)
#                 pen_down()
#                 time.sleep(3)
#                 pen_up()
#                 time.sleep(1)
#                 y += dy
#             x += dx
#
#         home()
#
#         while button_schouder.value():
#             time.sleep(0.01)
#
#
#
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
            brachiograph.set_xy(x, y)
            if theta == 0:
                time.sleep(1)
                brachiograph.pen_down()
                time.sleep(1)
            time.sleep(0.03)
            theta += theta_step
        time.sleep(1)
        brachiograph.pen_up()
        time.sleep(1)
        brachiograph.home()

        while button_schouder.value():
            time.sleep(0.01)


#
