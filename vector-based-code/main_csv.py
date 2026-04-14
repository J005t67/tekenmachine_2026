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
import Vector2D
from ulab import numpy as np
import brachi

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
button_SW1 = Pin(13, Pin.IN, Pin.PULL_DOWN)
button_SW2 = Pin(14, Pin.IN, Pin.PULL_DOWN)
button_SW3 = Pin(15, Pin.IN, Pin.PULL_DOWN)



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

from machine import SPI, Pin
import sdcard, os

# Constants
SPI_BUS = 0
SCK_PIN = 2
MOSI_PIN = 3
MISO_PIN = 4
CS_PIN = 5
SD_MOUNT_PATH = '/sd'
FILE_PATH = 'sd/sd_file.txt'

# Initialize and mount SD card
def mount_sd():
    try:
        sd = sdcard.SDCard(spi, cs)
        vfs = os.VfsFat(sd)
        os.mount(vfs, '/sd')
        print("✅ SD Card mounted successfully!")
    except OSError as e:
        print("❌ Failed to mount SD card:", e)

# Unmount SD card
def unmount_sd():
    try:
        os.umount('/sd')
        print("✅ SD Card unmounted successfully!")
    except OSError as e:
        print("❗ Error unmounting SD card:", e)


def extract_num(text):
    numbers = []
    current_number = ""

    for char in text:
        if char.isdigit() or char == '.':
            current_number += char
        elif current_number:
            numbers.append(float(current_number))
            current_number = ""

    # Don't forget the last number
    if current_number:
        numbers.append(float(current_number))
    return numbers

def line_float(line):
    line = line.rstrip('\n').rstrip('\r').split(delim)
    line = [x for x in line if x]
    lne = []
    for pnt in line:
        lne.append(extract_num(pnt))
    x, y = zip(*lne)
    return x, y

def extent(x, y):
    extent = max(max(x) - min(x), max(y) - min(y))
    print(f"Extent: {extent}")
    return extent

def line_count(fn):
    with open(fn) as file_obj:
        count = 0
        for _ in file_obj:
            count += 1
        return count


try:
    # Init SPI communication
    spi = SPI(SPI_BUS, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    cs = Pin(CS_PIN)
    sd = sdcard.SDCard(spi, cs)
    # Mount microSD card
except Exception as e:
    print('An error occurred:', e)

scale = 1/10
x_offset = -20
y_offset = 130
print("Start drawing by pressing SW1. Stop with SW3")
delim = '\t'

oled_message("SW1: Start", "SW2: Pause", "SW3: Stop")
brachiograph.home()


# =========================================================
# HOOFDLUS
# =========================================================

while True:
    if button_SW1.value():
        mount_sd()
        file_name = "sd/station.csv"
        N_lines = line_count(file_name)
        stop = False
        pause = False
        cnt = 0
        with open(file_name) as file_obj:
            for line in file_obj:
                if stop:
                    print("Drawing stopped.")
                    break
                cnt += 1
                print(f"Percentage done: {cnt/N_lines*100:3.1f}")
                x, y = line_float(line)
                if extent(x, y) > 100:
                    start_line = True
                    for x, y in zip(x, y):
                        if button_SW3.value(): # break
                            stop = True
                            break
                        if button_SW2.value(): # pause
                            while button_SW2.value():
                                time.sleep(0.01)
                            pause = not pause
                            while pause:
                                time.sleep(0.01)
                                if button_SW2.value(): # unpause
                                    while button_SW2.value():
                                        time.sleep(0.01)
                                    pause = not pause
                                if button_SW3.value(): # break during pause
                                    stop = True
                                    break

                        x = x * scale + x_offset
                        y = -1 * y * scale + y_offset
                        brachiograph.set_xy(x, y)
                        if start_line:
                            time.sleep(0.2)
                            brachiograph.pen_down()
                            time.sleep(0.2)
                            start_line = False
                        else:
                            time.sleep(0.01)
                    brachiograph.pen_up()
                    time.sleep(0.5)
        brachiograph.home()
        unmount_sd()
        while button_SW1.value():
            time.sleep(0.01)

        time.sleep(0.01)

