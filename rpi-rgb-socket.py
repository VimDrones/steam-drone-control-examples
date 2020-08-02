#!/usr/bin/env python3   
import time
from neopixel import *

LED_COUNT      = 6      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)

strip.begin()

def show(r, g, b):
    for i in range(LED_COUNT):
        strip.setPixelColor(i,  Color(g, r, b))
    strip.show()

print("led init...")
for r,g,b in [[255,255,255],[255,0,0],[0,255,0],[0,0,255],[0,0,0]]:
    show(r, g, b)
    time.sleep(0.5)

import struct
import socket
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)
    if len(data) == 3:
        r,g,b = struct.unpack('BBB', data)
        show(r,g,b)
