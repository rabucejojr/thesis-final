import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import requests
import serial
import os, time
from tkinter import *

#oled display Imports
import time
import board
import busio
from pathlib import Path
from luma.core.interface.serial import i2c, spi, pcf8574
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106, ws0010
from PIL import ImageFont


#GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
reader = SimpleMFRC522()



#OLED DISPLAY, Initialization/Setup
def oled_disp(text):
    font = ImageFont.load_default()
    font_mc = ImageFont.truetype('/home/admin/Desktop/scripts/Arialn.ttf', 32) #
    font_mcsmall = ImageFont.truetype('/home/admin/Desktop/scripts/Arialn.ttf', 14)
    serial = i2c(port=1, address=0x3C)
    device = sh1106(serial)
    x=0
    top=0
    width = device.width
    height = device.height
    with canvas(device) as draw:
        draw.text((x, top),  text  ,  font=font_mcsmall, fill=255)

#SIM900A
def sendMessage(phone):
    port.write(b'AT\r')
    rcv = port.read(10)
    time.sleep(1)
    port.write(b"AT+CMGF=1\r")
    print("Text Mode Enabled…")
    oled_disp("Text Mode Enabled…")
    time.sleep(3)
    a = b'AT+CMGS="'
    b = b'"\r'
    print(a+phone+b)
    port.write(a+phone+b)
    msg = " xxx Your son/daughter scanned his/her RFID. He/she is now inside the campus. xxx"
    print("sending message….")
    oled_disp("sending message...")
    time.sleep(3)
    port.reset_output_buffer()
    time.sleep(1)
    port.write(str.encode(msg+chr(26)))
    time.sleep(3)
    print("message sent…")
    oled_disp("message sent…")
    time.sleep(0.5)

#Code Start Here
while True:
    print("Scan your tag...")
    oled_disp("Scan your tag...")
    try:
        id = reader.read()[0]
        print(id)
        x = requests.get('http://snnhs-attendance-system.herokuapp.com/api/user/'+ str(id))
        if x.status_code == 404:
            print("tag not found")
            oled_disp("unregistered tag...")
            time.sleep(0.5)
        else:
            phone = (x.json()['phone'])
            phone = '+63'+phone
            phone = bytes(phone,'utf-8')
            y = requests.post('http://snnhs-attendance-system.herokuapp.com/api/attendance/', data = {"rfid":id})
            sendMessage(phone)
    finally:
        #GPIO.setwarnings(False)
        GPIO.cleanup()