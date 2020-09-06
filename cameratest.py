#!/usr/bin/env python3

from picamera import PiCamera
from time import sleep
import os
import RPi.GPIO as gp

adapter_info = {   "A":{   "i2c_cmd":"i2cset -y 1 0x70 0x00 0x04",
						   "gpio_sta":[0,0,1],
						},
					"B":{
							"i2c_cmd":"i2cset -y 1 0x70 0x00 0x05",
							"gpio_sta":[1,0,1],
						},
					"C":{
							"i2c_cmd":"i2cset -y 1 0x70 0x00 0x06",
							"gpio_sta":[0,1,0],
						},
					"D":{
							"i2c_cmd":"i2cset -y 1 0x70 0x00 0x07",
							"gpio_sta":[1,1,0],
						},
				 } 


def setupMultiplexer():
    #gp.setwarnings(False)
    gp.setmode(gp.BOARD)
    gp.setup(7, gp.OUT)
    gp.setup(11,gp.OUT)
    gp.setup(12,gp.OUT)
    gp.output(11, True)
    gp.output(12, True)


def choose_channel(index):
    channel_info = adapter_info.get(index)
    if channel_info == None:
        print("Can't get this info")
    os.system(channel_info["i2c_cmd"]) # i2c write
    gpio_sta = channel_info["gpio_sta"] # gpio write
    gp.output(7, gpio_sta[0])
    gp.output(11, gpio_sta[1])
    gp.output(12, gpio_sta[2])


def select_channel(index):
    channel_info = adapter_info.get(index)
    if channel_info == None:
        print("Can't get this info")
    gpio_sta = channel_info["gpio_sta"] # gpio write
    gp.output(7, gpio_sta[0])
    gp.output(11, gpio_sta[1])
    gp.output(12, gpio_sta[2])


setupMultiplexer()

choose_channel("A")
camera = PiCamera()
camera.resolution = (320,240)
camera.start_preview()
sleep(5)
camera.stop_preview()

choose_channel("C")
camera.resolution = (1280,720)
camera.start_preview()
sleep(5)
camera.stop_preview()


choose_channel("A")
camera.resolution = (1280,720)
camera.start_preview()
sleep(5)
camera.stop_preview()
