#!/usr/bin/env python3

from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.resolution = (1280,720)
camera.image_effect = 'oilpaint'
camera.framerate = 30

camera.start_preview()
sleep(5)
camera.stop_preview()
