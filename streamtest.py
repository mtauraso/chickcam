#!/usr/bin/env python3

import json
import sys
import os
import subprocess

import termios
import tty

from time import sleep

import RPi.GPIO as GPIO
import picamera 


# TODO write into Object structure below
# Load configuration
passwords = {}
with open ('passwords.json') as f:
    passwords = json.load(f)
STREAM_KEY = passwords["twitch_stream_key"]
TWITCH_RTMP = f"rtmp://live-sea.twitch.tv/app/{STREAM_KEY}"

# Video params
FPS = 30 # Frames per second
WIDTH = 1280 # Pixels
HEIGHT = 720 # Pixels

# Encode params
#KEYFRAME_INTERVAL = 60 # Frames per second (2 s interval)
BITRATE = 4000000 # Bits/Second (4MBits/s) Twitch limit is 6 MBits/s


# Python stdlib doesn't have this
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class ChickCam:
    def setup_multiplexer(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(11, True)
        GPIO.output(12, True)

    def switch_camera(self):
        # IR == Camera C
        # Visual == Camera A
        i2c_cmd = ["i2cset","-y","1","0x70","0x00", "0x06" if self.is_ir else "0x04"]
        gpio_state = [0,1,0] if self.is_ir else [0,0,1]
        subprocess.run(i2c_cmd)
        GPIO.output(7, gpio_state[0])
        GPIO.output(11, gpio_state[1])
        GPIO.output(12, gpio_state[2])
    
    def __init__(self):
        self.is_streaming = False
        self.is_ir = False
        self.setup_multiplexer()
        self.switch_camera()

    def __del__(self):
        self.stop_stream()
        GPIO.cleanup()
        print("Cleaned up GPIO")


    def start_twitch_pipe(self):
        # Capture video from stdin, send to twitch with no re-encode
        # TODO: Audio gets muxed in here
        stream_cmd = [ "/usr/bin/ffmpeg", 
                "-r", f"{FPS}", # Set FPS to correct FFMPEGs monitor output
                "-i", "-", # Input is from STDIN
                "-codec", "copy", # Don't re-encode
                "-f", "flv", TWITCH_RTMP] # Use FLV format and twitch url
        return subprocess.Popen(stream_cmd, stdin=subprocess.PIPE)

    def start_camera(self):
        self.camera = picamera.PiCamera(resolution=(WIDTH,HEIGHT), framerate=FPS)
        return self.camera


    ## Interface to blinkenlights
    ## TODO

    ## Interface to GPIO switches
    # Begin streaming to twitch from current input
    
    def start_stream(self):
        if self.is_streaming:
            print("Already streaming")
        else:
            self.is_streaming = True
            self.stream_proc = self.start_twitch_pipe()
            print("Opened Streaming proc")
            self.camera = self.start_camera()
            self.camera.start_recording(self.stream_proc.stdin, format="h264", bitrate=BITRATE)

            print("Started Streaming")

    # Stop streaming to twitch
    def stop_stream(self):
        if self.is_streaming:
            self.is_streaming = False

            print("Stopping Recording")
            self.camera.stop_recording()
            self.camera.close()
            print("Closing Stream Pipe")
            self.stream_proc.stdin.close()
            print("Closed stream pipe")
            self.stream_proc.wait()
            print("Streaming Process dead")
        else:
            print("Not Streaming")

    # switch to visual camera
    def visual_camera(self):
        if self.is_ir:
            self.is_ir = False
            self.switch_camera()
            # TODO probably need to reinitialize
        else:
            print("Already on visual camera")

    # Switch to IR camera
    def ir_camera(self):
        if self.is_ir:
            print("Already on IR camera")
        else:
            self.is_ir = True
            self.switch_camera()
            # TODO probably need to reinitialize
    

    ## Interface for kbd test driver program
    def toggle_stream(self):
        if self.is_streaming:
            self.stop_stream()
        else:
            self.start_stream()

    def toggle_camera(self):
        if self.is_ir:
            self.visual_camera()
        else:
            self.ir_camera()

    def preview(self):
        if not self.is_streaming:
            self.camera = self.start_camera()

        self.camera.start_preview()
        sleep(2)
        self.camera.stop_preview()

        if not self.is_streaming:
            self.camera.close()

    def status(self):
        print("")
        print(f"is_streaming = {self.is_streaming}")
        print(f"is_ir = {self.is_ir}")
        print("")

    def mainLoop(self):
        try:
            while True:
                c = getch()
                if c == 'c':
                    print ('Switching Camera')
                    cc.toggle_camera()

                elif c == 's':
                    print ('Toggling Stream')
                    cc.toggle_stream()

                elif c == 'p':
                    print ('2 second camera preview')
                    cc.preview()

                elif c == 'q':
                    break;

                else:
                    continue;

                # Status to console once we make the change
                cc.status()
        except KeyboardInterrupt:
            print("Keyboard interrupt, closing")


# Make the App
if __name__ == "__main__":
    try:
        cc = ChickCam()
        cc.mainLoop()
    finally:
        del cc

