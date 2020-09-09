# chickcam

This has an app which streams baby chickens to twitch.

## Getting Started
See Hardware and Software sections below to set up the Pi and install dependencies. Pi needs to be configured for i2c, camera, and GPIO. If you would like to do headless control you need tmux and a way to get a terminal on the pi. 

You need a file in the root directory of the checkout called `passwords.json` with a key called "twitch_stream_key" which will be used to access Twitch.tv. 

The main application is all in streamtest.py and is invoked by the headless_startup.sh script. The headless_startup script is designed to be run by rc.local on a standard RPi noobs install.
 
Control is via attaching to the tmux session started for the pi user and typing single character commands.
* 'c' Change Camera using the Camera Multiplexer
* 's' Start/Stop Streaming
* 'q' Quit the camera program
* 'p' 2 second camera preview on Pi's display (if plugged in)
* 'm' Mute or unmute audio (This causes the stream to be fully stopped and started)

## How the Software works
Everything is in streamtest.py There are three main classes:
* ChickCam Handles application state, keyboard control, camera multiplexer, setup & teardown during mode switches. The Camera Muiltiplexer uses 3 GPIO pins in this configuration and also needs i2c messages to switch properly.
* Camera Handles the PiCamera class, which is also a [frontend for an h264 encoder hiding on the Pi's graphics card](https://picamera.readthedocs.io/en/release-1.13/fov.html#background-processes). Camera als hosts a thread which updates the annotation text to show the current date and app-defined text, which is used to indicate Mute status and which camera is in use.
* Streamer runs and stops an ffmpeg process which does the heavy lifting of filtering and muxing in audio from the microphone and streaming the h264 video to twitch over Wifi. Video input to ffmpeg is over stdin (using subprocess.PIPE)

## Roadmap
* SW: Separate classes from streamtest.py to separate files
* SW: Synchronize audio and video streams for better cheeps and pecks
* SW: Schedule automatic transitions of camera, muting, or stream on/offline
* HW/SW: Prototype switch & indicator LEDs for main functions in tmux management interface
* HW: Add Battery pack for future outdoor operation
* HW: Assemble board and accessories to a single integrated unit
* Design: Add IR illumination in IR Mode
* SW: Fault detection and alarm on stream problems
* SW: Live stream processing statistics in video overlay

## Hardware Setup
* [Rasberry Pi 3bi with wifi](https://www.adafruit.com/product/3058)
* [Pi Camera Multiplexer](https://www.amazon.com/gp/product/B07TYL36MC/)
* [Visual](https://www.adafruit.com/product/3099) and [IR](https://www.adafruit.com/product/3100) PI cameras
* [USB Audio Adaptor](https://www.amazon.com/gp/product/B01N905VOY/)
* [Lavalier Microphone](https://www.amazon.com/gp/product/B01M4J5WCM/)

## Software Required
* *ffmpeg* Installed via apt-get, compilation took forever
* *tmux* to run headless startup scripts
* *alsa* for sound (free with NOOBS 3.1)
* *python3* (free with NOOBS 3.1)
* *pi camera python lib* (free with NOOBS 3.1)
* *pi GPIO python lib* (free with NOOBS 3.1)


