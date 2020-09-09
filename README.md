# chickcam

This has an app which streams baby chickens to twitch.

## Getting Started
See Hardware and Software sections below to set up the Pi and install dependencies

The main application is all in streamtest.py and is invoked by the headless_startup.sh script. The headless_startup script is designed to be run by rc.local on a standard RPi noobs install.

You need a file in the root directory of the checkout called `passwords.json` with a key called "twitch_stream_key" which will be used to access Twitch.tv. 

Control is via attaching to the tmux session started for the pi user and typing single character commands.
* 'c' Change Camera using the Camera Multiplexer
* 's' Start/Stop Streaming
* 'q' Quit the camera program
* 'm' Mute or unmute audio (This causes the stream to be fully stopped and started)

## Roadmap
* SW: Separate classes from streamtest.py to separate files
* SW: Add Switch control & indicator light functionality
* HW: Add Battery pack
* HW/SW: Add IR illumination in IR Mode
* HW: Assemble board and accessories to a single integrated unit

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

