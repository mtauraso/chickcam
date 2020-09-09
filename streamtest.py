#!/usr/bin/env python3


# Python stdlib doesn't have this
import termios
import tty
import fcntl
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

from contextlib import contextmanager
import threading
import datetime
import picamera 
from time import sleep
class Camera:
    # Overall frontend and lifecycle management of picamera
    # and any associated objects with similar lifecycles (update threads etc)
    # No responsibility for multiplexer

    # Video and encoder params
    # PiCamera's h264 encoder is what we're actually using to stream to twitch
    WIDTH = 1280 # Pixels
    HEIGHT = 720 # Pixels
    KEYFRAME_SEC = 2 # 2 seconds
    # Twitch upper limit is 6 Mbps. Limiting factor here is audio processing on the pi
    # We tune this to 3Mbps total, with 128Kbps reserved for audio
    # Units below are Bits Per Second
    BITRATE = 3 * (10**6) - 128 *(10**3) 

    def __init__(self, fps):
        self.started = False
        self.camera = None
        self.stream_pipe = None
        self.fps = fps
        self.flavor_text = "" # Public Member
        
        # Set up Events for annotation thread
        self.annotation_running = threading.Event()
        self.camera_running = threading.Event()
        self.annotation_thread = threading.Thread(target = self.annotation_loop)
        self.annotation_running.set()
        self.camera_running.clear()
        self.annotation_thread.start()

    def release(self):
        print ("DEALLOCATING CAMERA")
        if self.started:
            self.stop()
        # Stop and clear the annotation thread
        self.camera_running.clear()
        self.annotation_running.clear()
        self.annotation_thread.join()

    # Annotation update thread lifecycle
    # TODO start/stop a thread that runs this in a loop
    def annotation_loop(self):
        print ("AT: Thread Start")
       
        signal_timeout = 0.1
        update_timeout = self.frame_time_s()/2.0
        
        while self.annotation_running.wait(signal_timeout):
            while self.camera_running.wait(signal_timeout):
                date = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                self.camera.annotate_text = f"{self.flavor_text} {date}"
                sleep(update_timeout)
        
        print("AT: Shutting down")

    
    # Utility to give the integer number of frames
    # in between keyframes
    def keyframe_interval(self):
        return max(1,int(self.fps*Camera.KEYFRAME_SEC))

    # Returns frame length as an integer number of microseconds
    # Returns a minimum of 1
    def frame_time_us(self):
        us_per_s = 1000000.0 # 10^6 microseconds per second
        return int(max(1.0, self.frame_time_s()*us_per_s))

    # Returns a floating point number of seconds 
    def frame_time_s(self):
        return float(1.0/self.fps)
    
    # I think the camera backend will acommodate this
    # because it has a switcher. Todo: test
    def preview(self, sec):
        self.camera.start_preview()
        sleep(sec)
        self.camera.stop_preview()

    # Start and stop recording
    def start(self, stream_pipe):
        if self.started:
            return
       
        self.camera = picamera.PiCamera(
                resolution=(Camera.WIDTH,Camera.HEIGHT), 
                framerate=self.fps)
        #self.camera.shutter_speed = int((1.0/FPS)*1000000) #Frame length in microseconds
        self.camera.vflip = True
        self.camera.hflip = True
        self.camera.exposure_mode = 'auto'
        self.camera.iso = 800 

        # Set up annotations
        self.camera.annotate_frame_num = False
        self.camera.annotate_text_size = 20

        self.camera_running.set()

        # Start recording if the recording has a place to go
        self.stream_pipe = stream_pipe
        if self.stream_pipe:
            self.camera.start_recording(self.stream_pipe,
                    sei = True,
                    sps_timing = True,
                    intra_period = self.keyframe_interval(),
                    format="h264", 
                    bitrate=Camera.BITRATE)
    
        self.started = True

    def stop(self):
        if not self.started:
            return

        if self.stream_pipe:
            self.camera.stop_recording()
            self.stream_pipe = None

        # Stop annotator writing to camera
        # This really should be a mutex so we can tell
        # that the annotator thread isn't touching the camera
        self.camera_running.clear()

        # Close out picamera
        self.camera.close()
        self.camera = None

        # Fully stopped
        self.started = False
    
    # Context where the camera is temporarily
    # stopped from recording.
    # Noop if camera not started
    @contextmanager
    def off(self):
        was_started = self.started;
        if was_started:
            old_stream_pipe = self.stream_pipe
            self.stop()

        yield self;

        if was_started:
            self.start(old_stream_pipe)


#import os
import subprocess
import json

class Streamer:
    # Overall Responsibility for a streaming process that can accept
    # Frames via its stream_pipe from a Camera

    def __init__(self, fps, mute = False):
        self.streaming = False
        self.streaming_proc = None
        self.fps = int(fps)
        self.mute = mute
        
        passwords = None
        with open ('passwords.json') as f:
            passwords = json.load(f)
        
        stream_key = passwords["twitch_stream_key"]
        self.twitch_rtmp = f"rtmp://live-sea.twitch.tv/app/{stream_key}"

    def stream_pipe(self):
        return None if not self.streaming_proc else self.streaming_proc.stdin

    def start(self, mute = False):
        if self.streaming:
            return
        
        self.mute = mute

        audio_args = [
                # Select Audio input from ALSA and the USB sound card
                "-f", "alsa"
                ,"-thread_queue_size", "128"
                ,"-i", "default:CARD=Device"

                # Audio Encoding (Raw audio is PCM/WAV)
                ,"-codec:a", "aac"
                ,"-ar", "48000" # Chose 48k to match microphone/sound card default
                # 128k is CD quality audio. See notes on BANDWIDTH for audio/video 
                # Bandwidth trade off
                # TODO: Decouple Streamer & Camera Bandwidth calc my moving to app
                ,"-ab", "128k"

                
                # TODO: Tune high-pass filter to remove noise
                # TODO: Shop for and tune other ffmpeg filters to clean up audio
                ,"-filter_threads","4" # Set the processing threads for filters
                # Change the audio volume only so we can hear the chicks
                #,"-filter:a", "volume=7dB"
                # Both of these fit within Pi's performance
                #,"-filter:a","volume=7dB,highpass"
                ,"-filter:a", "highpass@frequency=1500,afftdn,volume=10dB"
                
        ] if not self.mute else []
        
        # Capture video from stdin, send to twitch with no re-encode
        stream_cmd = [ "/usr/bin/ffmpeg",
                # Controlling video capture
                # FPS Setting. FFMPEG keeps each source on its own thread 
                # and constructs its own timestamps?..? 
                "-framerate", f"{self.fps}",

                # Video input is on STDIN.
                "-thread_queue_size", "128", #Set a queue size of 128 packets for video
                "-f", "h264", #force h264 so ff doesnt have to detect it
                "-i", "-" # Input is from STDIN
        ]
        stream_cmd += audio_args
        stream_cmd += [
                # Twitch output
                "-use_wallclock_as_timestamps", "1", # Use wallclock timestamps 
                "-flush_packets","1", # I think this will have better sync...
                "-codec:v", "copy", # Don't re-encode
                "-f", "flv", # Use FLV format
                self.twitch_rtmp  # Use twitch url
        ]
        self.streaming_proc = subprocess.Popen(stream_cmd, 
                stdin = subprocess.PIPE)
        self.streaming = True

    def stop(self):
        if not self.streaming:
            return
        print("Closing Stream Pipe")
        self.streaming_proc.stdin.close()
        print("Sending sigterm to stream process")
        self.streaming_proc.terminate()
        self.streaming_proc.wait()
        print("Streaming process has died")
        self.streaming = False


import sys
import RPi.GPIO as GPIO
class ChickCam:
    FPS = 30.0

    def setup_multiplexer(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(11, True)
        GPIO.output(12, True)
        # Always do one cmera switch prior to starting
        # Always switch to the first camera (i.e. not the IR one)
        self.is_ir = False
        self.switch_camera()

    def switch_camera(self):
        with self.camera.off():
            self.camera.flavor_text = self.flavorText()
            # IR == Camera C
            # Visual == Camera A
            i2c_cmd = ["i2cset","-y","1","0x70","0x00", "0x06" if self.is_ir else "0x04"]
            gpio_state = [0,1,0] if self.is_ir else [0,0,1]
            subprocess.run(i2c_cmd)
            GPIO.output(7, gpio_state[0])
            GPIO.output(11, gpio_state[1])
            GPIO.output(12, gpio_state[2])
    
    def __init__(self, start_stream = False, start_ir = False):
        # Always start stream as false and turn it on later
        self.is_streaming = False
        self.is_muted = False
        self.camera = Camera(ChickCam.FPS)
        self.streamer = Streamer(ChickCam.FPS, mute = self.is_muted)
        self.setup_multiplexer()

        if start_ir:
            self.ir_camera()

        if start_stream:
            self.start_stream()

    def cleanup(self):
        print("Cleaning Up App")
        self.stop_stream()
        self.camera.release()
        GPIO.cleanup()
        print("All Done")

    def flavorText(self):
        return "Infrared" if self.is_ir else "Visual"
    
    ## Interface to blinkenlights
    ## TODO

    ## Interface for GPIO switches
    # Begin streaming to twitch from current input
    def start_stream(self):
        self.streamer.start(mute = self.is_muted)
        self.camera.start(self.streamer.stream_pipe())
        self.is_streaming = True
        print("Started Streaming")

    # Stop streaming to twitch
    def stop_stream(self):
        self.camera.stop()
        self.streamer.stop();
        self.is_streaming = False
        print("Stopped Streaming")

    @contextmanager
    def stream_off(self):
        was_streaming = self.is_streaming
        if was_streaming:
            self.stop_stream()

        yield

        if was_streaming:
            self.start_stream()

    ## Interface for kbd test driver program
    def toggle_stream(self):
        if self.is_streaming:
            self.stop_stream()
        else:
            self.start_stream()

    def toggle_camera(self):
        self.is_ir = not self.is_ir
        self.switch_camera()

    def toggle_mute(self):
        with self.stream_off():
            self.is_muted = not self.is_muted

    def status(self):
        print("")
        print(f"is_streaming = {self.is_streaming}")
        print(f"is_ir = {self.is_ir}")
        print(f"mute = {self.is_muted}")
        print("")

    def process_input(self):
        c = getch()
        
        if c == 'c':
            print ('Switching Camera')
            self.toggle_camera()
            self.status()

        elif c == 's':
            print ('Toggling Stream')
            self.toggle_stream()
            self.status()

        elif c == 'm':
            print ('Toggling Mute')
            self.toggle_mute()
            self.status()

        elif c == 'p':
            print ('2 second camera preview')
            self.camera.preview(2)
            self.status()

        elif c == 'q':
            return False

        return True


    def mainLoop(self):
        try:
            while self.process_input():
                sleep(0.1)

        except KeyboardInterrupt:
            print("Keyboard interrupt, closing")


# Make the App
if __name__ == "__main__":
    try:
        cc = ChickCam(start_stream = True )
        cc.mainLoop()
    finally:
        cc.cleanup()

