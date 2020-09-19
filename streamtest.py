#!/usr/bin/env python3

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
    # We tune this to 2Mbps total, with 128Kbps reserved for audio
    # Units below are Bits Per Second
    BITRATE = 2 * (10**6) - 128 *(10**3) 

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
            try:
                self.camera.stop_recording()
            # If the stream pipe is broken we will hit this
            # ignore the broken pipe and continue teardown
            except BrokenPipeError:
                pass
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

    # Return true if stream still alive
    # Need to call this in an update loop and re-create stream if
    # there is a network error
    def is_live(self):
        if self.streaming:
            return self.streaming_proc.poll() == None
        else:
            return False

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
                ,"-filter:a", "highpass@frequency=1500,highpass@frequency=1500,afftdn,volume=14dB"
                
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
import termios
import tty
import select
class ChickCam:
    FPS = 30.0
    SCHEDULE_EVAL_SEC = 300 # Only evaluate schedule (and reset user input) every 5 min
    STREAM_CHECK_SEC = 10 # Every 10 sec we check streaming

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
   

    def read_schedule(self):
        self.schedule = None
        
        if self.schedule_file == None:
            return

        with open(self.schedule_file) as f:
            self.schedule = json.load(f)

    def eval_schedule(self):
        # Only do this if we have a schedule
        if self.schedule == None:
            return

        # Find current time
        now_time = datetime.datetime.now()
        
        # Early return if not enough time has occurred since
        # last we evaluated the schedule
        if self.last_eval != None:
            sec_since_last_eval = (now_time - self.last_eval).seconds
            if sec_since_last_eval <= ChickCam.SCHEDULE_EVAL_SEC:
                return

        # We are evaluating
        self.last_eval = now_time

        # Go backward through schedule from current time until all vars are set
        # First we prepare a list of keys which is unique, ordered, and includes the 
        # current time
        search_list = list(self.schedule.keys())
        now_time_key = now_time.strftime('%H%M')
        print(f"Evaluating schedule at {now_time_key}")
        search_list.append(now_time_key)
        search_set = set(search_list)
        search_list = list(search_set)
        search_list = sorted(search_list, reverse = True)

        # We start our search at the current time
        search_index = search_list.index(now_time_key)

        is_ir = None
        is_streaming = None
        is_muted = None

        while (is_ir == None) or (is_streaming == None) or (is_muted == None):
            key_time = search_list[search_index]
            config = self.schedule.get(key_time)
            if config != None:
                if is_ir == None:
                    is_ir = config.get("is_ir")
                    if is_ir != None:
                        print(f"Schedule block {key_time} sets is_ir = {is_ir}")
                if is_streaming == None:
                    is_streaming = config.get("is_streaming")
                    if is_streaming != None:
                        print(f"Schedule block {key_time} sets is_streaming = {is_streaming}")
                if is_muted == None:
                    is_muted = config.get("is_muted")
                    if is_muted != None:
                        print(f"Schedule block {key_time} sets is_muted = {is_muted}")

            search_index = (search_index + 1) % len(search_list)

        changed = False

        #print(f"Current sched is_ir = {is_ir}, is_streaming = {is_streaming}, is_muted = {is_muted}")

        # Set streaming, muted, and ir state, calling relevant functions
        if self.is_streaming != is_streaming:
            print("Changing Streaming state to match schedule")
            self.toggle_stream()
            changed = True
        
        if self.is_muted != is_muted:
            print("Changing Mute to match schedule")
            self.toggle_mute()
            changed = True
        
        if self.is_ir != is_ir:
            print("Changing Camera to match schedule")
            self.toggle_camera()
            changed = True

        # Print status if we changed anything
        if changed:
            self.status()



    def __init__(self, start_stream = False, schedule = None):
        # Always start stream as false and turn it on later
        self.is_streaming = False
        self.is_muted = False
        self.camera = Camera(ChickCam.FPS)
        self.streamer = Streamer(ChickCam.FPS, mute = self.is_muted)
        self.setup_multiplexer()

        # Schedule tracking
        self.schedule_file = schedule
        self.read_schedule()

        # Eval the schedule immediately
        self.last_eval = None
        self.eval_schedule()

        if start_stream:
            self.start_stream()

    def cleanup(self):
        print("Cleaning Up App")
        self.stop_stream()
        self.camera.release()
        GPIO.cleanup()
        print("All Done")

    def flavorText(self):
        cameraType = "Infrared" if self.is_ir else "Visual"
        muting = " (Muted)" if self.is_muted else ""
        return f"{cameraType}{muting}"
    
    ## Interface to blinkenlights
    ## TODO

    ## Interface for GPIO switches
    # Begin streaming to twitch from current input
    def start_stream(self):
        self.streamer.start(mute = self.is_muted)
        self.camera.flavor_text = self.flavorText()
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
        print(f"is_muted = {self.is_muted}")
        print("")

    def process_input(self, c):

        changed = False

        if c == 'c':
            print ('Switching Camera')
            self.toggle_camera()
            changed = True

        elif c == 's':
            print ('Toggling Stream')
            self.toggle_stream()
            changed = True

        elif c == 'm':
            print ('Toggling Mute')
            self.toggle_mute()
            changed = True

        elif c == 'p':
            print ('2 second camera preview')
            self.camera.preview(2)
            self.status()

        elif c == 'r':
            print ('Reloading schedule')
            self.read_schedule()
            # evaluate the reloaded schedule immediately
            self.last_eval = None
            self.eval_schedule()

        elif c == 'q':
            return False

        # Each time the user makes a change push out
        # future schedule evals
        if changed:
            self.status()
            self.last_eval = datetime.datetime.now()

        return True


    def mainLoop(self):
        try:
            old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

            while True:
                if select.select([sys.stdin], [], [], ChickCam.STREAM_CHECK_SEC) == ([sys.stdin],[],[]):
                    if self.process_input(sys.stdin.read(1)):
                        continue
                    else:
                        break

                if self.schedule:
                    self.eval_schedule()

                if self.is_streaming and (not self.streamer.is_live()):
                    print ("Detected Stream dead, reinitializing")
                    self.toggle_stream()
                    self.toggle_stream()

        except KeyboardInterrupt:
            print("Keyboard interrupt, closing")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


# Make the App
if __name__ == "__main__":
    try:
        #cc = ChickCam(start_stream = True )
        cc = ChickCam(schedule = "schedule.json")
        cc.mainLoop()
    finally:
        cc.cleanup()

