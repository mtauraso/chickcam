#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

led_pin = 11 # pin 11 --- LED (we think? I am dubious)
led2_pin = 13
button_pin = 12 # Pin 12 --- Button (We think...? I am dubious)
button_bounce_time = 200 #Debounce time in ms for button

# Global LED status variable
# HIGH turns the led OFF, LOW turns the led ON
led_status = GPIO.HIGH

# Callback function we pass to add_event_detect
# To be triggered on the button press
def switchLed(ev=None):
	global led_status
	
	if led_status == GPIO.HIGH:
		led_status = GPIO.LOW
		print("LED On")
		
	elif led_status == GPIO.LOW:
		led_status = GPIO.HIGH
		print("LED Off")
		
	# Set the LED to a particular value
	GPIO.output(led2_pin, led_status)
	GPIO.output(led_pin, led_status)
	
def setup():
	global led_status
	# Using physical pin numbers
	GPIO.setmode(GPIO.BOARD) 
	
	# Setup modes for each pin
	GPIO.setup(led_pin, GPIO.OUT)
	GPIO.setup(led2_pin, GPIO.OUT) 
	GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
	
	# Set output to high initially
	GPIO.output(led_pin, led_status)
	GPIO.output(led2_pin, led_status)
	
	#Setup Event listener
	GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=switchLed, bouncetime=200)

def destroy():
	GPIO.output(led_pin, GPIO.HIGH)
	GPIO.cleanup()
	
if __name__ == '__main__':
	setup()
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		destroy()

	
			
