'''
Tuck Forbes Master's Thesis LED Test Code
8/1/25  
This code receives voltage from the pogo pin connection, reads that 
voltage, then turns on an LED.
'''

import RPi.GPIO as GPIO			# GPIO library
import time						# Library for importing time



# Setting up GPIO Pin Numbering
GPIO.cleanup()					# Reset all GPIO pins
GPIO.setmode(GPIO.BOARD)		# Using normal board pinout
GPIO.setwarnings(False)

# Setting pin modes
inpin = 16
outpin = 18
GPIO.setup(inpin,GPIO.IN)			# Make pin input
GPIO.setup(outpin,GPIO.OUT)			# Make pin output

# Setting LED to low to start
GPIO.output(outpin,GPIO.LOW)



print(GPIO.input(inpin))

# Main section where code runs till ended
while True:
	'''
	# Flash LED
	print("LED on")
	GPIO.output(outpin,GPIO.HIGH)
	time.sleep(1)
	GPIO.output(outpin,GPIO.LOW)
	print("LED off")
	time.sleep(1)
	'''
	if GPIO.input(inpin):
		print("Docking Successful")
		GPIO.output(outpin,GPIO.HIGH)
	else:
		GPIO.output(outpin,GPIO.LOW)
	time.sleep(0.1)
