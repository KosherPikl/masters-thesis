'''
Tuck Forbes Master's Thesis Code V1
7/31/25  
This code combines the accelerometer and camera code examples found 
online. The BMI160 accelerometer uses I2C and the RT1062 camera uses UART.
Example code and sources can be found in folder titled Example Code
'''

from DFRobot_BMI160 import * 	# Accelerometer library
import serial 					# Serial library for UART
import numpy as np				# Library for matrices and vectors
import sys						# Library for accessing I2C
import os						# Library for navigating operating system
import time						# Library for importing time

# Setting BMI160 Port address to the low port at 0x68 
bmi160 = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_L) 

# Opening serial port for UART
ser = serial.Serial ("/dev/serial0", 19200) # Open UART port with baud rate

# Checking connection to BMI160
while bmi160.begin() != BMI160_OK:
	print("BMI160 failed to connect.")
	time.sleep(1)
print("BMI160 connected.")
  
  # Main section where code runs till ended
while True:
	# BMI160 section
	data = bmi160.get_sensor_data() 		# Gather BMI160 data
	# Display BMI160 Data
	# Convert gyro from degrees to radians
	print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0))
	# Convert accelerometer data to gravity
	print("accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0))
	print("\n")
	time.sleep(0.1) 						# Wait 0.1 seconds
											# Camera section
	received_data = ser.read()              # Read serial port
	time.sleep(0.03)# Wait 0.03 seconds
	data_left = ser.inWaiting()             # Check for remaining data
	received_data += ser.read(data_left) 	# Combine data
	print (received_data)                   # Print received data
