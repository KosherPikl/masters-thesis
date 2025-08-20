'''
Tuck Forbes Master's Thesis Code
Docking Test 1
8/19/25  
This code combines the PD controller and the docking light code into one
file to test autonomous rendezvous & docking
'''


import socket					# UDP library
from DFRobot_BMI160 import * 	# Accelerometer library
import serial 					# Serial library for UART
import numpy as np				# Library for matrices and vectors
import sys						# Library for accessing I2C
import os						# Library for navigating operating system
import time						# Library for importing time
import struct					# Library for converting from bytes 
import RPi.GPIO as GPIO			# GPIO library
import math						# Math library

# Setting BMI160 Port address to the low port at 0x68 
bmi160 = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_L) 

# Opening serial port for UART
ser = serial.Serial ("/dev/serial0", 19200) # Open UART port with baud rate

# Setting UDP IP address and socket for receiver
UDP_IP = "192.168.2.155" 					# receiver IP address
UDP_Port = 5555 							# port for receiving data
sock = socket.socket(socket.AF_INET,		# Internet 
	socket.SOCK_DGRAM) 						# UDP
	
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


# Checking connection to BMI160
while bmi160.begin() != BMI160_OK:
	print("BMI160 failed to connect.")
	time.sleep(1)
print("BMI160 connected.")

# Zeroing BMI160
data_offset = bmi160.get_sensor_data() # Gather initial BMI160 data (not moving)

# Create string of time
timestr = time.strftime("%Y_%m_%d-%H_%M_%S")

# Find start time
start_time = time.time()

# Creating file to save acc data
accel_file_name = "accel_data_" + timestr
fa =  open(accel_file_name,"x") 
fa.write("grav grav grav sec\n")
fa.write("initial data: %f %f %f\n"%(data_offset['accel']['x']/16384.0, data_offset['accel']['y']/16384.0, data_offset['accel']['z']/16384.0))

# Creating file to save gyro data
gyro_file_name = "gyro_data_" + timestr
fg =  open(gyro_file_name,"x") 
fg.write("rads/s rads/s rads/s sec\n")
fg.write("initial data: %f %f %f\n"%(data_offset['gyro']['x']*3.14/180.0, data_offset['gyro']['y']*3.14/180.0, data_offset['gyro']['z']*3.14/180.0))
  
# Main section where code runs till ended
while True:
# BMI160 section
	imu_data = bmi160.get_sensor_data() 		# Gather BMI160 data
	'''
	# Display BMI160 Data
	# Convert gyro from degrees to radians
	print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(imu_data['gyro']['x']*3.14/180.0, imu_data['gyro']['y']*3.14/180.0, imu_data['gyro']['z']*3.14/180.0))
	# Convert accelerometer data to gravity
	print("accel :  x: %.3f g,  y: %.3f g,  z: %.3f g"%(imu_data['accel']['x']/16384.0, imu_data['accel']['y']/16384.0, imu_data['accel']['z']/16384.0))
	print("\n")
	'''
	acc_data = "%f %f %f %f\n"%(imu_data['accel']['x']/16384.0, imu_data['accel']['y']/16384.0, imu_data['accel']['z']/16384.0, time.time()-start_time)
	gyro_data = "%f %f %f %f\n"%(imu_data['gyro']['x']*3.14/180.0, imu_data['gyro']['y']*3.14/180.0, imu_data['gyro']['z']*3.14/180.0, time.time()-start_time)

# Write data to file
	fa.write(acc_data)
	fg.write(gyro_data)
	
# Camera section
	received_data = ser.read()              # Read serial port
	time.sleep(0.25)						# Wait 0.25 seconds
	data_left = ser.inWaiting()             # Check for remaining data
	received_data += ser.read(data_left) 	# Combine data
	
	print (received_data)                   # Print received data
	print (type(received_data))
	
	cam_data_decode = received_data.decode()
	cam_data_list = cam_data_decode.split()	# Make data into list
	'''
	print(cam_data_list)	
	print(type(cam_data_list))
	'''
	cam_data = np.zeros(6)
	for i in range(6):
		cam_data[i] = float(cam_data_list[i]) # make data into float
		
	#print(cam_data)
	trans_data = cam_data[:3]
	#print(trans_data)
	rot_data = cam_data[3:]
	#print(rot_data)
		
	
# Control law section
	forw_dist = -1*trans_data[2]; 	# Data comes out - so mult by -1
	l_r_dist = trans_data[1];	# Left is - right is +
	'''
	print(l_r_dist)
	rot_angle = -1*(rot_data[0]-180); # + is CW - is CCW (data is 0-360)
	print(rot_angle)
	'''
	'''
	Step 1: Align with target in l/r direction
	Step 2: Align with target in angle
	Step 3: Drive to target
	'''
	l_r_tol = 0.5			# Set left right tolerance here
	forw_tol = 0		# Set forward tolerance here
	ang_tol = 5			# Set tolerance for angle here
	
	if abs(l_r_dist) >= l_r_tol:	# If outside l/r tolerance fix it
		wait_time = math.sqrt(abs(l_r_dist))
		if l_r_dist > 0: 		# If left of target move right
			command = "Right"	# Set command to right
		else: 					# If not left of target move left
			command = "Left"	# Set command to left
	elif abs(rot_angle) >= ang_tol:	# If outside angle tolerance fix it
		wait_time = 0.1*abs(rot_angle)
		if rot_angle > 0:		# If CW from target move CCW
			command = "CCW"		# Set command to CCW
		else: 					# If not CW from target move CW
			command = "CW"		# Set command to CW
	elif abs(forw_dist) >= forw_tol:	
		wait_time = sqrt(abs(forw_dist))
		command = "Forward"
		
# Command send section
	message = command 						# Write Command
	sock.sendto(message.encode(), (UDP_IP, UDP_Port)) # Send Command
	print(f"Sent: {message} to {UDP_IP}:{UDP_Port}")  # Confirm send
	time.sleep(wait_time)  					# Wait before stopping
	# Stop vehicle
	message = "Stop"
	sock.sendto(message.encode(), (UDP_IP, UDP_Port)) # Send Command
	
# Check if docking has occurred
	while GPIO.input(inpin):
		print("Docking Successful")
		GPIO.output(outpin,GPIO.HIGH)	# Turn on LED
		message = "Stop"				# Stop Vehicle
		sock.sendto(message.encode(), (UDP_IP, UDP_Port)) # Send Command

	GPIO.output(outpin,GPIO.LOW)
	time.sleep(0.1)
