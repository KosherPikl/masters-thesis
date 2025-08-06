'''
Tuck Forbes Master's Thesis Code V2
8/1/25  
This code receives sensor data and sends out commands to the other 
raspberry pi. The sensors used are BMI160 and RT1062 camera. The commands
sent are strings of data. The commands are created using a basic PD 
controller.
'''


import socket					# UDP library
from DFRobot_BMI160 import * 	# Accelerometer library
import serial 					# Serial library for UART
import numpy as np				# Library for matrices and vectors
import sys						# Library for accessing I2C
import os						# Library for navigating operating system
import time						# Library for importing time
import struct					# Library for converting from bytes 

# Setting BMI160 Port address to the low port at 0x68 
bmi160 = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_L) 

# Opening serial port for UART
ser = serial.Serial ("/dev/serial0", 19200) # Open UART port with baud rate

# Setting UDP IP address and socket for receiver
UDP_IP = "192.168.1.155" 					# receiver IP address
UDP_Port = 5555 							# port for receiving data
sock = socket.socket(socket.AF_INET,		# Internet 
	socket.SOCK_DGRAM) 						# UDP

# Checking connection to BMI160
while bmi160.begin() != BMI160_OK:
	print("BMI160 failed to connect.")
	time.sleep(1)
print("BMI160 connected.")
  
# Main section where code runs till ended
while True:
# BMI160 section
	imu_data = bmi160.get_sensor_data() 		# Gather BMI160 data
	gyro_data = imu_data['gyro']
	acc_data = imu_data['accel']
	'''
	# Display BMI160 Data
	# Convert gyro from degrees to radians
	print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(imu_data['gyro']['x']*3.14/180.0, imu_data['gyro']['y']*3.14/180.0, imu_data['gyro']['z']*3.14/180.0))
	# Convert accelerometer data to gravity
	print("accel :  x: %.3f g,  y: %.3f g,  z: %.3f g"%(imu_data['accel']['x']/16384.0, imu_data['accel']['y']/16384.0, imu_data['accel']['z']/16384.0))
	print("\n")
	'''
	with open("acc_data.txt","a") as f:
		f.write(acc_data)
	with open("gyro_data.txt","a") as f:
		f.write(gyro_data)
	
# Camera section
	received_data = ser.read()              # Read serial port
	time.sleep(0.05)						# Wait 0.05 seconds
	data_left = ser.inWaiting()             # Check for remaining data
	received_data += ser.read(data_left) 	# Combine data
	'''
	print (received_data)                   # Print received data
	print (type(received_data))
	'''
	cam_data_decode = received_data.decode()
	cam_data_list = cam_data_decode.split()	# Make data into list
	'''
	print(cam_data_list)	
	print(type(cam_data_list))
	'''
	cam_data = np.zeros(6)
	for i in range(6):
		cam_data[i] = float(cam_data_list[i]) # make data into float
	print(cam_data)
	trans_data = cam_data[:3]
	print(trans_data)
	rot_data = cam_data[3:]
	print(rot_data)
	
# Control law section
	forw_dist = -1*trans_data[2]; 	# Data comes out - so mult by -1
	l_r_dist = trans_data[1];	# Left is - right is +
	print(l_r_dist)
	rot_angle = -1*(rot_data[0]-180); # + is CW - is CCW (data is 0-360)
	print(rot_angle)
	'''
	Step 1: Align with target in l/r direction
	Step 2: Align with target in angle
	Step 3: Drive to target
	'''
	l_r_tol = 1			# Set left right tolerance here
	forw_tol = 0.5		# Set forward tolerance here
	ang_tol = 5			# Set tolerance for angle here
	
	if abs(l_r_dist) >= l_r_tol:	# If outside l/r tolerance fix it
		wait_time = sqrt(abs(l_r_dist))
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
	time.sleep(wait_time)  					# Wait before sending again
