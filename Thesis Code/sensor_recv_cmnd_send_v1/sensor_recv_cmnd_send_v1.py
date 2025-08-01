'''
Tuck Forbes Master's Thesis Code V1
8/1/25  
This code receives sensor data and sends out commands to the other 
raspberry pi. The sensors used are BMI160 and RT1062 camera. The commands
sent are strings of data.
'''


import socket					# UDP library
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
	data = bmi160.get_sensor_data() 		# Gather BMI160 data
	# Display BMI160 Data
	# Convert gyro from degrees to radians
	print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0))
	# Convert accelerometer data to gravity
	print("accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0))
	print("\n")
	
# Camera section
	received_data = ser.read()              # Read serial port
	time.sleep(0.05)						# Wait 0.05 seconds
	data_left = ser.inWaiting()             # Check for remaining data
	received_data += ser.read(data_left) 	# Combine data
	print (received_data)                   # Print received data
	
# Command send section
	message = "Left" 						# Write Command
	sock.sendto(message.encode(), (UDP_IP, UDP_Port)) # Send Command
	print(f"Sent: {message} to {UDP_IP}:{UDP_Port}") # Confirm send
	time.sleep(0.1)  							# Wait 0.1 seconds
