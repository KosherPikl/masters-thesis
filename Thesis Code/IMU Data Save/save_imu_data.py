'''
Tuck Forbes Master's Thesis Code V1
7/31/25  
This code writes accelerometer data to a file and names it based on time
and date
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
	data = bmi160.get_sensor_data() 		# Gather BMI160 data
	print(data)
	# Display BMI160 Data
	# Convert gyro from degrees to radians
	print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%((data['gyro']['x']-data_offset['gyro']['x'])*3.14/180.0, (data['gyro']['y']-data_offset['gyro']['y'])*3.14/180.0, (data['gyro']['z']-data_offset['gyro']['z'])*3.14/180.0))
	# Convert accelerometer data to gravity
	print("accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%((data['accel']['x']-data_offset['accel']['x'])/16384.0, (data['accel']['y']-data_offset['accel']['y'])/16384.0, (data['accel']['z']-data_offset['accel']['z'])/16384.0))
	print("\n")
	
	acc_data = "%f %f %f %f\n"%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0, time.time()-start_time)
	gyro_data = "%f %f %f %f\n"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0, time.time()-start_time)

# Write data to file
	fa.write(acc_data)
	fg.write(gyro_data)
