import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm =  PCA9685(i2c_bus)

move_speed = 0x2FFF  # half of Max pulse length out of 0xFFFF

# Set frequency to 60hz, good for servos.
pwm.frequency = 60
GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
GPIO.setwarnings(False)
#define L298N(Model-Pi motor drive board) GPIO pins
IN1_FRONT = 23  #FRONT Right  motor(BK1 port) direction pin
IN2_FRONT = 24  #FRONT Right  motor(BK1 port) direction pin
IN3_FRONT = 27  #FRONT Left motor(BK3 port) direction pin
IN4_FRONT = 22  #FRONT Left motor(BK3 port) direction pin
ENA_FRONT = 0 	#ENA_FRONT _B area Left motor speed PCA9685 port 0
ENB_FRONT = 1	#ENB_FRONT _B area Right motor speed PCA9685 port 1

IN1_REAR = 21  #Rear Right  motor(AK1 port) direction pin
IN2_REAR = 20  #Rear Right  motor(AK1 port) direction pin
IN3_REAR = 16  #Rear Left motor(AK3 port) direction pin
IN4_REAR = 12  #Rear Left motor(AK3 port) direction pin
ENA_REAR = 2 	#ENA_REAR _A area Left motor speed PCA9685 port 2
ENB_REAR = 3	#ENB_REAR _A area Right motor speed PCA9685 port 3
# Define motor control  pins as output
GPIO.setup(IN1_FRONT, GPIO.OUT)   
GPIO.setup(IN2_FRONT, GPIO.OUT) 
GPIO.setup(IN3_FRONT, GPIO.OUT)   
GPIO.setup(IN4_FRONT, GPIO.OUT) 
GPIO.setup(IN1_REAR, GPIO.OUT)   
GPIO.setup(IN2_REAR, GPIO.OUT) 
GPIO.setup(IN3_REAR, GPIO.OUT)   
GPIO.setup(IN4_REAR, GPIO.OUT) 
def changespeed(speed):
    pwm.channels[ENA_FRONT].duty_cycle = speed
    pwm.channels[ENB_FRONT].duty_cycle = speed
    pwm.channels[ENA_REAR].duty_cycle = speed
    pwm.channels[ENB_REAR].duty_cycle = speed

def stop_car():
    GPIO.output(IN1_FRONT, GPIO.LOW)
    GPIO.output(IN2_FRONT, GPIO.LOW)
    GPIO.output(IN3_FRONT, GPIO.LOW)
    GPIO.output(IN4_FRONT, GPIO.LOW)
    GPIO.output(IN1_REAR, GPIO.LOW)
    GPIO.output(IN2_REAR, GPIO.LOW)
    GPIO.output(IN3_REAR, GPIO.LOW)
    GPIO.output(IN4_REAR, GPIO.LOW)
    changespeed(0)
    
def rr_ahead():
    GPIO.output(IN1_REAR,GPIO.HIGH)
    GPIO.output(IN2_REAR,GPIO.LOW)
    #pwm.channels[ENA_REAR].duty_cycle = speed
    
def rr_back():
    GPIO.output(IN1_REAR,GPIO.LOW)
    GPIO.output(IN2_REAR,GPIO.HIGH)
    #pwm.channels[ENA_REAR].duty_cycle = speed
    
def rl_ahead():  
    GPIO.output(IN3_REAR,GPIO.HIGH)
    GPIO.output(IN4_REAR,GPIO.LOW)
    #pwm.channels[ENB_REAR].duty_cycle = speed   

def rl_back():  
    GPIO.output(IN3_REAR,GPIO.LOW)
    GPIO.output(IN4_REAR,GPIO.HIGH)
    #pwm.channels[ENB_REAR].duty_cycle = speed   

def fr_ahead():
    GPIO.output(IN1_FRONT,GPIO.HIGH)
    GPIO.output(IN2_FRONT,GPIO.LOW)
    #pwm.channels[ENA_FRONT].duty_cycle = speed
    
def fr_back():
    GPIO.output(IN1_FRONT,GPIO.LOW)
    GPIO.output(IN2_FRONT,GPIO.HIGH)
    #pwm.channels[ENA_FRONT].duty_cycle = speed
    
def fl_ahead():  
    GPIO.output(IN3_FRONT,GPIO.HIGH)
    GPIO.output(IN4_FRONT,GPIO.LOW)
    #pwm.channels[ENB_FRONT].duty_cycle = speed   

def fl_back():  
    GPIO.output(IN3_FRONT,GPIO.LOW)
    GPIO.output(IN4_FRONT,GPIO.HIGH)
    #pwm.channels[ENB_FRONT].duty_cycle = speed  

def go_ahead(speed):
    rl_ahead()
    rr_ahead()
    fl_ahead()
    fr_ahead()
    changespeed(speed)
    
def go_back(speed):
    rr_back()
    rl_back()
    fr_back()
    fl_back()
    changespeed(speed)

#making right turn   
def turn_right(speed):
    rl_ahead()
    rr_back()
    fl_ahead()
    fr_back()
    changespeed(speed)
      
#make left turn
def turn_left(speed):
    rr_ahead()
    rl_back()
    fr_ahead()
    fl_back()
    changespeed(speed)

# parallel left shift 
def shift_left(speed):
    fr_ahead()
    rr_back()
    rl_ahead()
    fl_back()
    changespeed(speed)
    
# parallel right shift 
def shift_right(speed):
    fr_back()
    rr_ahead()
    rl_back()
    fl_ahead()
    changespeed(speed)
      
def upper_right(speed):
    rr_ahead()
    fl_ahead()
    changespeed(speed)

def lower_left(speed):
    rr_back()
    fl_back()
    changespeed(speed)
    
def upper_left(speed):
    fr_ahead()
    rl_ahead()
    changespeed(speed)
    
def lower_right(speed):
    fr_back()
    rl_back()
    changespeed(speed)

print('ahead...')

go_ahead(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('backward...')
go_back(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

shift_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

shift_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

upper_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

lower_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

upper_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

lower_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

turn_left(move_speed)
time.sleep(1)
stop_car()

turn_right(move_speed)
time.sleep(1)
stop_car()

GPIO.cleanup()    

print('press Ctrl-C to quit...')
#while True:
# Move servo on channel O between extremes.
 
