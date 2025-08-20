import time                         # Import time library
from board import SCL, SDA          # Import the PCA9685 module.
import busio                        # Import I2C library
from adafruit_pca9685 import PCA9685# Import the PCA9685 module.
import socket                       # Import UDP library
import RPi.GPIO as GPIO             # Import GPIO pin library


i2c_bus = busio.I2C(SCL, SDA)       # Create the I2C bus interface.

pwm =  PCA9685(i2c_bus) # Initialise the PCA9685 using the default address (0x40).

move_speed = 0x2FFF  # hexadecimal, Max pulse length out of 0xFFFF

UDP_IP = "192.168.1.155"            # receiver IP address
UDP_Port = 5555                     # port for data reception
sock = socket.socket(socket.AF_INET,# Internet
    socket.SOCK_DGRAM)              # UDP
	
sock.bind((UDP_IP,UDP_Port))        # Bind UDP Port

#time_run = 0.1 # Time delay between signals

# Set frequency to 60hz, good for servos.
pwm.frequency = 60
GPIO.setmode(GPIO.BCM) # GPIO number in BCM mode
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

command = "Stop"

while True:
<<<<<<< HEAD
    try :
        data,addr = sock.recvfrom(1024) # receive data
    except :
        command = command
    else :
        command = data.decode() # unpack data
    finally: 
        if command == "Forward":
            print("Forward") 
            go_ahead(move_speed)
            
        elif command == "Stop":
            print("Stop")
            stop_car()
=======
    data,addr = sock.recvfrom(1024) # data size and receive data
    command = data.decode() # data type and unpack data here
>>>>>>> cce141ed0c0f9ad73c23014ba6f98030fcab7e3a

        elif command == "Back":
            print("Back")
            go_back(move_speed)


        elif command == "Right":
            print("Right")
            shift_right(move_speed)


        elif command == "Left":
            print("Left") 
            shift_left(move_speed)

        elif command == "CW":
            print("CW")
            turn_right(move_speed)

        elif command == "CCW":
            print("CCW")
            turn_left(move_speed)


        elif command == "UpR":
            print("UpR")
            upper_right(move_speed)


        elif command == "UpL":
            print("UpL")
            upper_left(move_speed)


        elif command == "LowR":
            print("LowR")
            lower_right(move_speed)


        else command == "LowL":
            print("LowL")
            lower_left(move_speed)
