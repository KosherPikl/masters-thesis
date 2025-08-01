import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

#from pynput import keyboard
# Imports library to allow keyboard presses

import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm =  PCA9685(i2c_bus)

move_speed = 0x2FFF  # hexadecimal, Max pulse length out of 0xFFFF

#time = 0.5 # Time delay between signals

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


"""
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
"""
'''#while True:
# Move servo on channel O between extremes.
while True:
    event = keyboard.read_event() # Reads the keyboard
    if event.event_type == keyboard.KEY_UP:
        print('forward...')
        go_ahead(move_speed)
        time.sleep(0.1)
    if event.event_type == keyboard.KEY_DOWN:
        print('back...')
        go_back(move_speed)
        time.sleep(0.1)
    if event.name == 'esc':  # Stop on 'Esc' key
        break
        
    else:
        print('stop...')
        stop_car()
        time.sleep(0.1)
        '''
'''
def on_press(key):
    try:
        print(f'Key {key.char} pressed')
        variable = key.char
        #print(type(variable))
    except AttributeError:
        print(f'Special key {key} pressed')
        variable = "blank"
    if variable == "1":
        print(f'Forward') 
        go_ahead(move_speed)
    if variable == "2":
        print(f'Back')
        go_back(move_speed)
    if variable == "3":
        print(f'Right')
        shift_right(move_speed)
    if variable == "4":
        print(f'Left') 
        shift_left(move_speed)
    if variable == "5":
        print(f'CW')
        turn_right(move_speed)
    if variable == "6":
        print(f'CCW')
        turn_left(move_speed)
    if variable == "7":
        print(f'UpR')
        upper_right(move_speed)
    if variable == "8":
        print(f'UpL')
        upper_left(move_speed)
    if variable == "9":
        print(f'LowR')
        lower_right(move_speed)
    if variable == "0":
        print(f'LowL')
        lower_left(move_speed)

def on_release(key):
    print(f'Key {key} released')
    print(f'Stop')
    if key == keyboard.Key.esc:  # Stop listener when 'Esc' is pressed
        return False

# Start listening
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
'''

'''
while True:
    str = input()
    if str.lower == "forward":
        print(f'Forward') 
        go_ahead(move_speed)
    if str == "2":
        print(f'Back')
        go_back(move_speed)
    if str == "3":
        print(f'Right')
        shift_right(move_speed)
    if str == "4":
        print(f'Left') 
        shift_left(move_speed)
    if str == "5":
        print(f'CW')
        turn_right(move_speed)
    if str == "6":
        print(f'CCW')
        turn_left(move_speed)
    if str == "7":
        print(f'UpR')
        upper_right(move_speed)
    if str == "8":
        print(f'UpL')
        upper_left(move_speed)
    if str == "9":
        print(f'LowR')
        lower_right(move_speed)
    if str == "0":
        print(f'LowL')
        lower_left(move_speed)
'''
'''
while True:
    str = input()
    if str.lower() == "forward":
        print(f'Forward') 
        go_ahead(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "2":
        print(f'Back')
        go_back(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "3":
        print(f'Right')
        shift_right(move_speed)        
        time.sleep(1)
        stop_car()
    if str.lower() == "4":
        print(f'Left') 
        shift_left(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "5":
        print(f'CW')
        turn_right(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "6":
        print(f'CCW')
        turn_left(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "7":
        print(f'UpR')
        upper_right(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "8":
        print(f'UpL')
        upper_left(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "9":
        print(f'LowR')
        lower_right(move_speed)
        time.sleep(1)
        stop_car()
    if str.lower() == "0":
        print(f'LowL')
        lower_left(move_speed)
        time.sleep(1)
        stop_car()
        '''
while True:
    command = input("Enter command: ").lower().strip()

    if command == "1":
        print("Forward") 
        go_ahead(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "2":
        print("Back")
        go_back(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "3":
        print("Right")
        shift_right(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "4":
        print("Left") 
        shift_left(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "5":
        print("CW")
        turn_right(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "6":
        print("CCW")
        turn_left(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "7":
        print("UpR")
        upper_right(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "8":
        print("UpL")
        upper_left(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "9":
        print("LowR")
        lower_right(move_speed)
        time.sleep(1)
        stop_car()

    elif command == "0":
        print("LowL")
        lower_left(move_speed)
        time.sleep(1)
        stop_car()
