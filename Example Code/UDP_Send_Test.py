'''
Example adapted from https://wiki.python.org/moin/UdpCommunication
'''

import socket
import time

UDP_IP = "192.168.1.155" # receiver IP address

UDP_Port = 5555 # where you will receive data to

sock = socket.socket(socket.AF_INET,# Internet 
	socket.SOCK_DGRAM) # UDP
	
try:
    while True:
        message = "Left"
        sock.sendto(message.encode(), (UDP_IP, UDP_Port))
        print(f"Sent: {message} to {UDP_IP}:{UDP_Port}")
        time.sleep(2)  # Send commands every 2 seconds
except KeyboardInterrupt:
    print("UDP client stopped.")
finally:
    sock.close() # Close the socket when done
