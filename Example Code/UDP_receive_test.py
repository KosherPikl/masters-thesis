import socket
import time

UDP_IP = "192.168.1.155" # receiver IP address
UDP_Port = 5555 # port for data reception
sock = socket.socket(socket.AF_INET,  # Internet
	socket.SOCK_DGRAM) # UDP
	
sock.bind((UDP_IP,UDP_Port))

while True:
	data,addr = sock.recvfrom(1024) # data size and receive data
	unpacked_data = data.decode() # data type and unpack data here
	print(f"received message: {unpacked_data}")
	time.sleep(0.1)
