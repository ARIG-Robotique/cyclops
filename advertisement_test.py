import socket
from threading import Thread
from time import sleep
import sys
import errno

exit = False

def rxThread(portNum):
	global exit
	
	#Generate a UDP socket
	rxSocket = socket.socket(socket.AF_INET, #Internet
							 socket.SOCK_DGRAM) #UDP
							 
	#Bind to any available address on port *portNum*
	rxSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	rxSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	rxSocket.bind(("",portNum))
	
	#Prevent the socket from blocking until it receives all the data it wants
	#Note: Instead of blocking, it will throw a socket.error exception if it
	#doesn't get any data
	
	rxSocket.setblocking(0)
	
	print("RX: Receiving data on UDP port " + str(portNum))
	print("")
	
	while not exit:
		try:
			#Attempt to receive up to 1024 bytes of data
			data,addr = rxSocket.recvfrom(1024) 
			#Echo the data back to the sender
			#rxSocket.sendto(str(data),addr)
		except socket.error as err: 
			pass
		else:
			print(data)
		sleep(.1)
	
	
def main():    
	global exit
	print( "UDP Tx/Rx Example application")
	print( "Press Ctrl+C to exit")
	print( "")
	
	portNum = 50666
   
	udpRxThreadHandle = Thread(target=rxThread,args=(portNum,))    
	udpRxThreadHandle.start()
		
	sleep(.1)
	
	while True:
		try:
			 #Retrieve input data 
			txChar = input("TX: ")
		except KeyboardInterrupt:
			exit = True
			print( "Received Ctrl+C... initiating exit")
			break
		sleep(.1)
		 
	udpRxThreadHandle.join()
		
	return

if __name__=="__main__":
	main()