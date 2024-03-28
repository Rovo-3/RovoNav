# Master is Companion
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')

import socket
import time

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Initialized socket")

# Set the port
bind_ip = '192.168.2.2'
bind_port = 8986
print("Port set to:", bind_port)

# Bind the socket to an address
s.bind((bind_ip, bind_port))
s.listen(5)
s.setblocking(False)
s.settimeout(5)

print("Socket is waiting for a client")

try:
    while True:
        try:
            c, addr = s.accept()
            print('Client', addr)

            while True:
                print("MASTER: ")

                try:
                    # Send a thank-you message to the client
                    c.send('PING'.encode())
                    received_data = c.recv(1024).decode()
                    if received_data == "ACK":
                        print("Data Acknowledged!")
                        print("Received from client: {}".format(received_data))
                    else:
                        master.arducopter_disarm()
                        print("VEHICLE: DISARM")

                except:
                    print("SLAVE IS FREE")
                    break
                time.sleep(1)
            
        except socket.timeout:
            print("SLAVE RAN AWAY")
            print("CONNECTION TIMEOUT")

except KeyboardInterrupt:
    s.close()
