#!/usr/bin/env python
# Master is Companion
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')

import json
import socket
import time

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Initialized socket")

# Set the port
bind_ip = '192.168.2.2'
bind_port = 2345
print("Port set to:", bind_port)

# Bind the socket to an address
s.bind((bind_ip, bind_port))
s.listen(5)
#s.setblocking(False)
s.settimeout(5)

print("Socket is waiting for a client")
timeout=0
armed_permission=True

def reset_timeout():
    global timeout, armed_permission
    timeout=0
    armed_permission=True

def count_timeout(connection):
    global armed_permission,timeout
    if armed_permission:
        timeout+=1
    if timeout>5 and armed_permission:
        connection.arducopter_disarm()
        print("Vehicle DISARMED")
        armed_permission=False
try:
    while True:
        try:
            c, addr = s.accept()
            print('Client', addr)

            while True:
                print("MASTER: ")

                try:
                    # Send a thank-you message to the client
                    # c.send('ACK'.encode())
                    received_data = c.recv(1024).decode("utf-8")
                    print("Received JSON: {}".format(received_data))
                    hb = json.loads(received_data)
                    basemode = hb['base_mode']
                    print("Base Mode: ", basemode)
                    print(type(basemode))
                    print("DISARM TIMEOUT:" ,timeout)
                    if hb['base_mode'] == 192:
                        print("Data Acknowledged!")
                        print("Received from client: {}".format(received_data))
                        c.send('ACK'.encode())
                        reset_timeout()
                    else:
                        raise Exception("Cannot Get Data")

                except:
                    reset_timeout()
                    print("SLAVE IS FREE")
                    break
                time.sleep(1)

        except socket.timeout:
            count_timeout(master)
            print("DISARM TIMEOUT", timeout)
            print("SLAVE RAN AWAY")
            print("CONNECTION TIMEOUT")
            # break # <---------------------------------- delete this, just for single tests

except KeyboardInterrupt:
    s.close()
