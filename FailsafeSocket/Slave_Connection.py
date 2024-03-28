#Slave is GCS
from pymavlink import mavutil

slave = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
slave.wait_heartbeat()
import socket
import time 

# Set the port
bind_ip = '192.168.2.2'
bind_port = 8986
print("Port set to:", bind_port)

try:
    while True:
        # Connect to the server
        try: 
            # Socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("SEARCHING NEW MASTER")
            
            # Make connection
            s.connect((bind_ip, bind_port))
            print("MASTER FOUND")
        
            while True:
                print("SLAVE: ")
                try:
                    # Receive data from the server
                    server_response = s.recv(1024).decode()
                    if server_response =="PING":
                        # send data to server
                        s.send('ACK'.encode())
                    else:
                        raise Exception('I dont get PING!')
                    print(f"Received from server: {server_response}")
                    
                except:
                    s.close()
                    print("Connection disconnected")
                    print("FREE FROM SLAVERY")
                    break
                
                time.sleep(1)
                
        except:
            slave.arducopter_disarm()
            s.close()
            time.sleep(1)
            pass
except KeyboardInterrupt:
    s.close()
