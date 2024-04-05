# Slave is GCS
import time
import socket
import json
from pymavlink import mavutil
import sys

slave = mavutil.mavlink_connection('udpin:0.0.0.0:14445')
slave.wait_heartbeat()

# Set the port
bind_ip = '192.168.2.2'
bind_port = 2345
print("Port set to:", bind_port)
GCS_ID = 250

def SendHB():
    while True:
        hb = slave.recv_match(type="HEARTBEAT", blocking=True)
        converted_hb = hb.to_dict()
        # print(slave.source_system)

        # print(hb.get_srcSystem())
        # print(hb)
        # print("Param ID: %s" % hb.param_id)
        # print("Message type: ", hb.get_type())
        json_data = json.dumps(converted_hb)
        if hb.get_srcSystem() == GCS_ID:
            return hb.get_srcSystem(), json_data

    # sendhb = slave.mav.heartbeat_send(
    #     9,     # MAVTYPE = MAV_TYPE_GCS
    #     8,      # MAVAUTOPILOT = MAV_AUTOPILOT_INVALID
    #     191,    # MAV_MODE = MAV_MODE_FLAG_SAFETY_ARMED, have also tried 0 here
    #     0,0)    # MAVSTATE =

    # if sendhb:
    #     print("FAULTY SENDING HB")
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
        # Connect to the server
        try:
            # Socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("SEARCHING NEW MASTER")

            # Make connection
            s.connect((bind_ip, bind_port))
            print("MASTER FOUND")

            while True:
                print("\nSLAVE: ")
                
                try:
                    QGC_ID, hb = SendHB()
                    if QGC_ID == GCS_ID:
                        # hb = json.loads(hb)
                        # print("Base Mode: ", hb['base_mode'])
                        # s.send('PING'.encode())
                        # s.send(str(hb).encode())
                        print(hb)
                        s.sendall(bytes(hb, encoding="utf-8"))
                        
                    
                    # Receive data from the server
                    server_response = s.recv(1024).decode()
                    
                    # if hb.param_id == "base_mode":
                    #     print("YEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                    
                    if server_response == "ACK":
                        # send data to server
                        print("Master Says, OK and VERIFFIED")
                        print(hb)
                        reset_timeout()
            
                    else:
                        raise Exception('Master Didnt Give Permission!')
                    
                    print(f"Received from server: {server_response}")

                # except KeyboardInterrupt:
                #     sys.exit(1)
                except:
                    reset_timeout()
                    s.close()
                    print("Connection disconnected")
                    print("FREE FROM SLAVERY")
                    break
                

                time.sleep(1)

        except:
            count_timeout(slave)
            s.close()
            time.sleep(1)
            # break # <------------------------------------------------------ i changed this, pls change it back to pass
except KeyboardInterrupt:
    s.close()
    # sys.exit()
# slave.arducopter_disarm()