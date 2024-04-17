# Slave is GCS
import time
import socket
import json
from pymavlink import mavutil
import sys

slave = mavutil.mavlink_connection('udpin:0.0.0.0:14445')
slave2 = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
slave2.wait_heartbeat()
slave.wait_heartbeat()

bind_ip = '192.168.2.2'  # Set IP to Companion IP
bind_port = 2345
print("Port set to:", bind_port)
GCS_ID = 255  # GCS ID must be 255, otherwise controller will not work


def SendHB():
    while True:
        hb = slave.recv_match(type="HEARTBEAT", blocking=True)
        bs = slave.recv_match(type="SYS_STATUS", blocking=True)
        converted_hb = hb.to_dict()

        json_data = json.dumps(converted_hb)

        if hb.get_srcSystem() == GCS_ID:
            return hb.get_srcSystem(), json_data, bs


timeout = 0
armed_permission = True


def reset_timeout():
    global timeout, armed_permission

    timeout = 0
    armed_permission = True


def count_timeout(connection):
    global armed_permission, timeout

    if armed_permission:
        timeout += 1

    if timeout > 4 and armed_permission:
        connection.arducopter_disarm()
        print("Vehicle DISARMED")
        armed_permission = False


try:
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("SEARCHING NEW MASTER")

            s.connect((bind_ip, bind_port))
            print("MASTER FOUND")

            while True:
                print("\nSLAVE: ")

                try:
                    QGC_ID, hb, bs = SendHB()
                    if QGC_ID == GCS_ID:
                        print(hb)
                        print(bs)
                        s.sendall(bytes(hb, encoding="utf-8"))

                    server_response = s.recv(1024).decode()

                    if server_response == "ACK":
                        print("Master Says, OK and VERIFFIED")
                        print(hb)
                        reset_timeout()

                    else:
                        raise Exception('Master Didnt Give Permission!')

                    print(f"Received from server: {server_response}")

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

except KeyboardInterrupt:
    s.close()
