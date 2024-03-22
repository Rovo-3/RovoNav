"""
Example of how to set target depth in depth hold mode with pymavlink
"""

"Edited by Jason"

import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase
import matplotlib.pyplot as plt
import numpy as np

time_elapsed = 0
depth_array=np.array([])
pos_x_array = np.array([])
pos_y_array = np.array([])

def plot(depth, pos_x, pos_y):
        global time_elapsed
        global depth_array
        global pos_x_array
        global pos_y_array
        mytime = np.arange(0, time_elapsed, 1)
        depth_array=np.append(depth_array, depth)
        pos_x_array=np.append(pos_x_array, pos_x)
        pos_y_array=np.append(pos_y_array, pos_y)
        my_data = {"Depth":depth_array,
                   "Position X":pos_x_array,
                   "Position Y": pos_y_array
                   }
        print(my_data)
        plt.clf()
        plt.title("Graphs for Position X, Y, Z")
        print(enumerate(my_data.items()))
        for i, (key, array) in enumerate(my_data.items()):
            plt.subplot(len(my_data), 1, i+1)
            plt.plot(mytime[0:time_elapsed], array[0:time_elapsed],
                label=key)
            plt.ylabel('Value (m)')
            plt.legend()

        plt.draw()
        plt.pause(0.05)
        time_elapsed+=1


# Create the connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

depth = alt= lon= lat = 0

while True:
    msg_attitude = master.recv_match(type='ATTITUDE', blocking=False)
    msg_depth = master.recv_match(type='VFR_HUD', blocking=False)
    msg_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg_attitude and msg_depth and msg_position is not None:
            roll = msg_attitude.roll
            pitch = msg_attitude.pitch
            yaw = msg_attitude.yaw 
            depth = msg_depth.alt 
            lat = msg_position.lat
            lon= msg_position.lon
            alt = msg_position.alt
            # print(roll,pitch,yaw,depth)
        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     # Get latitude and longitude in degrees
        #     lat = msg.lat / 1e7
        #     lon = msg.lon / 1e7
        #     # Altitude in meters
        #     alt = msg.alt / 1e3
        #     print("Position (Lat, Lon, Alt):", lat, lon, alt)
            
        # elif msg.get_type() == 'ATTITUDE':
        #     # Roll, pitch, and yaw in radians
        #     roll = msg.roll
        #     pitch = msg.pitch
        #     yaw = msg.yaw
        #     print("Attitude (Roll, Pitch, Yaw):", roll, pitch, yaw)
            
        # elif msg.get_type() == 'VFR_HUD':
        #     # Altitude above ground in meters
        #     alt_agl = msg.alt
            # print("Altitude AGL:", alt_agl)
    plot(depth, lat, lon)
    time.sleep(1)
  