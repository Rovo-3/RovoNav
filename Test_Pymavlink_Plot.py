"""
Example of how to set target depth in depth hold mode with pymavlink
"""

"Edited by Jason"
import utm
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
pitch_array = np.array([])

def plot(depth, pos_x, pos_y, pitch):
        global time_elapsed
        global depth_array
        global pos_x_array
        global pos_y_array
        global pitch_array
        mytime = np.arange(0, time_elapsed, 1)
        depth_array=np.append(depth_array, depth)
        pos_x_array=np.append(pos_x_array, pos_x)
        pos_y_array=np.append(pos_y_array, pos_y)
        pitch_array=np.append(pitch_array, pitch)
        my_data = {"Depth":depth_array,
                   "Position X":pos_x_array,
                   "Position Y": pos_y_array,
                   "Pitch": pitch_array
                   }
#        print(my_data)
        
        plt.clf()
        plt.suptitle("Graphs for Position X, Y, Z, and Pitch")
        # plt.cla()
        
        
#        print(enumerate(my_data.items()))
        for i, (key, array) in enumerate(my_data.items()):
            plt.subplot(len(my_data), 1, i+1)
            plt.plot(mytime[0:time_elapsed], array[0:time_elapsed],
                label=key)
            plt.ylabel('Value (m)')
            plt.legend()
        
        plt.draw()
        plt.pause(0.1)
        time_elapsed+=1


# Create the connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14445')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

depth = pitch = alt = lon = lat = 0
initial_x_position = initial_y_position = 0
first = True
x = y = 0
while True:
    #msg_attitude = master.recv_match(type='ATTITUDE', blocking=False)
    #msg_depth = master.recv_match(type='VFR_HUD', blocking=False)
    #try:
    #    msg = master.recv_match().to_dict()
    #    print(msg)
    #except:
    #    pass
    
    msg_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg_attitude = master.recv_match(type='ATTITUDE', blocking=True)
    msg_depth = master.recv_match(type='VFR_HUD', blocking=False)    
    try:
        roll = msg_attitude.roll
        pitch = msg_attitude.pitch*180/3.14
        yaw = msg_attitude.yaw
    except:
        pass
    try:
        depth = msg_position.alt / 1e3
        lat = msg_position.lat / 1e7
        lon= msg_position.lon / 1e7
        u = utm.from_latlon(lat,lon)
        if first:
            initial_x_position, initial_y_position = (u[0], u[1])
        x=u[0] - initial_x_position
        y=u[1] - initial_y_position
        first = False
        print(x,y)
    except:
        pass
    try:
        alt = msg_depth.alt 
    except:
        pass
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
    plot(depth, x, y, pitch)
  
