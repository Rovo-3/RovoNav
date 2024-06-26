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

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
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


def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )


# Create the connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print("armed")

# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)
alt= lon= lat = 0

set_target_depth(-1)
while True:
    msg = master.recv_match()
    if msg is not None:
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            # Get latitude and longitude in degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            # Altitude in meters
            alt = msg.alt / 1e3
            print("Position (Lat, Lon, Alt):", lat, lon, alt)
            
        elif msg.get_type() == 'ATTITUDE':
            # Roll, pitch, and yaw in radians
            roll = msg.roll
            pitch = msg.pitch
            yaw = msg.yaw
            print("Attitude (Roll, Pitch, Yaw):", roll, pitch, yaw)
            
        elif msg.get_type() == 'VFR_HUD':
            # Altitude above ground in meters
            alt_agl = msg.alt
            print("Altitude AGL:", alt_agl)
        # alt+=1
        plot(alt, lat, lon)
    time.sleep(1)
     
# set a depth target


time.sleep(2)

# go for a spin
# (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
roll_angle = 0
pitch_angle = 40
yaw_angle = 0

set_target_attitude(roll_angle, pitch_angle, yaw_angle)
time.sleep(2)
print("setting alt test")

depth = 0.0
setpoint = -5

# while depth > setpoint:
set_rc_channel_pwm(5, 1400)

for depth in range(-1, -20, -1):
    set_target_depth(depth)
    set_target_attitude(roll_angle, pitch_angle, yaw_angle)
    time.sleep(2)  # wait for a second
    # depth += 0.1

set_rc_channel_pwm(5, 1500)

# for yaw_angle in range(0, 500, 10):
#     set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#     time.sleep(1) # wait for a second

# # spin the other way with 3x larger steps
# for yaw_angle in range(500, 0, -30):
#     set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#     time.sleep(1)

# clean up (disarm) at the end
master.arducopter_disarm()
master.motors_disarmed_wait()
