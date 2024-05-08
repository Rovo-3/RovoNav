from pymavlink import mavutil
import time
import math
from pymavlink.quaternion import QuaternionBase

UdpIpPort = 'udp:0.0.0.0:14445' 

conn = mavutil.mavlink_connection(UdpIpPort)
conn.wait_heartbeat()

boot_time = time.time()

def set_target_attitude(master, roll, pitch, yaw):
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

def GetIMU():
    sensorData = conn.recv_match(type="ATTITUDE", blocking=True)

    return sensorData.to_dict()

def GetVFRHUD():
    bs = conn.recv_match(type="VFR_HUD", blocking=True)

    return bs.to_dict()

print("roll(degree), pitch(degree), yaw(degree), rollspeed(w), pitchspeed(w), yawspeed(w), airspeed(m/s), groundspeed(m/s), heading(degree), altitude(m), climbspeed(m/s)")

while True:
    try: 
        imu = GetIMU()
        compass = GetVFRHUD()
        print("%.7f, %.7f, %.3f, %.7f, %.7f, %.7f, %.3f, %.3f, %d, %.3f, %.3f" % 
              (imu['roll']*180/3.14, imu['pitch']*180/3.14, imu['yaw']*180/3.14, imu['rollspeed']*180/3.14, imu['pitchspeed']*180/3.14, imu['yawspeed']*180/3.14, 
               compass['airspeed'], compass['groundspeed'], compass['heading'], compass['alt'], compass['climb']))
        # print()
        # print(imu, compass)

    except KeyboardInterrupt:
        break
    
    
    
    
# introducing our surveyor ROV, Rovocean and Rovo-3
# ready to explore the depths with Rovocean and Rovo-3

