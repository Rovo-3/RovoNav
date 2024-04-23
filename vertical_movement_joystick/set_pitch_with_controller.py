# import necessary packages
from pyjoystick.sdl2 import Key, run_event_loop
import threading
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
# import matplotlib.pyplot as plt
# import numpy as np
# import sys
import os

# classes
class MySub:
    def __init__(self, master_connection, boot_time):
        print("Initialization of MySub")
        self.__str__="MySub Class"
        self.mode = "MANUAL"
        self.master=master_connection
        self.boot_time = boot_time
        self.is_armed = False
        # return self.master

    def wait_heartbeat(self):
        self.master.wait_heartbeat()
        print("Got Heartbeat!")

    def arming(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        self.is_armed=True
        print("Armed")

    def disarming(self):
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        self.is_armed=False
        print("Disarmed")

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        # flag manual check
        # if self.mode != "MANUAL":
        #     print("Cannot set each channel of thruster other than manual mode")
        #     return
        try:
            if channel_id<1 or channel_id>18:
                print("Channel does not exist")
                return
            rc_channel_values = [65535 for _ in range(18)]
            rc_channel_values[channel_id-1]=pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values #unpack the rc_channel_values
            )
        except:
            print("Failed to set the RC channel PWM")
    def set_target_attitude(self, roll,pitch,yaw):
        """ Sets the target attitude while in depth-hold mode.
        Args
        'roll', 'pitch', and 'yaw' are angles in degrees.

        """
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            self.master.target_system, self.master.target_component,
            # allow throttle to be controlled by depth_hold mode
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
        )
    def change_mode(self,mode):
        self.mode=mode
        if self.is_armed == False:
            self.arming()
        mode_now = self.master.mode_mapping()[self.mode]
        while not self.master.wait_heartbeat().custom_mode == mode_now:
            self.master.set_mode(self.mode)
            print("Mode changed to ", self.mode)
    
class StoppableThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()
        print("Initialization of StopEvent")

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
    
class My_joystick:
    def __init__(self,yaw_desired=0, pitch_desired=90):
        print("Initialization of My_joystick")
        self.yaw_desired = yaw_desired
        self.yaw_changes = 0
        self.pitch_desired = pitch_desired
        self.pitch_changes = 0
        self.status="HOLD"
    def print_add(self, joy):
        print('Added', joy)

    def print_remove(self, joy):
        print('Removed', joy)

    def key_received(self, key):
        # please add the the desired_yaw modifier
        max_changes = 5
        if key.keytype == "Axis":
            if key.number == 0:
                if abs(key.value)>0.2:
                    self.yaw_changes=max_changes*key.value
                # elif key.value<-0.2:
                #     self.yaw_changes=max_changes*key.value
                else:
                    self.yaw_changes=0
                print("yaw changes ", self.yaw_changes)
                self.yaw_changes = int(self.yaw_changes)
            if key.number == 2:
                if abs(key.value)>0.2:
                    self.pitch_changes=-1*max_changes*key.value
                else:
                    self.pitch_changes=0
                print("pitch changes ", self.pitch_changes)
                # print("You hit the Axis!!")
                self.pitch_changes = int(self.pitch_changes)
                self.status = "PITCH_DOWN"
            if key.number == 5:
                if key.value>0.2:
                    self.pitch_changes=max_changes*key.value
                else:
                    self.pitch_changes=0
                print("pitch changes ", self.pitch_changes)
                self.pitch_changes = int(self.pitch_changes)
                self.status = "PITCH_UP"
            if key.number == 2 or key.number == 5:
                if abs(key.value)<0.2:
                    self.status = "HOLD"
        # if key.keytype =="-Axis":
        print('received', key)
        print('key type', key.keytype)
        print('key value', key.value)
        print('key number', key.number)
    def update_yaw(self):
        self.yaw_desired+=self.yaw_changes
        if self.yaw_desired>360:
            self.yaw_desired-=360
        if self.yaw_desired<-360:
            self.yaw_desired+=360
        return self.yaw_desired
    def update_pitch(self):
        self.pitch_desired += self.pitch_changes
        if self.pitch_desired>110:
            self.pitch_desired=110
        if self.pitch_desired<-110:
            self.pitch_desired=-110
        return self.pitch_desired

if __name__ == "__main__":
    # setting up sub
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    # master = mavutil.mavlink_connection('udp:0.0.0.0:12346')
    boot_time = time.time()
    joystick_event = My_joystick()
    my_sub = MySub(master_connection=master, boot_time=boot_time)
    my_sub.wait_heartbeat()
    # my_sub.arming()
    thread1 = threading.Thread(target=run_event_loop, args=(joystick_event.print_add, joystick_event.print_remove, joystick_event.key_received,))
    # thread2 = threading.Thread(target=otherloop, args=(stop_event,))
    thread1.start()
    # thread2.start()
    my_sub.change_mode("ALT_HOLD")
    try:
        while True:
            print("here we are on main loop")
            # ADD PITCH CONTROL 90 DEGREE
            # update the desired yaw value
            pitch_desired = joystick_event.update_pitch()
            yaw_desired = joystick_event.update_yaw()
            roll_desired = 0
            my_sub.set_target_attitude(roll_desired, pitch_desired, yaw_desired)

            print("desired rpy :", roll_desired,pitch_desired,yaw_desired)
            # ADD YAW DESIRED CHANGE BY JOYSTICK
            time.sleep(0.5)
            # if(joystick_event.status == "PITCH_UP"):
            #     my_sub.set_rc_channel_pwm(1,1600)
            # elif(joystick_event.status == "PITCH_DOWN"):
            #     my_sub.set_rc_channel_pwm(1,100)
            # else:
            #     my_sub.set_rc_channel_pwm(1,1500)
            # print(joystick_event.status)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, stopping threads...")
        # stop_event.stop()
        my_sub.disarming()
        os._exit(1)
        # print("All threads have finished.")
