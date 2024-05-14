# import necessary packages
from pyjoystick.sdl2 import Key, run_event_loop, stop_event_wait
import threading
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import json
import sys
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
        """
        Changing the mode of ROV. \n
        Args is the mode of ROV in string ex. MANUAL, POS_HOLD, etc
        """ 
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
        self.is_shift_key = False
        """
        you can modify the shift key and the combination key for vertical mode activation
        those key will change the value of activate_vertical_mode
        rpy key can also be changed. For now yawing will use key number 0, while pitching will use 2 and 5.
        """
        self.shift_key = 10
        self.vertical_mode_key = 1
        self.rpy_key = {
            "rpy_button":"Axis",
            "yaw_key_number":0,
            "pitch_down_key_number":2,
            "pitch_up_key_number":5,
        }
        self.activate_vertical_mode = False
        self.json_path = "C:/Users/Admin/Desktop/json_data/verticalMode.json"
        self.vertical_data={
            "isVerticalActive" : 0
        }
    def print_add(self, joy):
        print('Added', joy)

    def print_remove(self, joy):
        print('Removed', joy)

    def key_received(self, key):
        # checking shift key
        self.check_shift(key.keytype, key.number, key.value)
        print("Checking shift key status: ", self.is_shift_key)
            
        self.check_combination(key.keytype, key.number, key.value)
        print("Does vertical mode active? ", self.activate_vertical_mode)
        self.activate_vertical_mode=self.read_json()

        if self.activate_vertical_mode:
            self.vertical_operation(key)
        
        # # Uncomment this code to get the joystick keytype, number, and value
        # print('received', key)
        # print('key type', key.keytype)
        # print('key number', key.number)
        # print('key value', key.value)
    def update_yaw(self):
        """
        Updating yaw_desired and fixing it so that 0 <= yaw_desired < 360
        """
        self.yaw_desired+=self.yaw_changes
        if self.yaw_desired>=360:
            self.yaw_desired-=360
        if self.yaw_desired<0:
            self.yaw_desired+=360
        return self.yaw_desired
    
    def update_pitch(self):
        """
        Updating pitch_desired and fixing it desired so that -110 <= pitch_desired <= 110
        """
        self.pitch_desired += self.pitch_changes
        if self.pitch_desired>110:
            self.pitch_desired=110
        if self.pitch_desired<-110:
            self.pitch_desired=-110
        return self.pitch_desired
    
    def check_shift(self, type, number, value):
        """
        Checking the shift button is pressed, it will change the is shift key to True. \n
        If other button is pressed then it will make deactivate vertical_mode
        """
        if type == "Button" and number == self.shift_key and value == 1:
            self.is_shift_key=True
        elif type == "Button" and number == self.shift_key and value == 0: 
            self.is_shift_key=False
        elif type == "Button" and number != self.shift_key and value == 1:
            self.activate_vertical_mode=False
            self.modify_json(self.activate_vertical_mode)

    def check_combination(self, type, number, value):
        """
        Checking the combination. It will only triggered when the shift_key is pressed.
        """
        print("Checking the combination")
        if self.is_shift_key == False:
            print("Shift Key is not pressed")
            return
        print("Shift Key is pressed! checking the combination key")
        if type == "Button" and number == self.vertical_mode_key and value == 1:
            self.activate_vertical_mode=not(self.activate_vertical_mode)
        elif type =="Button" and number != self.vertical_mode_key and value ==1:
            self.activate_vertical_mode=False
        self.modify_json(self.activate_vertical_mode)

    def vertical_operation(self, key):
        """
        Handle the vertical operation key
        """
        max_changes = 5
        if key.keytype == self.rpy_key["rpy_button"]:
            if key.number == self.rpy_key["yaw_key_number"]:
                if abs(key.value)>0.2:
                    self.yaw_changes=max_changes*key.value
                # elif key.value<-0.2:
                #     self.yaw_changes=max_changes*key.value
                else:
                    self.yaw_changes=0
                print("yaw changes ", self.yaw_changes)
                self.yaw_changes = int(self.yaw_changes)
            if key.number == self.rpy_key["pitch_down_key_number"]:
                if abs(key.value)>0.2:
                    self.pitch_changes=-1*max_changes*key.value
                else:
                    self.pitch_changes=0
                print("pitch changes ", self.pitch_changes)
                self.pitch_changes = int(self.pitch_changes)
                self.status = "PITCH_DOWN"
            if key.number == self.rpy_key["pitch_up_key_number"]:
                if key.value>0.2:
                    self.pitch_changes=max_changes*key.value
                else:
                    self.pitch_changes=0
                print("pitch changes ", self.pitch_changes)
                self.pitch_changes = int(self.pitch_changes)
                self.status = "PITCH_UP"
            if self.rpy_key["pitch_down_key_number"] or self.rpy_key["pitch_up_key_number"]:
                if abs(key.value)<0.2:
                    self.pitch_changes=0
                    self.status = "HOLD"
    def read_json(self):
        print("reading JSON file")
        try:
            with open(self.json_path, "r") as file:
                self.vertical_data = json.load(file)
            return self.vertical_data["isVerticalActive"]
        except Exception as e:
            print(e)
            print(f"File '{self.json_path}' not found. Waiting for it to be created...")
            with open(self.json_path, "w") as file:
                json.dump(self.vertical_data, file)
            print("JSON file is created")

    def modify_json(self, value):
        with open(self.json_path, "w") as file:
            self.vertical_data["isVerticalActive"]=int(value)
            json.dump(self.vertical_data,file)
        print("JSON file is modified")

if __name__ == "__main__":
    # setting up sub mavlink connection
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    # master = mavutil.mavlink_connection('udp:0.0.0.0:12346')
    boot_time = time.time()
    # instantiation for joystick event and for the ROV
    my_joystick = My_joystick()
    my_sub = MySub(master_connection=master, boot_time=boot_time)
    my_sub.wait_heartbeat()
    # make a thread for the joystick event
    thread1 = threading.Thread(target=run_event_loop, args=(my_joystick.print_add, my_joystick.print_remove, my_joystick.key_received,))
    thread1.start()
    try:
        while True:
            # print("here we are on main loop")
            # This mode is trigerred by using shift key (button on the center, key number 10) with B key (key number 1)
            vertical_mode_status = my_joystick.read_json()
            # my_sub.compare_json(vertical_mode_status, my_joystick.activate_vertical_mode)

            if vertical_mode_status:
                my_sub.change_mode("ALT_HOLD")
                pitch_desired = my_joystick.update_pitch()
                yaw_desired = my_joystick.update_yaw()
                roll_desired = 0
                # Setting the rpy by the desired setpoint on default 0,90,0
                my_sub.set_target_attitude(roll_desired, pitch_desired, yaw_desired)
                print("desired rpy :", roll_desired,pitch_desired,yaw_desired)
                time.sleep(0.5)
                # passing next code and go back to the new loop
                pass
            time.sleep(1)
            print("Waiting for the custom mode command")
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, stopping threads...")
        my_sub.disarming()     
        # sys.exit(0)
        os._exit(1)
        # print("All threads have finished.")
