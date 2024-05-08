# Open or create a log file in write mode
import datetime
import time
from pymavlink import mavutil
import time
import math
from pymavlink.quaternion import QuaternionBase
class Log:
    def __init__(self):
        UdpIpPort = 'udp:0.0.0.0:14445' 
        self.conn = mavutil.mavlink_connection(UdpIpPort)
        self.conn.wait_heartbeat()
        self.boot_time = time.time()
        self.time_step = 1
        self.isFirst = True
        self.file_name_base = "ROV_logs_.txt"
        self.now = datetime.datetime.now()
        self.timestamp = datetime.datetime.strptime(str(self.now), "%Y-%m-%d %H:%M:%S.%f")
        self.formatted_timestamp = self.timestamp.strftime("%Y-%m-%d_%H-%M-%S")
        print(self.formatted_timestamp)
        self.file_name = "ROVlogs_"+self.formatted_timestamp+".txt"
        self.target_file = open(self.file_name, "a")
        self.data_log = {
        "Date":0,
        "Time":0,
        "roll(degree)":0,
        "pitch(degree)":0,
        "yaw(degree)":0,
        "rollspeed(w)":0, 
        "pitchspeed(w)":0, 
        "yawspeed(w)": 0, 
        "airspeed(m/s)": 0, 
        "groundspeed(m/s)": 0, 
        "heading(degree)": 0, 
        "altitude(m)":0, 
        "climbspeed(m/s)":0
        }
        self.createHeader()
    def createHeader(self):
        keys = list(self.data_log.keys())
        header = ','.join(keys) + '\n'
        self.target_file.write(header)

    def getData(self):
        imu = self.GetIMU()
        compass = self.GetVFRHUD()
        date = self.getDate()
        time_now = self.getTime()
        self.data_log["Date"]=date
        self.data_log["Time"]=time_now
        self.data_log["roll(degree)"] = imu['roll']*180/3.14
        self.data_log["pitch(degree)"]= imu['pitch']*180/3.14
        self.data_log["yaw(degree)"] = imu['yaw']*180/3.14
        self.data_log["rollspeed(w)"] = imu['rollspeed']*180/3.14
        self.data_log["pitchspeed(w)"] = imu['pitchspeed']*180/3.14
        self.data_log["yawspeed(w)"] = imu['yawspeed']*180/3.14
        self.data_log["airspeed(m/s)"] = compass['airspeed']
        self.data_log["groundspeed(m/s)"] = compass['groundspeed']
        self.data_log["heading(degree)"] = compass['heading']
        self.data_log["altitude(m)"] = compass['alt']
        self.data_log["climbspeed(m/s)"] = compass['climb']

    
    def GetIMU(self):
        sensorData = self.conn.recv_match(type="ATTITUDE", blocking=True)
        return sensorData.to_dict()

    def GetVFRHUD(self):
        bs = self.conn.recv_match(type="VFR_HUD", blocking=True)
        return bs.to_dict()
    def getDate(self):
        return datetime.datetime.now().date()
    def getTime(self):
        return datetime.datetime.now().time()

if __name__ == "__main__":
    last_log=time.time()
    logging = Log()
    # main loop
    while True:
        # change with data available from heartbeat
        now = time.time()
        if (now-last_log < logging.time_step):
            continue
        logging.getData()
        data_values = list(logging.data_log.values())
        # change to string
        conv_data_val = [str(data_val) for data_val in data_values]
        datas = ','.join(conv_data_val) + '\n'
        logging.target_file.write(datas)
        last_log=time.time()
        print(conv_data_val)