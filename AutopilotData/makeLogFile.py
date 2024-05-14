# Open or create a log file in write mode
import time
import datetime
from pymavlink import mavutil


class Log:
    def __init__(self):
        UdpIpPort = "udp:0.0.0.0:14445"
        self.conn = mavutil.mavlink_connection(UdpIpPort)
        self.conn.wait_heartbeat()
        self.boot_time = time.time()
        self.isFirst = True
        self.file_name_base = "./log/ROV_logs_"
        self.now = datetime.datetime.now()
        self.timestamp = datetime.datetime.strptime(
            str(self.now), "%Y-%m-%d %H:%M:%S.%f"
        )
        self.formatted_timestamp = self.timestamp.strftime("%Y-%m-%d_%H-%M-%S")
        print(self.formatted_timestamp)
        self.file_name = self.file_name_base + self.formatted_timestamp + ".txt"
        self.target_file = open(self.file_name, "a")
        self.data_log = {
            "Date": 0,
            "Time": 0,
            "roll(degree)": 0,
            "pitch(degree)": 0,
            "yaw(degree)": 0,
            "rollspeed(w)": 0,
            "pitchspeed(w)": 0,
            "yawspeed(w)": 0,
            "airspeed(m/s)": 0,
            "groundspeed(m/s)": 0,
            "heading(degree)": 0,
            "altitude(m)": 0,
            "climbspeed(m/s)": 0,
            "xacc": 0,
            "yacc": 0,
            "zacc": 0
        }
        self.createHeader()

    def createHeader(self):
        keys = list(self.data_log.keys())
        header = ",".join(keys) + "\n"
        self.target_file.write(header)
        self.target_file.flush()

    def getData(self):
        imu = self.GetIMU()
        compass = self.GetVFRHUD()
        accell = self.GetAccell()
        date = self.getDate()
        time_now = self.getTime()
        self.data_log["Date"] = date
        self.data_log["Time"] = time_now
        self.data_log["roll(degree)"] = imu["roll"] * 180 / 3.14
        self.data_log["pitch(degree)"] = imu["pitch"] * 180 / 3.14
        self.data_log["yaw(degree)"] = imu["yaw"] * 180 / 3.14
        self.data_log["rollspeed(w)"] = imu["rollspeed"] * 180 / 3.14
        self.data_log["pitchspeed(w)"] = imu["pitchspeed"] * 180 / 3.14
        self.data_log["yawspeed(w)"] = imu["yawspeed"] * 180 / 3.14
        self.data_log["airspeed(m/s)"] = compass["airspeed"]
        self.data_log["groundspeed(m/s)"] = compass["groundspeed"]
        self.data_log["heading(degree)"] = compass["heading"]
        self.data_log["altitude(m)"] = compass["alt"]
        self.data_log["climbspeed(m/s)"] = compass["climb"]
        self.data_log["xacc"] = accell["xacc"] / 100
        self.data_log["yacc"] = accell["yacc"] / 100
        self.data_log["zacc"] = accell["zacc"] / 100

    def GetIMU(self):
        imuData = self.conn.recv_match(type="ATTITUDE", blocking=True)
        return imuData.to_dict()

    def GetVFRHUD(self):
        vfrData = self.conn.recv_match(type="VFR_HUD", blocking=True)
        return vfrData.to_dict()

    def GetAccell(self):
        accellData = self.conn.recv_match(type="SCALED_IMU2", blocking=True)
        return accellData.to_dict()

    def getDate(self):
        return datetime.datetime.now().date()

    def getTime(self):
        return datetime.datetime.now().time()

if __name__ == "__main__":
    logging = Log()

    while True:

        logging.getData()
        data_values = list(logging.data_log.values())
        
        conv_data_val = [str(data_val) for data_val in data_values]
        datas = ",".join(conv_data_val) + "\n"
        logging.target_file.write(datas)
        logging.target_file.flush()
        
        print(conv_data_val)
