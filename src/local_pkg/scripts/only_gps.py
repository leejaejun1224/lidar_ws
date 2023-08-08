import rospy
import pymap3d
import time
import json
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT


class GPS():
    def __init__(self):
        rospy.Subscriber("ublox_gps/fix", NavSatFix, self.gps_call_back)
        rospy.Subscriber("ublox_gps/navpvt", NavPVT, self.navpvt_call_back)
        rospy.Subscriber("/simul_gps", Pose, self.gps_call_back_simul)
        

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.hAcc = 0
        self.heading_switch = False

        self.cnt = False
        self.prev_heading = 0

        self.time = 0.0
       
        self.lat = 37.383784
        self.lon = 126.654310, 
        self.alt = 15.4 

    def gps_call_back(self, data):
        self.x, self.y, _ = pymap3d.geodetic2enu(data.latitude, data.longitude, self.alt,
                                                 self.lat, self.lon, self.alt)

    def gps_call_back_simul(self, data):
        self.x, self.y, _ = pymap3d.geodetic2enu(data.position.x, data.position.y, self.alt,
                                                 self.lat, self.lon, self.alt)
        self.time = time.time()

    def navpvt_call_back(self, data):
        self.time = time.time()
        self.hAcc = data.hAcc
        headAcc = data.headAcc

        gps_heading = (450-(data.heading * 10**(-5))) % 360

        if headAcc < 400000:
            self.heading_switch = True
            self.heading = gps_heading
        else:
            self.heading_switch = False


if __name__ == '__main__':
    try:
        gps = GPS()
        
    except rospy.ROSInterruptException:
        pass
