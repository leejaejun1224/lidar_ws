#!/usr/bin/env python3
import rospy
import pymap3d
import time
import json
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from local_pkg.msg import NavPVTWithHeader
from ublox_msgs.msg import NavPVT


class GPS():
    def __init__(self, base_name):
        rospy.Subscriber("synced_gps_fix", NavSatFix, self.gps_call_back)
        rospy.Subscriber("/synced_gps_navpvt", NavPVT, self.navpvt_call_back)
        rospy.Subscriber("/simul_gps", Pose, self.gps_call_back_simul)

        self.base_name = base_name        
        
        # NavSatFix
        self.x = 0.0
        self.y = 0.0

        # NavPVT        
        self.heading = 0.0
        self.gps_heading = 0.0
        self.gps_heading_raw_data = 0.0
        self.gspeed = 0
        self.hAcc = 0
        self.headAcc = 0
        
        self.heading_switch = False

        self.cnt = False
        self.time = 0.0

        with open('/home/gigacha/TEAM-GIGACHA/src/local_pkg/scripts/base.json') as base:
            base_data = json.load(base)

        self.base = base_data[self.base_name]  # KCity, Siheung, KCity_semi, Songdo 
        self.lat = self.base['lat']
        self.lon = self.base['lon']
        self.alt = self.base['alt']

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
        self.gspeed = data.gSpeed
        self.headAcc = data.headAcc / 100000 
        
        gps_heading_value = (450-(data.heading * 10**(-5))) % 360
        self.gps_heading = gps_heading_value
        if self.headAcc < 7:
            self.heading_switch = True
            self.heading = gps_heading_value
            
        else:
            self.heading_switch = False


if __name__ == '__main__':
    try:
        gps = GPS()
    except rospy.ROSInterruptException:
        pass
