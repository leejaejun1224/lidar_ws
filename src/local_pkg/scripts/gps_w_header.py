#!/usr/bin/env python3
import rospy
import pymap3d
import time
import json
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from local_pkg.msg import Local
from local_pkg.msg import NavPVTWithHeader
from ublox_msgs.msg import NavPVT

    
def main():
    rospy.init_node("gps_w_header")
    pub_navpvt = rospy.Publisher('/gps_navpvt_w_header', NavPVTWithHeader, queue_size=10)


    def navpvt_callback(data):
        
        navpvt_with_header = NavPVTWithHeader()   
        
        # header
        navpvt_with_header.header.stamp = rospy.Time.now()
        navpvt_with_header.header.frame_id = "gps_navpvt_w_header"

        # navpvt_msg
        navpvt_with_header.iTOW = data.iTOW
        navpvt_with_header.year = data.year
        navpvt_with_header.month = data.month
        navpvt_with_header.day = data.day
        navpvt_with_header.hour = data.hour
        navpvt_with_header.min = data.min
        navpvt_with_header.sec = data.sec
        navpvt_with_header.valid = data.valid
        navpvt_with_header.tAcc = data.tAcc
        navpvt_with_header.nano = data.nano
        navpvt_with_header.fixType = data.fixType
        navpvt_with_header.flags = data.flags
        navpvt_with_header.flags2 = data.flags2
        navpvt_with_header.numSV = data.numSV
        navpvt_with_header.lon = data.lon
        navpvt_with_header.lat = data.lat
        navpvt_with_header.height = data.height
        navpvt_with_header.hMSL = data.hMSL
        navpvt_with_header.hAcc = data.hAcc
        navpvt_with_header.vAcc = data.vAcc
        navpvt_with_header.velN = data.velN
        navpvt_with_header.velE = data.velE
        navpvt_with_header.velD = data.velD
        navpvt_with_header.gSpeed = data.gSpeed
        navpvt_with_header.heading = data.heading
        navpvt_with_header.sAcc = data.sAcc
        navpvt_with_header.headAcc = data.headAcc
        navpvt_with_header.pDOP = data.pDOP
        navpvt_with_header.reserved1 = data.reserved1
        navpvt_with_header.headVeh = data.headVeh
        navpvt_with_header.magDec = data.magDec
        navpvt_with_header.magAcc = data.magAcc


        pub_navpvt.publish(navpvt_with_header)

    rospy.Subscriber("ublox_gps/navpvt", NavPVT, navpvt_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
   
