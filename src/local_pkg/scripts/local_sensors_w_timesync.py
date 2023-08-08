#!/usr/bin/env python3
import rospy
import pymap3d
import time
import json
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose,PointStamped
from sensor_msgs.msg import NavSatFix , Imu
from ublox_msgs.msg import NavPVT
from local_pkg.msg import NavPVTWithHeader
import message_filters


# AHRS_PARSER, ENCODER_PARSER, ODOMETRY 는 그대로 두고, 이 코드는 센서들의 타임 싱크를 맞추기 위한 코드이다. 

class Sensors():
    def __init__(self):
        # GPS
        # AHRS
        # ENCODER
        self.gps_fix_pub = rospy.Publisher('/synced_gps_fix',NavSatFix,queue_size=10)
        self.gps_navpvt_pub = rospy.Publisher('/synced_gps_navpvt',NavPVTWithHeader,queue_size=10)
        self.imu_pub = rospy.Publisher('/synced_imu',Imu, queue_size=10)
        #self.encoder_pub = rospy.Publisher('/synced_encoder',PointStamped,queue_size=10)
        
    
    def callback(self,gps_fix_data,gps_navpvt_data,imu_data):        
        
        self.gps_fix_pub.publish(gps_fix_data)
        self.gps_navpvt_pub.publish(gps_navpvt_data)
        self.imu_pub.publish(imu_data)
        #self.encoder_pub.publish(encoder_data)
                

    def listener(self):
        rospy.init_node('message_synchronizer_local',anonymous=False)        
        gps_fix_sub = message_filters.Subscriber('/ublox_gps/fix',NavSatFix) # header 있음
        gps_navpvt_sub = message_filters.Subscriber('/gps_navpvt_w_header',NavPVTWithHeader) # header 넣어서 새로 만든 msg
        imu_sub = message_filters.Subscriber('/imu',Imu) # header 있음
        #encoder_sub = message_filters.Subscriber('/Displacement_right',PointStamped) # header 있음

        ts = message_filters.ApproximateTimeSynchronizer([gps_fix_sub,gps_navpvt_sub,imu_sub],queue_size=10,slop=0.2,allow_headerless=False)
        ts.registerCallback(self.callback)

        rospy.spin()        


if __name__ == '__main__':
    try:
        sensors = Sensors()
        
        
        sensors.listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
