from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgGpsHdt
from local_pkg.msg import Local
from sensor_msgs.msg import Imu
import rospy
import pymap3d
import time

class EllipseD():
    def __init__(self):
        rospy.init_node('EllipseD', anonymous=False)
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.nav_call_back)
        rospy.Subscriber("/sbg/gps_hdt", SbgGpsHdt, self.gps_call_back)
        rospy.Subscriber("/imu", Imu, self.imu_call_back)
        
        self.pub = rospy.Publisher('/local_msgs', Local, queue_size=1)
        
        self.msg = Local()
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        
        
        self.time = 0.0


        #base
        self.lat = 37.3845170046
        self.lon = 126.655514367
        self.alt = 47.1644 
       
    def nav_call_back(self, data):
        self.x, self.y, _ = pymap3d.geodetic2enu(data.latitude, data.longitude, self.alt,
                                                 self.lat, self.lon, self.alt)

        
        self.time = 0.0


        #base
        
        self.time = 0.0


        #base


    def gps_call_back(self, data):
        self.heading = data.true_heading
    def imu_call_back(self, data):
        self.heading = data.true_heading
        
    def main(self):
        self.msg.x = self.x
        self.msg.y = self.y
        self.msg.heading = (self.heading + 14)%360
        self.pub.publish(self.msg)
        rospy.loginfo(self.msg)



if __name__ == '__main__':
    #Activate_Signal_Interrupt_Handler()
    gps = EllipseD()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        gps.main()
        rate.sleep()
