# libarary
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import csv
import rospy
import time
from collections import deque 
# class
from ahrs import IMU
# from gps import GPS
# msgs
from sensor_msgs.msg import Imu 
from local_pkg.msg import Local
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from local_pkg.msg import NavPVTWithHeader
from sig_int_handler import Activate_Signal_Interrupt_Handler
import sys
import math

class imu_offset() :
    def __init__ (self):
        rospy.init_node("EKF_GPS_IMU", anonymous=False)      
        self.pub = rospy.Publisher("/heading_ekf", PointStamped, queue_size=1)  
        self.pub2 = rospy.Publisher("/imu_raw", Imu, queue_size=5)  
        # rospy.Subscriber("/synced_imu", Imu, self.imu_call_back)  
        rospy.Subscriber("/imu", Imu, self.imu_call_back)  
        rospy.Subscriber("/synced_gps_navpvt", NavPVTWithHeader, self.gps_call_back)  
        rospy.Subscriber("/local_msgs", Local, self.local_call_back)  
        
        # class
        self.imu = IMU()
        # self.gps = GPS("Songdo")
        self.imu_raw = Imu()
        # ekf parameter
        dt = 0.1
        self.heading_from_position = 0.0

        # EKF class's parameter
        self.r = 0.0
        self.yaw = 0.0
        self.gps_heading = 0.0
        self.hAcc = 0.0
        self.heading_ekf = 0.0
        self.headAcc = 180.0
        self.gear = 0
        self.yaw_x = 0.0
        self.msg = PointStamped() 
        self.init = True

        self.count = 0
        self.offset = 0.0 # None
        self.imu_offset = 0.0

        # moving filter
        self.deque_size = 130
        self.heading_data = deque(maxlen=self.deque_size)

        # imu_parameter
        self.heading = 0.0
        self.parameter = 0.0

    def imu_call_back(self,data):
        self.imu_raw = Imu()
        self.imu_raw.header.stamp = rospy.Time.now()
        self.imu_raw.header.stamp = "imu_link"
        roll = data.orientation.x
        pitch = data.orientation.y
        yaw = data.orientation.z
        euler = [roll, pitch, yaw]
        ori_x, ori_y, ori_z, ori_w = self.euler_to_quaterion(euler)
        self.imu_raw = data
        self.imu_raw.header.stamp = data.header.stamp
        self.imu_raw.orientation.x = ori_x
        self.imu_raw.orientation.y = ori_y
        self.imu_raw.orientation.z = ori_z
        self.imu_raw.orientation.w = ori_w
        print(self.imu_raw)
        self.r = data.angular_velocity.z
        self.yaw = -1*(data.orientation.z) % 360

    # def gps_call_back(self,data):
    #     self.gps_heading = data.gps_heading
    #     # print("headACC : ", data.headAcc)
    #     self.headAcc = data.headAcc
    #     self.yaw = (data.imu_heading) %360
    #     self.gear = data.gear
    def euler_to_quaterion(self, euler):
        roll_half = euler[0] / 2
        pitch_half = euler[1] / 2
        yaw_half = euler[2] / 2

        cos_roll_half = math.cos(roll_half)
        sin_roll_half = math.sin(roll_half)
        cos_pitch_half = math.cos(pitch_half)
        sin_pitch_half = math.sin(pitch_half)
        cos_yaw_half = math.cos(yaw_half)
        sin_yaw_half = math.sin(yaw_half)

        orientation_w = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half
        orientation_x = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half
        orientation_y = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half
        orientation_z = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half
        return orientation_x, orientation_y, orientation_z, orientation_w

    def gps_call_back(self,data):
        self.headAcc = data.headAcc    
        self.hAcc = data.hAcc   
        
    def local_call_back(self,data):
        self.gps_heading = data.gps_heading  
        self.heading_from_position = data.heading_from_position
        self.gear = data.gear     

    def moving_average_filter(self,heading):              

        self.heading_data.append(heading)

        if len(self.heading_data) == self.deque_size:
            filtered_data = np.mean(self.heading_data)           
        
        else :
            filtered_data = heading
        
        return self.heading_data, filtered_data


    def main(self):
        args = sys.argv[1] # 스크립트 이름을 제외한 인수를 가져옴
        
        if self.init and self.gear == 0 and self.yaw != 0:              
            self.init = False      
            self.imu_offset = (360-int(args)+90)%360      
            self.parameter = self.imu_offset - self.yaw                              
        self.yaw_x = (self.yaw + self.parameter)%360  

        #self.heading = self.heading_ekf
        _, self.heading = self.moving_average_filter(self.yaw_x)
        if self.gear == 2:
            self.heading = (self.heading-180) %360
        # rosmsg
        self.msg.header = Header()                
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "heading_EKF"

        
        self.msg.point.x = self.heading % 360 #(self.heading_ekf) % 360
        self.msg.point.y = self.yaw_x
        self.msg.point.z = self.parameter # self.yaw

        self.pub.publish(self.msg)
        self.pub2.publish(self.imu_raw)
        # rospy.loginfo(self.msg)   



if __name__ == '__main__': 
    Activate_Signal_Interrupt_Handler()  
    imu_os = imu_offset()
    #rate = rospy.Rate(1000)
    while not rospy.is_shutdown():        
        imu_os.main()
        #rate.sleep()
        #rospy.spin()