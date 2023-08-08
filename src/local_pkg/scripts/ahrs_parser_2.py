#!/usr/bin/env python3
import serial
import rospy
from sensor_msgs.msg import Imu,MagneticField
from std_msgs.msg import Header
from sig_int_handler import Activate_Signal_Interrupt_Handler
from local_functions import euler_from_quaternion as efq 
import numpy
import time

class AHRS_Parsing:
    def __init__(self):
        rospy.init_node("ahrs_raw", anonymous=False)
        rospy.loginfo('AHRS_Parsing : Serial connecting to /dev/imu')
        self.ser = serial.Serial(port = '/dev/imu', baudrate = 115200)        

        self.raw_data = Imu()
        self.msg = MagneticField()
        self.raw_data.header.stamp = rospy.Time.now()
        self.raw_data.header.frame_id = "imu_link"
        self.pub = rospy.Publisher("/imu", Imu, queue_size=1)
        #self.pub2 = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
        
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.distance_z = 0.0

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0

    def parser(self):
        ser_read = self.ser.readline()
        try:
            self.data = ser_read.decode('ascii')
        except:
            UnicodeDecodeError

        try:
            print(self.data)
            sdata = self.data.split(",") 
            print(sdata)           
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "ahrs_parser_link"

            self.raw_data.header = header
            ### when AHRS transmits euler values
            self.raw_data.orientation.x = float(sdata[1])
            self.raw_data.orientation.y = float(sdata[2])
            self.raw_data.orientation.z = float(sdata[3]) # euler angle <ok>
            self.raw_data.orientation.w = 0.0 # 
            self.raw_data.angular_velocity.x = float(sdata[4])
            self.raw_data.angular_velocity.y = float(sdata[5])
            self.raw_data.angular_velocity.z = float(sdata[6]) # Gyroscope
            self.raw_data.linear_acceleration.x = float(sdata[7]) * 9.81 # <g -> m/s**2>
            self.raw_data.linear_acceleration.y = float(sdata[8]) * 9.81
            self.raw_data.linear_acceleration.z = float(sdata[9]) * 9.81 # Linear Accelerometer
            
            ### when AHRS transmits quaternion values
            # self.raw_data.orientation.x = float(sdata[3])
            # self.raw_data.orientation.y = float(sdata[2])
            # self.raw_data.orientation.z = float(sdata[1]) # euler angle
            # self.raw_data.orientation.w = float(sdata[4]) # quaternion <ok> zyxw
            # self.raw_data.angular_velocity.x = float(sdata[5])
            # self.raw_data.angular_velocity.y = float(sdata[6])
            # self.raw_data.angular_velocity.z = float(sdata[7]) # Gyroscope
            # self.raw_data.linear_acceleration.x = float(sdata[8]) * 9.81 #<g -> m/s**2>
            # self.raw_data.linear_acceleration.y = float(sdata[9]) * 9.81
            # self.raw_data.linear_acceleration.z = float(sdata[10]) * 9.81 # Linear Accelerometer
        
            #self.msg.magnetic_field.x = float(sdata[11])
            #self.msg.magnetic_field.y = float(sdata[12])
            #self.msg.magnetic_field.z = float(sdata[13])
            #self.distance_x = float(sdata[10])
            #self.distance_y = float(sdata[11])
            #self.distance_z = float(sdata[12])
            
            
            
        except IndexError:  
            #rospy.loginfo(sdata)
            rospy.loginfo("=============================")


        self.pub.publish(self.raw_data)
        #self.pub2.publish(self.msg)

if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    imu = AHRS_Parsing()
    try:
        while not rospy.is_shutdown():
            imu.parser()
    except rospy.ROSInitException:
        pass
