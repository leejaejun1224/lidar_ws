#!/usr/bin/env python3
import time
import rospy
import math
import argparse
from local_pkg.msg import Local
from nav_msgs.msg import Path
from gps import GPS
from ahrs import IMU
from odometry import DR
from ahrs_parser import AHRS_Parsing
from local_functions import quaternion_from_euler
from sig_int_handler import Activate_Signal_Interrupt_Handler
from collections import deque
from ublox_msgs.msg import NavPVT
from std_msgs.msg import Header, Int16, Int64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from local_pkg.msg import NavPVTWithHeader
from collections import deque 

class Localization():
    def __init__(self, base):
        rospy.init_node('Localization', anonymous=False)
        rospy.Subscriber("/vis_global_path", Path, self.masterCallback)
        rospy.Subscriber("/synced_gps_navpvt", NavPVTWithHeader, self.navpvt_call_back)
        rospy.Subscriber("/heading_ekf",PointStamped,self.heading_ekf_call_back)
        #rospy.Subscriber("/heading_ekfs",PointStamped,self.heading_ekf_call_back)
        self.flag = False

        #self.pub3 = rospy.Publisher('/local_msg', Local, queue_size=1)
        self.pub = rospy.Publisher('/local_msgs', Local, queue_size=1)
        self.pub2 = rospy.Publisher('/local_msgs_to_vision',PoseStamped, queue_size=10)
        #self.pub3 = rospy.Publisher('/sample', Local, queue_size=1)
        self.msgs = PoseStamped()
        self.msg = Local()
        self.base = base
        self.gps = GPS(self.base)
        self.imu = IMU()
        self.dr = DR()

        #self.ahrs_parser = AHRS_Parsing()
        self.heading_list = deque(maxlen=5)

        #serial 
        self.gear = 0
        self.offset = 0

        self.heading = 0.0
        self.gps_heading = 0.0
        self.imu_heading = 0.0
        self.headAcc = 0

        self.master_switch = False
        self.dr_init = False
        self.last_pulse = 0

        self.dead_heading = 0.0
        self.encoder_left = 0
        self.encoder_right = 0

        # heading_ekf
        self.heading_ekf = 0.0

        # heading_from_position
        self.heading_from_position = 0.0
        self.previous_heading = 0.0

        self.previous_x = 0.0
        self.previous_y = 0.0

        self.dist = 0.0


    # original
    # def heading_ekf_call_back(self,msg):
    #     self.heading_ekf = msg.point.x
    
    def heading_ekf_call_back(self,msg):
        self.heading_ekf = msg.point.x
        
    def masterCallback(self, msg):
        self.master_switch = True

    def navpvt_call_back(self, data):
        self.flag = True
        # print("navpvt callbacked")
        self.time = time.time()
        self.hAcc = float(data.hAcc)
        # print(type(data.headAcc))
        headAcc = data.headAcc
        self.gspeed = data.gSpeed
        gps_heading_value = (450-(data.heading * 10**(-5))) % 360
    
        if headAcc < 400000:
            self.heading_switch = True
            self.heading = gps_heading_value
            self.gps_heading = gps_heading_value
        else:
            self.heading_switch = False

        # print("callback finished")

    def heading_decision(self):
        global time_sync
        main_time = time.time()
        time_sync = None
    
        if self.dr.gear == 0:  
    
            if (main_time - self.gps.time) < 0.2 and (main_time - self.imu.time) < 0.2:
                time_sync = "Sync"
                if self.gps.heading_switch == True:
                    self.offset = self.gps.heading - self.imu.heading
                    self.heading = (self.imu.heading + self.offset)%360
                else:
                    self.heading = (self.imu.heading + self.offset)%360
            else:
                time_sync = "Unsync"
                self.heading = (self.imu.heading + self.offset)%360

        else:
            self.heading = (self.imu.heading + self.offset)%360

            # if self.gps.heading_switch == True:
            #         gps_heading_back = (self.gps.heading - 180)% 360
            #         self.offset = gps_heading_back - self.imu.heading
            #         self.heading = (self.imu.heading + self.offset)%360
            # else:
            #     self.heading = (self.imu.heading + self.offset)%360
    
        
    def true_heading(self):
        if self.dr.gear == 0:
            recent_heading = deque(maxlen=3)

            recent_heading.append(self.heading)

            heading_offset = 130
            
            last = recent_heading[0]
            
            if abs(last - self.heading) > heading_offset :
                self.heading = last
                recent_heading.append(last)
            else :
                recent_heading.append(self.heading) 

    def calculate_heading_from_position(self):

        x = self.gps.x
        y = self.gps.y

        self.dist = math.sqrt((x-self.previous_x)**2+(y-self.previous_y)**2)

        if self.dist >= 0.15 :
            heading = math.degrees(math.atan2(y-self.previous_y,x-self.previous_x)) % 360
            self.heading_from_position = heading
            self.previous_x = x 
            self.previous_y = y
            self.previous_heading = heading
        
        else :
            self.heading_from_position = self.previous_heading


        

    def main(self):
        #if self.flag == True:
        #self.heading_decision()
        #self.true_heading()
        
        #header 
        self.msg.header = Header()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = 'local_link'
        
        #imu
        orientation = list(quaternion_from_euler(
            self.imu.roll, self.imu.pitch, self.heading))            
        self.msg.imu_heading = self.imu.heading

        # ahrs
        #self.msg.distance_x = self.ahrs_parser.distance_x
        #self.msg.distance_y = self.ahrs_parser.distance_y
        #self.msg.distance_z = self.ahrs_parser.distance_z
        

        # heading_from_position
        self.calculate_heading_from_position()
        self.msg.heading_from_position = round(self.heading_from_position,2)
        self.msg.dist = self.dist
        # gps
        self.msg.x = self.gps.x
        self.msg.y = self.gps.y
        self.msg.hAcc = self.gps.hAcc
        self.msg.gspeed = self.gps.gspeed * 0.0036 # self.gps.gspeed *0.001 #[mm/s]->[m/s] *0.0036 # [mm/s]->[km/h]
        self.msg.gps_heading = self.gps.gps_heading 
        self.msg.gps_heading_raw_data = self.gps.gps_heading_raw_data 

                
        
        if self.gps.headAcc <= 4.0 :
            self.msg.gps_heading_under_4_headAcc = self.gps.gps_heading 
            self.heading_list.append(self.gps.gps_heading)
        elif len(self.heading_list) <= 4 :
            self.msg.gps_heading_under_4_headAcc = 0.0 
        else :
            self.msg.gps_heading_under_4_headAcc = self.heading_list[0]

        self.msg.headAcc = int(self.gps.headAcc)

        #encoder & serial            
        self.msg.encoder_left = self.dr.left
        self.msg.encoder_right = self.dr.right
        self.msg.speeed = self.dr.speed    
        self.msg.dis = self.dr.pulse / 58.82
        self.msg.gear = self.dr.gear
        self.msg.dead_right = self.dr.dead_right 
        self.msg.distance = self.dr.distance
        self.msg.encoder_x = self.dr.encoder_x
        self.msg.encoder_y = self.dr.encoder_y

        self.msg.position_error_between_encoder_and_gps = round(math.sqrt((self.gps.x - self.dr.encoder_x)**2 + (self.gps.y - self.dr.encoder_y)**2),2)

        
        # local msgs to vision
        self.msgs.header = Header()
        self.msgs.header.stamp = rospy.Time.now()
        self.msgs.header.frame_id = 'base_link'

        self.msgs.pose.position.x = self.gps.x
        self.msgs.pose.position.y = self.gps.y
        self.msgs.pose.position.z = 15.0

        self.msgs.pose.orientation.x = self.gps.gspeed * 0.0036 
        self.msgs.pose.orientation.y = self.dr.speed 
        self.msgs.pose.orientation.z = 0.0
        self.msgs.pose.orientation.w = self.heading_ekf #self.heading
        # self.msgs.pose.orientation.w = 10 #self.headingL

        if self.master_switch:
            if 0 < self.gps.hAcc < 50:
                self.msg.dr_x = self.gps.x
                self.msg.dr_y = self.gps.y
                self.dr_init = False

            else:
                if not self.dr_init:
                    self.last_pulse = self.dr.pulse
                    self.dr_init = True

                if self.gps.heading_switch == True and self.dr_init:
                    if self.dr.gear == 0:
                        self.dead_heading = self.heading
                    elif self.dr.gear == 2:
                        self.dead_heading = (self.heading + 180) % 360

                    self.diff_pulse = self.dr.pulse - self.last_pulse
                    dis = self.diff_pulse / 58.82

                    self.msg.dr_x += dis * \
                        math.cos(math.radians(self.dead_heading))
                    self.msg.dr_y += dis * \
                        math.sin(math.radians(self.dead_heading))
                    self.last_pulse = self.dr.pulse

        #ahrs
        self.msg.roll = self.imu.roll
        self.msg.pitch = self.imu.pitch
        self.msg.heading = self.heading_ekf  #self.heading        
    
        self.msg.orientation.x = orientation[0]
        self.msg.orientation.y = orientation[1]
        self.msg.orientation.z = orientation[2]
        self.msg.orientation.w = orientation[3]\
            
        self.pub.publish(self.msg)
        self.pub2.publish(self.msgs)
        #self.pub3.publish(self.msg)

        rospy.loginfo(self.msg)
        # rospy.loginfo(self.msg.heading)



if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    #parser = argparse.ArgumentParser()
    #parser.add_argument('--base', '-b', nargs='*',
    #                    help='base_names', default=[], dest='base_names')
    #basename_input = parser.parse_args().base_names

    #if len(basename_input) == 0:
    #    base_name = "Songdo" #"KCity"       

    #elif len(basename_input) == 1:
    #    base_name = basename_input[0]

    #else:
    #    raise Exception('Invalid Arguments')

    loc = Localization("Songdo") #KCity
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        loc.main()
        rate.sleep()
    #if len(basename_input) == 0:
    #    base_name = "Songdo" #"KCity"