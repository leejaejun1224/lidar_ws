#!/usr/bin/env python3
import serial
import rospy
import time 
import numpy as np
from sensor_msgs.msg import Imu
from sig_int_handler import Activate_Signal_Interrupt_Handler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
import math
from numpy.linalg import inv
from math import cos, sin, asin, atan2, pi
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
from filterpy.kalman import KalmanFilter





class IMU_KF(): 
    def __init__(self): 
        rospy.init_node("imu_kf", anonymous=False)
        rospy.Subscriber("/imu", Imu, self.imu_call_back) 
        self.pub = rospy.Publisher("/imu_kf", PoseStamped, queue_size=1)        
        self.msg = PoseStamped()
        # oiler 
        self.roll = 0.0 
        self.pitch = 0.0 
        self.yaw = 0.0  # yaw
   
        # Gyroscope     
        self.p = 0.0 
        self.q = 0.0 
        self.r = 0.0 

        # Linear Accelerometer
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.firstRun = True

        # Kalmanfilter           
        self.H = np.eye(4) # observation matrix
        self.A = np.eye(4)        
        self.x = np.array([1, 0, 0, 0]).transpose()
        self.P = np.eye(4)        
        self.kf = KalmanFilter(dim_x = 4, dim_z=4)        
        self.kf.P = self.P # initial covariance matrix
        self.kf.H = self.H # observation matrix
         
    
    def quaternion_from_euler(self, roll,pitch, yaw):
        r = Rotation.from_euler('xyz',[roll,pitch,yaw],degrees=False)
        return r.as_quat()

    def imu_call_back(self, data):
        self.time = time.time() 
        self.p = data.angular_velocity.x
        self.q = data.angular_velocity.y
        self.r = data.angular_velocity.z

        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z  
    
    def angular_velocity_to_quaternion(self, angular_velocity,dt):
        
        norm = np.linalg.norm(angular_velocity)
        if norm == 0 :
            return np.array([1,0,0,0])

        angle = norm *dt
        axis = angular_velocity / norm
        

        qw = np.cos(angle/2)
        qx = axis[0]*np.sin(angle/2)
        qy = axis[1]* np.sin(angle/2)
        qz = axis[2]* np.sin(angle/2)

        return np.array([qx,qy,qz,qw])

    def angular_velocity_to_euler(p, q, r, dt):
        # 각속도를 오일러 각도로 변환하는 함수

        roll = np.arctan2(q * np.sin(dt) + r * np.cos(dt), p * np.cos(dt) + q * r * np.sin(dt))
        pitch = np.arcsin(-p * np.sin(dt) + q * np.cos(dt) * np.sin(roll) + r * np.cos(dt) * np.cos(roll))
        yaw = np.arctan2(q * np.sin(roll) - r * np.cos(roll), np.cos(roll) * p + np.sin(roll) * q * np.sin(dt) + np.sin(roll) * r * np.cos(dt))

        return roll,pitch, np.degrees(yaw)

    def EulerAccel(self,ax, ay,az):   
        g = 9.8
        theta = asin(ax / g)
        phi = asin(-ay / (g * cos(theta)))
        return phi, theta
      
    def accelerometer_to_euler(self,ax, ay, az, dt):
        # 가속도를 기반으로 롤, 피치, 요 계산

        # 가속도를 적분하여 회전 변화를 추정
        roll_rate = np.arctan2(-ay, np.sqrt(ax**2 + az**2))
        pitch_rate = np.arctan2(ax, np.sqrt(ay**2 + az**2))
        yaw_rate = np.arctan2(np.sqrt(ax**2 + ay**2), az)

        roll = roll_rate * dt
        pitch = pitch_rate * dt
        yaw = yaw_rate * dt

        return roll, pitch, yaw

      
         
    def main(self):                    
                   
        # state marix
        dt = 0.001      
        angular_velocity = [self.p, self.q, self.r]
        #angular_velocity_to_euler(self.p, self.q, self.r, dt)
        A = self.angular_velocity_to_quaternion(angular_velocity,dt)
        
        #A = 0.5*np.array([[0,-self.p,-self.q,-self.r],[self.p,0,self.r,-self.q],[self.q,-self.r,0,self.p],[self.r,self.q,-self.p,0]])        
        #self.x = np.array([1, 0, 0, 0]).transpose()
        
        # measurement matrix 
        #phi, theta = self.EulerAccel(self.ax, self.ay, self.az)
        phi,theta,psi = self.accelerometer_to_euler(self.ax, self.ay, self.az,dt)
         
        z = self.quaternion_from_euler(phi, theta,psi) #State variable as Quaternion form            
        
        self.kf.x = A # @ self.x # initial state matrix           
        self.Q = 0.1* np.eye(4)
        self.R = 0.1 * np.eye(4)
        self.kf.Q = self.Q # process_noise_covariance_matrix
        self.kf.R = self.R # measurement_noise_covariance_matrix                
        
        # kalmanfilter        
        self.kf.predict()
        self.kf.update(z)
        #print(phi,theta,psi)
        rotation = Rotation.from_quat(self.kf.x)               
        roll, pitch, yaw, = rotation.as_euler('xyz',degrees = True)            

        
        self.roll += roll 
        self.pitch += pitch
        self.yaw  += yaw
        
        self.msg.header = Header()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = 'imu_kf_link'

        self.msg.pose.position.x = 0.0 #self.roll % 360
        self.msg.pose.position.y = 0.0 #self.pitch % 360
        self.msg.pose.position.z = self.yaw % 360

        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0
        self.msg.pose.orientation.z =0.0
        self.msg.pose.orientation.w = 0.0

        self.pub.publish(self.msg)
        rospy.loginfo(self.msg)





if __name__ == '__main__': 

    Activate_Signal_Interrupt_Handler()
    imu_kf = IMU_KF()  
    

    try:
        while not rospy.is_shutdown():
            imu_kf.main()
            rate = rospy.Rate(50)
            rate.sleep()
    except rospy.ROSInitException:
        pass




