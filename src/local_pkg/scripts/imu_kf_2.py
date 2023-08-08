#!/usr/bin/env python3
import serial
import rospy
import time 
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, tan, pi
from sensor_msgs.msg import Imu
from sig_int_handler import Activate_Signal_Interrupt_Handler
#from scipy import io
from ahrs import IMU
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
import math
from numpy.linalg import inv
from math import cos, sin, asin, atan2, pi
from std_msgs.msg import Header



class IMU_KF(): 
    def __init__(self): 
        rospy.init_node("imu_kf", anonymous=False)
        rospy.Subscriber("/imu", Imu, self.imu_call_back) 
        self.pub = rospy.Publisher("/imu_kf", PoseStamped, queue_size=1)
        
        self.msg = PoseStamped()
        # oiler 
        self.roll = 0.0 
        self.pitch = 0.0 
        self.heading = 0.0  # yaw
   
        # Gyroscope     
        self.p = 0.0 
        self.q = 0.0 
        self.r = 0.0 

        # Linear Accelerometer
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.firstRun = True

        self.H = np.eye(4)
        self.Q = 0.0001 * np.eye(4)
        self.R = 10 * np.eye(4)
        self.x = np.array([1, 0, 0, 0]).transpose()
        self.P = np.eye(4)


    def imu_call_back(self, data):
        self.time = time.time() 
        self.p = data.angular_velocity.x
        self.q = data.angular_velocity.y
        self.r = data.angular_velocity.z

        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z           

        
  

    def EulerToQuaternion(self, phi, theta, psi):
        sinPhi = sin(phi/2)
        cosPhi = cos(phi/2)
        sinTheta = sin(theta/2)
        cosTheta = cos(theta/2)
        sinPsi = sin(psi/2)
        cosPsi = cos(psi/2)
        z = np.array([cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi,
                    sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi,
                    cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi,
                    cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi])
        return z

    def EulerAccel(self,ax, ay,az):   
        g = 9.8
        theta = asin(ax / g)
        phi = asin(-ay / (g * cos(theta)))
        return phi, theta

    def EulerKalman(self, A, z):       
    
        Xp = A @ self.x # Xp : State Variable Prediction
        Pp = A @ self.P @ A.T + self.Q # Error Covariance Prediction

        K = (Pp @ self.H.T) @ inv(self.H@Pp@self.H.T + self.R) # K : Kalman Gain

        self.x = Xp + K@(z - self.H@Xp) # Update State Variable Estimation
        P = Pp - K@self.H@Pp # Update Error Covariance Estimation
        
        phi   = atan2(2 * (self.x[2] * self.x[3] + self.x[0] * self.x[1]), 1 - 2*(self.x[1]**2 + self.x[2]**2))              
        if math.fabs(2*(self.x[1]*self.x[3]-self.x[0]*self.x[2])) >=1 :
            theta = math.copysign(math.pi/2,2*(self.x[1]*self.x[3]-self.x[0]*self.x[2]))  
        else :
            theta = -math.asin(2*(self.x[1]*self.x[3]-self.x[0]*self.x[2]))  
        psi   = atan2(2 *  (self.x[1] * self.x[2] + self.x[0] * self.x[3]), 1-2*(self.x[2]**2 + self.x[3]**2))
        return phi ,theta, psi
        
    def main(self):                          
        #roll  -> phi
        #pitch -> theta
        #yaw   -> psi     
        
        dt = 0.001           
        A = np.eye(4) + dt*0.5*np.array([[0,-self.p,-self.q,-self.r],[self.p,0,self.r,-self.q],[self.q,-self.r,0,self.p],[self.r,self.q,-self.p,0]])        
        phi, theta = self.EulerAccel(self.ax, self.ay, self.az)        
        z = self.EulerToQuaternion(phi, theta, 0) #State variable as Quaternion form        
        phi, theta, psi = self.EulerKalman(A, z)       
        
        self.roll = math.degrees(phi)%360
        self.pitch = math.degrees(theta)%360
        self.yaw = math.degrees(psi)%360

        self.msg.header = Header()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = 'imu_kf_link'

        self.msg.pose.position.x = self.roll
        self.msg.pose.position.y = self.pitch
        self.msg.pose.position.z = self.yaw      

        self.msg.pose.orientation.x = 88.0
        self.msg.pose.orientation.y = 88.0
        self.msg.pose.orientation.z = 88.0
        self.msg.pose.orientation.w = 88.0

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


