#!/usr/bin/env python3
import serial
import rospy
import signal
from std_msgs.msg import Int64
from geometry_msgs.msg import PointStamped

from sig_int_handler import Activate_Signal_Interrupt_Handler

class Encoder_Parsing():
    def __init__(self):
        rospy.init_node('Displacement_right', anonymous = False)
        self.pub = rospy.Publisher('/Displacement_right', PointStamped, queue_size = 1)
        self.ser = serial.Serial(port = '/dev/encoder', baudrate = 115200) 
        self.init_data = 0

    def main(self):
        res = self.ser.readline()
        while True:
            try:
                self.init_data = int(res)
                rospy.loginfo(int(res))
                print("encoder : ",res.decode('utf-8'))
                break
            except:
                res = self.ser.readline()

        # msg # header & point
        encoder_right = PointStamped()        
        encoder_right.header.stamp = rospy.Time.now()
        encoder_right.header.frame_id = "encoder_w_header"
        encoder_right.point.x = int(res) 

        self.pub.publish(encoder_right)

if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    enc = Encoder_Parsing()

    while not rospy.is_shutdown():
        enc.main()