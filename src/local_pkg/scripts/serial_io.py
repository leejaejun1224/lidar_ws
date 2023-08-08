#!/usr/bin/env python3
from sig_int_handler import Activate_Signal_Interrupt_Handler
import serial
from local_pkg.msg import Serial_Info, Control_Info
import threading
import struct
import rospy
from std_msgs.msg import Int16

class Serial_IO:
    def __init__(self):
        self.serial_lock=threading.Lock()
        
        # Steer manual compensation
        self.steer_offset = 1.35

        # Serial Connect
        # self.ser = serial.Serial("/dev/ttyUSB0", 115200) # Simulation
        self.ser = serial.Serial("/dev/erp42", 115200) # Real World
        print("Serial_IO: Serial connecting to /dev/erp42...")

        # ROS Publish
        rospy.init_node("Serial_IO", anonymous=False)
        self.serial_pub = rospy.Publisher("/serial", Serial_Info, queue_size=1)
        print("Serial_IO: Initializing ROS node...")
        #print("test")


        # Messages/Data
        self.serial_msg = Serial_Info()  # Message o publish
        self.alive = 0

        #Subscribe Control Info from Controller
        rospy.Subscriber("/controller", Control_Info, self.controlCallback)
        self.control_input = Control_Info()

        # Serial Read Thread
        th_serialRead = threading.Thread(target=self.serialRead)
        th_serialRead.daemon = True
        th_serialRead.start()

        # Main Loop
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.serialWrite()
                print(self.control_input, "\n--------------")
                rate.sleep()
            except struct.error:
                print("struct error occurred!")


    def serialRead(self):
        print("Serial_IO: Serial reading thread successfully started")

        while True:
            
            # print(f"Serial_IO: Reading serial {self.alive}")

            packet = self.ser.read_until(b'\x0d\x0a')
            # print(len(packet))
            if len(packet) == 18:
                header = packet[0:3].decode()

                if header == "STX":
                    #auto_manual, e-stop, gear
                    (self.serial_msg.auto_manual,
                    self.serial_msg.emergency_stop,
                    self.serial_msg.gear) \
                    = struct.unpack("BBB", packet[3:6])
                    
                    #speed, steer
                    tmp1, tmp2 = struct.unpack("2h", packet[6:10])
                    self.serial_msg.speed = tmp1 / 10  # km/h

                    self.serial_msg.steer = tmp2 / 71  # degree


                    #brake
                    self.serial_msg.brake = struct.unpack("B", packet[10:11])[0]

                    #encoder -- not working
                    self.serial_msg.encoder = struct.unpack("BBBB", packet[11:15])
                    # self.serial_msg.encoder = -1

                    #alive (heartbeat)
                    self.alive = struct.unpack("B", packet[15:16])[0]

                    self.serial_pub.publish(self.serial_msg)

    def serialWrite(self):
        self.serial_lock.acquire()
        #Min/Max Limit
        if self.control_input.speed > 20:
            self.control_input.speed = 20
        if self.control_input.speed < 0:
            self.control_input.speed = 0

        self.control_input.speed = max(0, min(self.control_input.speed, 20))
        
        if self.control_input.brake > 200:
            self.control_input.brake = 200
        if self.control_input.brake < 0:
            self.control_input.brake = 0

        if self.control_input.gear == 2:
            self.control_input.steer = -self.control_input.steer

        self.control_input.steer += self.steer_offset
        if self.control_input.steer < -27:
            self.control_input.steer = -27
        if self.control_input.steer > 27:
            self.control_input.steer = 27
        
    
        result = struct.pack( 
            ">BBBBBBHhBBBB",
            0x53,
            0x54,
            0x58,
            1, #always auto
            self.control_input.emergency_stop,
            int(self.control_input.gear),
            int(self.control_input.speed * 10),
            int(self.control_input.steer * 71),
            int(self.control_input.brake),
            self.alive,
            0x0D,
            0x0A
        )
        self.ser.write(result)

        self.serial_lock.release()


    def controlCallback(self, msg):
        self.control_input = msg

if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    erp = Serial_IO()
