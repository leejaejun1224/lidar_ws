import csv
import rospy
from local_pkg.msg import Local
from geometry_msgs.msg import PointStamped

csv_file = 'moving_3.csv'

#csv_header = ['time','heading','gps_heading','speeed','gspeed','gear','headAcc']
#csv_header = ['x','y']
#csv_header = ['time','heading','imu_heading','gps_heading','gspeed','gear']
#csv_header = ['time','heading_ekf','gps_heading','imu_heading','headAcc']
csv_header = ['time','heading_from_position','gps_heading','imu_heading','heading_ekf','headAcc']
#def ekf_callback(data) : 
    

def callback(data):
    time = rospy.get_time()
    # x = data.x
    # y = data.y
    heading_ekf = data.heading
    imu_heading = data.imu_heading
    gps_heading = data.gps_heading
    # speeed = data.speeed
    # gspeed = data.gspeed
    #gear = data.gear
    headacc = data.headAcc
    # encoder_right = data.encoder_right
    #heading_ekf  = data.point.x
    #gps_heading = data.point.y
    #headAcc = data.header.seq
    heading_from_position = data.heading_from_position

    with open(csv_file,'a') as file:
        writer = csv.writer(file)
        #writer.writerow([time,heading,gps_heading,speeed,gspeed,gear,headacc])
        #writer.writerow([x,y])
        #writer.writerow([time,heading,imu_heading,gps_heading,gspeed,gear])
        #writer.writerow([x,y,time,encoder_right])
        #writer.writerow([time,heading_ekf,gps_heading,imu_heading,headacc])
        writer.writerow([time,heading_from_position,gps_heading,imu_heading,heading_ekf,headacc])
def spring():
    rospy.init_node('data_logger', anonymous=True)

    with open(csv_file, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)
    rospy.Subscriber('/local_msgs', Local, callback)
    #rospy.Subscriber('/heading_ekf', PointStamped, callback)


    rospy.spin()

if __name__ == '__main__':
    spring()