import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

csv_file = 'moving_3.csv'
data_frame = pd.read_csv(csv_file)

#x = data_frame['x']
#y = data_frame['y']
time = data_frame['time']
#heading = data_frame['heading']
gps_heading = data_frame['gps_heading']
imu_heading = data_frame['imu_heading']
#speed = data_frame['speeed']
#gspeed = data_frame['gspeed']
#gear = data_frame['gear']
headAcc = data_frame['headAcc']
#encoder_right = data_frame['encoder_right']
heading_ekf = data_frame['heading_ekf']
#gps_heading = data_frame['gps_heading']
#headAcc = data_frame['headAcc']
heading_from_position = data_frame['heading_from_position']

plt.figure()



#plt.plot(time,heading,label='heading')
#plt.plot(time,gps_heading,label='gps_heading')
#plt.plot(time,speed,label='speeed')
#plt.plot(time,gspeed,label='gspeed')
#plt.plot(x2,y,label = 'y')
#plt.plot(time,gear,label='gear')
#plt.plot(time,gear,label='headAcc')

#plt.xlabel('time')
#plt.ylabel('value')
#plt.legend()

#plt.figure(2)
plt.plot(time,heading_from_position,label='heading_from_position')
plt.plot(time,imu_heading,label = 'imu_heading')
plt.plot(time,gps_heading,label='gps_heading')
plt.plot(time,headAcc,label='headAcc')
plt.plot(time,heading_ekf,label='heading_ekf')
#plt.plot(time,x,label = 'x')
#plt.plot(time,y,label = 'y')
#plt.xlabel('Time')
#plt.ylabel('Position')
#plt.title('local msg')

plt.legend()

plt.show()

