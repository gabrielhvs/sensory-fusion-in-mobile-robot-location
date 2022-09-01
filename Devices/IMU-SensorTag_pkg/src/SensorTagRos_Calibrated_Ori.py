#!/usr/bin/python3
from cmath import pi
import rospy
from sensor_msgs.msg import Imu, MagneticField
from math import  atan2, asin
import time
import csv
import numpy as np
import csv
import SensorTag_Conect
   
class SensorTag_Ros:
    
    def __init__(self):
        self.rospy=rospy
        #self.rospy.init_node('SensorTag', anonymous=True)
        self.rospy.init_node('SensorTag', anonymous=True)
        self.initPublishers()
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.sensorImu = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.sensorMag = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.sensor_data_Imu = Imu()
        self.sensor_data_Magnetic = MagneticField()
        self.rate = rospy.Rate(10)

def euler_from_quaternion(x, y, z, w):
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def get_orientation(orientationE, acc, gyo, mag):
        '''
        acc[0] = ((cal_offsets[0][0]*acc[0] - cal_offsets[0][1]))
        acc[1] = ((cal_offsets[1][0]*acc[1] - cal_offsets[1][1]))
        acc[2] = ((cal_offsets[2][0]*acc[2] - cal_offsets[2][1]))
        
        gyo[0] = (gyo[0] - cal_offsets[3])
        gyo[1] = (gyo[1] - cal_offsets[4])
        gyo[2] = (gyo[2] - cal_offsets[5])
        
        mag[0] = (mag[0] - cal_offsets[6])
        mag[1] = (mag[1] - cal_offsets[7])
        mag[2] = (mag[2] - cal_offsets[8])
        ''' 
        mag[0] = mag[0] + 484.203
        mag[1] = mag[1] -282.946
       

        acc = np.array(acc)
        mag = np.array(mag)
        gyo = np.array(gyo)
        
        deltapitch = gyo[0]* 0.1
        deltaroll = gyo[1] * 0.1
        deltayaw = gyo[2] * 0.1
        
        if(abs(deltapitch)>0.01 ):
            orientationE[1] = orientationE[1] + deltapitch

        if(abs(deltaroll)>0.01 ):
            orientationE[0] = orientationE[0] + deltaroll
            

        if(abs(deltayaw)>0.01 ):
            orientationE[2] = orientationE[2] + deltayaw

        roll = atan2 (acc[0],acc[2])
        pitch = atan2 (acc[1],acc[2])
        yaw= atan2(mag[1], mag[0])
        print(180*yaw/pi)
        orientationsQ = get_quaternion_from_euler(roll,pitch,yaw)
        return acc, gyo, mag, orientationsQ, orientationE

def main():

    imu = SensorTag_Ros()
    
    tag = SensorTag_Conect.Sensor_conect('A0:E6:F8:AF:6B:83')

    # Iniciando 
    time.sleep(0.5)# Esse tempo é especificado pelo o fabricante para iniciar os sensores
    '''
    cal_filename = 'src/Ararajuba/Devices/IMU-SensorTag_pkg/src/params/mpu9250_cal_params.csv' # filename for saving calib coeffs
    cal_offsets = np.array([[],[],[],0.0,0.0,0.0,[],[],[]]) # cal vector
    
    # Get dos coeficientes calculados na etapa de Calibração
    with open(cal_filename,'r',newline='') as csvfile:
        reader = csv.reader(csvfile,delimiter=',')
        iter_ii = 0
        for row in reader:
            if len(row)>2:
                row_vals = [float(ii) for ii in row[int((len(row)/2)+1):]]
                cal_offsets[iter_ii] = row_vals
            else:
                cal_offsets[iter_ii] = float(row[1])
            iter_ii+=1
     ''' 
    
    orientationE = np.array([0.,0.,0.])
    while not rospy.is_shutdown():
        # Get dos dados dos sensores 
        gyo, acc, mag = tag.read()
        [acc, gyo,mag, orientationQ, orientationE]=get_orientation(orientationE,list(acc), list(gyo), list(mag))
        print(gyo, acc, mag)
        now = rospy.get_rostime()
        # Envio dos dados adquiridos para o ROS 
        imu.sensor_data_Imu.angular_velocity.x=gyo[0]
        imu.sensor_data_Imu.angular_velocity.y=gyo[1]
        imu.sensor_data_Imu.angular_velocity.z=gyo[2]
        imu.sensor_data_Imu.orientation.x = orientationQ[0]
        imu.sensor_data_Imu.orientation.y = orientationQ[1]
        imu.sensor_data_Imu.orientation.z = orientationQ[2]
        imu.sensor_data_Imu.orientation.w = orientationQ[3]
        imu.sensor_data_Imu.linear_acceleration.x = acc[0]
        imu.sensor_data_Imu.linear_acceleration.y = acc[1]
        imu.sensor_data_Imu.linear_acceleration.z = acc[2]
        imu.sensor_data_Magnetic.magnetic_field.x = mag[0]
        imu.sensor_data_Magnetic.magnetic_field.y = mag[1]
        imu.sensor_data_Magnetic.magnetic_field.z = mag[2]
        imu.sensor_data_Magnetic.header.frame_id = 'mag'
        imu.sensor_data_Imu.header.frame_id = 'imu'
        imu.sensor_data_Imu.header.stamp.secs = now.secs
        imu.sensor_data_Imu.header.stamp.nsecs = now.nsecs
        imu.sensor_data_Magnetic.header.stamp.secs = now.secs
        imu.sensor_data_Magnetic.header.stamp.nsecs = now.nsecs
        
        imu.sensorImu.publish(imu.sensor_data_Imu)
        imu.sensorMag.publish(imu.sensor_data_Magnetic)
        
    #Desconexão da tag, deixando  acesso livre
    tag.disconnect()
    del tag

if __name__ == "__main__":

    try: 
             main()
    except rospy.ROSInterruptException:
            pass
   