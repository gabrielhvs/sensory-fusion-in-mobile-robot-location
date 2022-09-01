from cmath import pi
import rospy
from sensor_msgs.msg import Imu
from math import  atan2, sqrt, sin, cos
from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import time
import csv
import SensorTag_Conect

class SensorTag_Ros:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('SensorTag', anonymous=True)
        self.initPublishers()
        self.rate = rospy.Rate(100)
        
    def initPublishers(self):
        self.sensor = rospy.Publisher('Sensor_Tag', Imu, queue_size=10)
        self.sensor_data = Imu()
        self.rate = rospy.Rate(100)


def main():

    
    imu = SensorTag_Ros()
    
    sensor = SensorTag_Conect.Sensor_conect('A0:E6:F8:AF:6B:83')

    gyo, acc, mag = [],[],[]
    fileName="Sample7.csv"

    i = 0
    j = 0

    while j<9:
        a = input("Enter!")
        
        while i<100:
            # Get dos dados dos sensores 

            gyo, acc, mag = sensor.read()
            mag = list(mag)
            acc = list(acc)
            gyo = list(gyo)
            print(i)
            file = open(fileName, 'a')
            writer = csv.writer(file)
            writer.writerow([imu.rospy.get_rostime().secs,imu.rospy.get_rostime().nsecs,gyo[0],gyo[1],gyo[2],acc[0],acc[1],acc[2],mag[0]- (-484.203), mag[1]- 282.946,mag[2]])
            print([imu.rospy.get_rostime().secs,imu.rospy.get_rostime().nsecs,gyo[0],gyo[1],gyo[2],mag[0]- (-484.203), mag[1]- 282.946,mag[2]])
            i=i+1
            
            # Envio dos dados adquiridos para o ROS 
            imu.sensor_data.angular_velocity.x=gyo[0]
            imu.sensor_data.angular_velocity.y=gyo[1]
            imu.sensor_data.angular_velocity.z=gyo[2]
            imu.sensor_data.linear_acceleration.x = acc[0]
            imu.sensor_data.linear_acceleration.y = acc[1]
            imu.sensor_data.linear_acceleration.z = acc[2]
            imu.sensor_data.orientation.x = 180*atan2 (acc[0],sqrt(acc[1]*acc[1] + acc[2]*acc[2]))/pi
            imu.sensor_data.orientation.y = 180*atan2 (acc[1],sqrt(acc[0]*acc[0] + acc[2]*acc[2]))/pi
            imu.sensor_data.orientation.z = 180*atan2(mag[1], mag[0])/pi 
            imu.sensor.publish(imu.sensor_data)
        file.close()
        i = 0
        j = j+1


    print("here!")    
    sensor.end()



if __name__ == "__main__":

    try:
             main()
    except rospy.ROSInterruptException:
            pass
   