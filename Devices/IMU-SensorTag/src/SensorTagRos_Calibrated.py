from cmath import pi
import rospy
from sensor_msgs.msg import Imu
from math import  atan2, asin
from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import time
import csv
import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick


def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))

# Sensortag versions
AUTODETECT = "-"
SENSORTAG_V1 = "v1"
SENSORTAG_2650 = "CC2650"

class SensorBase:
    # Derived classes should set: svcUUID, ctrlUUID, dataUUID
    sensorOn  = struct.pack("B", 0x01)
    sensorOff = struct.pack("B", 0x00)
    period = struct.pack("B", 0x0A)

    def __init__(self, periph):
        self.periph = periph
        self.service = None
        self.ctrl = None
        self.data = None
        self.per = None

    def enable(self):
        if self.service is None:
            self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.per is None:
            self.per= self.service.getCharacteristics(self.perUUID) [0]
        if self.sensorOn is not None:
            self.ctrl.write(self.sensorOn,withResponse=True)
        if self.per is not None:
            self.per.write(self.period)

    def read(self):
        return self.data.read()

    def disable(self):
        if self.ctrl is not None:
            self.ctrl.write(self.sensorOff)

class MovementSensorMPU9250(SensorBase):
    svcUUID  = _TI_UUID(0xAA80)
    dataUUID = _TI_UUID(0xAA81)
    ctrlUUID = _TI_UUID(0xAA82)
    perUUID = _TI_UUID(0xAA83)
    sensorOn = None
    GYRO_XYZ =  7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
        self.ctrlBits = 0

    def enable(self, bits):
        SensorBase.enable(self)
        self.ctrlBits |= bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def disable(self, bits):
        self.ctrlBits &= ~bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def rawRead(self):
        dval = self.data.read()
        return struct.unpack("<hhhhhhhhh", dval)

class AccelerometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.bits = self.sensor.ACCEL_XYZ | self.sensor.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented?
        

    def enable(self):
        self.sensor.enable(self.bits)

    def disable(self):
        self.sensor.disable(self.bits)

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        rawVals = self.sensor.rawRead()[3:6]
        return tuple([ v*self.scale for v in rawVals ])

class MagnetometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 1#4912.0 / 32760
        # Reference: MPU-9250 register map v1.4

    def enable(self):
        self.sensor.enable(self.sensor.MAG_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.MAG_XYZ)

    def read(self):
        '''Returns (x_mag, y_mag, z_mag) in units of uT'''
        rawVals = self.sensor.rawRead()[6:9]
        return tuple([ v*self.scale for v in rawVals ])

class GyroscopeSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 500.0/65536.0

    def enable(self):
        self.sensor.enable(self.sensor.GYRO_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.GYRO_XYZ)

    def read(self):
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        rawVals = self.sensor.rawRead()[0:3]
        return tuple([ v*self.scale for v in rawVals ])

class SensorTag(Peripheral):
    def __init__(self,addr,version=AUTODETECT):
        Peripheral.__init__(self,addr)
        if version==AUTODETECT:
            svcs = self.discoverServices()
            if _TI_UUID(0xAA70) in svcs:
                version = SENSORTAG_2650


        fwVers = self.getCharacteristics(uuid=AssignedNumbers.firmwareRevisionString)
        if len(fwVers) >= 1:
            self.firmwareVersion = fwVers[0].read().decode("utf-8")
        else:
            self.firmwareVersion = u''

        if version==SENSORTAG_2650:
            self._mpu9250 = MovementSensorMPU9250(self)
            self.accelerometer = AccelerometerSensorMPU9250(self._mpu9250)
            self.magnetometer = MagnetometerSensorMPU9250(self._mpu9250)
            self.gyroscope = GyroscopeSensorMPU9250(self._mpu9250)
   
class SensorTag_Ros:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('SensorTag', anonymous=True)
        self.initPublishers()
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.sensor = rospy.Publisher('Sensor_Tag', Imu, queue_size=10)
        self.sensor_data = Imu()
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

def get_orientation(acc, gyo, mag, cal_offsets):

        acc[0] = ((cal_offsets[0][0]*acc[0] - cal_offsets[0][1]))
        acc[1] = ((cal_offsets[1][0]*acc[1] - cal_offsets[1][1]))
        acc[2] = ((cal_offsets[2][0]*acc[2] - cal_offsets[2][1]))

        gyo[0] = (gyo[0] - cal_offsets[3])
        gyo[1] = gyo[1] - cal_offsets[4]
        gyo[2] = gyo[2] - cal_offsets[5]

        mag[0] = mag[0] - cal_offsets[6]
        mag[1] = mag[1] - cal_offsets[7]
        mag[2] = mag[2] - cal_offsets[8]

        acc = np.array(acc)
        mag = np.array(mag)
        gyo = np.array(gyo)

        #madgwick = Madgwick() # Allocate for quaternions
        #Q = madgwick.updateMARG(Q, gyr=gyo, acc=acc, mag=mag)
        #[roll, pitch, yaw] = euler_from_quaternion(Q[1],Q[2],Q[3],Q[0])
        

        roll = 180*atan2 (acc[0],acc[2])/pi
        pitch = 180*atan2 (acc[1],acc[2])/pi
        yaw= 180*atan2(mag[1], mag[0])/pi
        #print(mag)
        return acc, gyo, [roll, pitch, yaw]

def main():


    host = 'A0:E6:F8:AF:6B:83' # MAC do dispositivo utilizado
    time_conetion = 1/50 #Periodo da conexão Bluetooh
    #O sensor pode chegar até 7ms de velocidade conexão,
    # porém o periodo minimo de aquisição de dados dos sensores é de 100ms 

    print('Connecting to ' + host)
    rospy.loginfo('Connecting to ' + host)
    
    imu = SensorTag_Ros()
    
    tag = SensorTag(host)

    # Ativando os sensores relativos a aplicação do Imu
    tag.accelerometer.enable()
    tag.gyroscope.enable()
    tag.magnetometer.enable()

    # Iniciando 
    time.sleep(0.5)# Esse tempo é especificado pelo o fabricante para iniciar os sensores

    
    cal_filename = '/home/robottraining03/Ararajuba/src/Extras/sensor_tag_pkg/src/params/mpu9250_cal_params.csv' # filename for saving calib coeffs
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
      
    #Q= np.array([0, 0, 0, 0])
    #Q = np.tile([1., 0., 0., 0.],1)
    while not rospy.is_shutdown():
        # Get dos dados dos sensores 
        acc = list(tag.accelerometer.read())
        gyo = list(tag.gyroscope.read())
        mag = list(tag.magnetometer.read())
        
        
        [acc, gyo, orientation]=get_orientation(acc, gyo, mag, cal_offsets)
        
        # Envio dos dados adquiridos para o ROS 
        imu.sensor_data.angular_velocity.x=gyo[0]
        imu.sensor_data.angular_velocity.y=gyo[1]
        imu.sensor_data.angular_velocity.z=gyo[2]
        imu.sensor_data.linear_acceleration.x = acc[0]
        imu.sensor_data.linear_acceleration.y = acc[1]
        imu.sensor_data.linear_acceleration.z = acc[2]

        imu.sensor_data.orientation.x = orientation[0]
        imu.sensor_data.orientation.y = orientation[1]
        imu.sensor_data.orientation.z = orientation[2]
        imu.sensor.publish(imu.sensor_data)

        # Espera pela a proxima amostra
        tag.waitForNotifications(time_conetion)
    #Desconexão da tag, deixando  acesso livre
    tag.disconnect()
    del tag


if __name__ == "__main__":

    try:
             main()
    except rospy.ROSInterruptException:
            pass
   