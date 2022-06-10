import rospy
from sensor_msgs.msg import Imu
from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import time

# Referência: https://usermanual.wiki/Document/CC265020SensorTag20Users20Guide2020Texas20Instruments20Wiki.2070227354
    ## As referencias de UUID's e escalas foram retiradas do manual acima.
def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))


AUTODETECT = "-"
SENSORTAG_2650 = "CC2650" # Versão Sensor - tag 

class SensorBase:
    sensorOn  = struct.pack("B", 0x01)
    sensorOff = struct.pack("B", 0x00)
    period = struct.pack("B", 0x0A)  # Relativo a um periodo de amostragem de 100ms  

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
            self.per.write(self.period) #Especificação do periodo de amostragem para cada sensor

    def read(self):
        return self.data.read()

    def disable(self):
        if self.ctrl is not None:
            self.ctrl.write(self.sensorOff)

class MovementSensorMPU9250(SensorBase):
    svcUUID  = _TI_UUID(0xAA80) #Bloco hexa relativo as notificações
    dataUUID = _TI_UUID(0xAA81) #Bloco hexa relativo aos dados
    ctrlUUID = _TI_UUID(0xAA82) #Bloco hexa relativo as configurações
    perUUID = _TI_UUID(0xAA83) #Bloco hexa relativo ao periodo de amostragem
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
        rawVals = self.sensor.rawRead()[3:6]
        return tuple([ v*self.scale for v in rawVals ])

class MagnetometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 4912.0 / 32760

    def enable(self):
        self.sensor.enable(self.sensor.MAG_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.MAG_XYZ)

    def read(self):
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
            self.gyroscope = GyroscopeSensorMPU9250(self._mpu9250)

    def __init__(self):
        DefaultDelegate.__init__(self)
        self.lastVal = 0

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

    # Iniciando 
    time.sleep(0.5)# Esse tempo é especificado pelo o fabricante para iniciar os sensores

    counter=0
    while not rospy.is_shutdown():
        # Get dos dados dos sensores 
        acc = tag.accelerometer.read()
        gyo = tag.gyroscope.read()

        # Envio dos dados adquiridos para o ROS 
        imu.sensor_data.angular_velocity.x=gyo[0]
        imu.sensor_data.angular_velocity.y=gyo[1]
        imu.sensor_data.angular_velocity.z=gyo[2]
        imu.sensor_data.linear_acceleration.x = acc[0]
        imu.sensor_data.linear_acceleration.y = acc[1]
        imu.sensor_data.linear_acceleration.z = acc[2]
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
   