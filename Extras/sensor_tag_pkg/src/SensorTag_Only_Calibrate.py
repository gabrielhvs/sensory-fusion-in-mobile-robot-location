from cmath import pi
from math import  atan2, sqrt
from bluepy.btle import UUID, Peripheral, AssignedNumbers
import struct
import time
import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Sensortag versions
AUTODETECT = "-"
SENSORTAG_V1 = "v1"
SENSORTAG_2650 = "CC2650"

# 
######################################
# Funções Relativas as conexões da Tag
######################################
#

def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))

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

##
#Refer: https://makersportal.com/blog/calibration-of-a-magnetometer-with-raspberry-pi
##
# 
#####################################
# Gyro calibration (Steady)
#####################################
#

def gyro_cal(tag):
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    mpu_array = [] # imu array for gyro vals
    gyro_offsets = [0.0,0.0,0.0] # gyro offset vector
    while True:
        try:
            wx,wy,wz = tag.gyroscope.read() # read and convert mpu6050 data
            tag.waitForNotifications(1/50)
        except:
            continue

        mpu_array.append([wx,wy,wz]) # gyro vector append
        tag.waitForNotifications(1/50)
        if np.shape(mpu_array)[0]==cal_size:
            for qq in range(0,3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:,qq]) # calc gyro offsets
            break
    print('Gyro Calibration Complete')
    return gyro_offsets # return gyro coeff offsets

# 
#####################################
# Accel Calibration (gravity)
#####################################
#

def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration
#
def get_accel(tag):
    ax,ay,az = tag.accelerometer.read() # read and convert accel data
    return ax,ay,az
    
def accel_cal(tag):
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = get_accel(tag) # get accel variables
                    mpu_array.append([ax,ay,az]) # append to array
                    tag.waitForNotifications(1/50)
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets

# 
#####################################
# Mag Calibration Fitting
#####################################
#

def outlier_removal(x_ii,y_ii):
    x_diff = np.append(0.0,np.diff(x_ii)) # looking for outliers
    stdev_amt = 5.0 # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff)>np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff))) # outlier in x-var
    x_inliers  = np.where(np.abs(x_diff)<np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    y_diff     = np.append(0.0,np.diff(y_ii)) # looking for outliers
    y_outliers = np.where(np.abs(y_diff)>np.abs(np.mean(y_diff))+\
                          (stdev_amt*np.std(y_diff))) # outlier in y-var
    y_inliers  = np.abs(y_diff)<np.abs(np.mean(y_diff))+\
                 (stdev_amt*np.std(y_diff)) # outlier vector
    if len(x_outliers)!=0:
        x_ii[x_outliers] = 0 # null outlier
        y_ii[x_outliers] = 0 # null outlier
    if len(y_outliers)!=0:
        y_ii[y_outliers] = 0 # null outlier
        x_ii[y_outliers] = 0 # null outlier
    return x_ii,y_ii

def mag_cal(tag):
    print("-"*50)
    print("Magnetometer Calibration")
    cal_rot_indices = [[0,1],[1,2],[0,2]] # indices of heading for each axis
    mag_cal_rotation_vec = [] # variable for calibration calculations
    for qq,ax_qq in enumerate(mag_cal_axes):
        input("-"*8+" Press Enter and Start Rotating the IMU Around the "+ax_qq+"-axis")
        print("\t When Finished, Press CTRL+C")
        mag_array = []
        t0 = time.time()
        count = 0
        while count<cal_size:
            mx,my,mz = tag.magnetometer.read() # read and convert AK8963 magnetometer data
            tag.waitForNotifications(1/50)
            mag_array.append([mx,my,mz]) # mag array
            count = count + 1
            print(count)
        mag_array = mag_array[20:] # throw away first few points (buffer clearing)
        mag_cal_rotation_vec.append(mag_array) # calibration array
        print("Sample Rate: {0:2.0f} Hz".format(len(mag_array)/(time.time()-t0)))
        
    mag_cal_rotation_vec = np.array(mag_cal_rotation_vec) # make numpy array
    ak_fit_coeffs = [] # mag fit coefficient vector
    indices_to_save = [0,0,1] # indices to save as offsets
    for mag_ii,mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags) # mag numpy array
        x,y = mags[:,cal_rot_indices[mag_ii][0]],\
                        mags[:,cal_rot_indices[mag_ii][1]] # sensors to analyze
        x,y = outlier_removal(x,y) # outlier removal
        y_0 = (np.nanmax(y)+np.nanmin(y))/2.0 # y-offset
        x_0 = (np.nanmax(x)+np.nanmin(x))/2.0 # x-offset
        ak_fit_coeffs.append([x_0,y_0][indices_to_save[mag_ii]]) # append to offset
        
    return ak_fit_coeffs
# 
#####################################
# Plot Real-Time Values to Test
#####################################
#

    #
    #############################
    # Figure/Axis Formatting
    #############################
    #
    plt.style.use('ggplot') # stylistic visualization
    fig = plt.figure(figsize=(12,9)) # start figure
    axs = [[],[],[],[]] # axis vector
    axs[0] = fig.add_subplot(321) # accel axis
    axs[1] = fig.add_subplot(323) # gyro axis
    axs[2] = fig.add_subplot(325) # mag axis
    axs[3] = fig.add_subplot(122,projection='polar') # heading axis
    plt_pts = 1000 # points to plot
    y_labels = ['Acceleration [g]','Angular Velocity [$^\circ/s$]','Magnetic Field [uT]']
    for ax_ii in range(0,len(y_labels)):
        axs[ax_ii].set_xlim([0,plt_pts]) # set x-limits for time-series plots
        axs[ax_ii].set_ylabel(y_labels[ax_ii]) # set y-labels
    ax_ylims = [[-4.0,4.0],[-300.0,300.0],[-100.0,100.0]] # ax limits
    for qp in range(0,len(ax_ylims)):
        axs[qp].set_ylim(ax_ylims[qp]) # set axis limits
    axs[3].set_rlim([0.0,100.0]) # set limits on heading plot
    axs[3].set_rlabel_position(112.5) # offset radius labels
    axs[3].set_theta_zero_location("N") # set north to top of plot
    axs[3].set_theta_direction(-1) # set rotation N->E->S->W
    axs[3].set_title('Magnetometer Heading') # polar plot title
    axs[0].set_title('Calibrated MPU9250 Time Series Plot') # imu time series title
    fig.canvas.draw() # draw axes
    #
    #############################
    # Pre-allocate plot vectors
    #############################
    #
    dummy_y_vals = np.zeros((plt_pts,)) # for populating the plots at start
    dummy_y_vals[dummy_y_vals==0] = np.nan # keep plots clear
    lines = [] # lines for looping updates
    for ii in range(0,9):
        if ii in range(0,3): # accel pre-allocation
            line_ii, = axs[0].plot(np.arange(0,plt_pts),dummy_y_vals,
                                label='$'+mpu_labels[ii]+'$',color=plt.cm.tab10(ii))
        elif ii in range(3,6): # gyro pre-allocation
            line_ii, = axs[1].plot(np.arange(0,plt_pts),dummy_y_vals,
                                label='$'+mpu_labels[ii]+'$',color=plt.cm.tab10(ii))
        elif ii in range(6,9): # mag pre-allocation
            jj = ii-6
            line_jj, = axs[2].plot(np.arange(0,plt_pts),dummy_y_vals,
                                label='$'+mpu_labels[ii]+'$',color=plt.cm.tab10(ii))
            line_ii, = axs[3].plot(dummy_y_vals,dummy_y_vals,
                                label='$'+mag_cal_axes[jj]+'$-Axis Heading',
                                   color=plt.cm.tab20b(int(jj*4)),
                                   linestyle='',marker='o',markersize=3)
            lines.append(line_jj)
        lines.append(line_ii)
    ax_legs = [axs[tt].legend() for tt in range(0,len(axs))] # legends for axes
    ax_bgnds = [fig.canvas.copy_from_bbox(axs[tt].bbox) for tt in range(0,len(axs))] # axis backgrounds
    fig.show() # show figure
    mpu_array = np.zeros((plt_pts,9)) # pre-allocate the 9-DoF vector
    mpu_array[mpu_array==0] = 0
    #
    #############################
    # Real-Time Plot Update Loop
    #############################
    #
    ii_iter = 0 # plot update iteration counter 
    cal_rot_indicies = [[6,7],[7,8],[6,8]] # heading indices
    while True:
        try:
            [ax,ay,az] = tag.accelerometer.read()
            [wx,wy,wz] = tag.gyroscope.read() # read and convert mpu6050 data
            [mx,my,mz] = tag.magnetometer.read() # read and convert AK8963 magnetometer data
            tag.waitForNotifications(1/50)
        except:
            continue
        
        ax = 1.0000355080059604*ax - 0.0012537066265980723
        ay = -0.9979695014097623*ay - 0.022839678792854312
        az = -0.9882356123153436*az - 0.08896211505290272

        mx = mx - 199.0
        my = my - 354.5
        mz = mz + 960.0

        roll = 180*atan2 (ax,sqrt(ay*ay + az*az))/pi
        pitch = 180*atan2 (ay,sqrt(ax*ax + az*az))/pi

        yaw= 180*atan2(my, mx)/pi

        print([roll, pitch,yaw])

if __name__ == '__main__':
    #
    ###################################
    # Connect Tag
    ###################################
    #
    host = 'A0:E6:F8:AF:6B:83'
    tag = SensorTag(host)
    tag.accelerometer.enable()
    tag.gyroscope.enable()
    tag.magnetometer.enable()
    time.sleep(0.5)
    #
    ###################################
    # input parameters
    ###################################
    #
    mpu_labels = ['a_x','a_y','a_z','w_x','w_y','w_z','m_x','m_y','m_z']
    cal_labels = [['a_x','m','b'],['a_y','m','b'],['a_z','m','b'],'w_x','w_y','w_z',
                    ['m_x','m_x0'],['m_y','m_y0'],['m_z','m_z0']]
    mag_cal_axes = ['z','y','x'] # axis order being rotated for mag cal
    cal_filename = '/home/robottraining03/Ararajuba/src/Extras/sensor_tag_pkg/src/params/mpu9250_cal_params.csv' # filename for saving calib coeffs
    cal_size = 200 # how many points to use for calibration averages
    cal_offsets = np.array([[],[],[],0.0,0.0,0.0,[],[],[]]) # cal vector
    #
    ###################################
    # call to calibration functions
    ###################################
    #
    
    print("-"*50)
    input("Press Enter to Start The Calibration Procedure")
    time.sleep(1)
    gyro_offsets = gyro_cal(tag) # calibrate gyro offsets under stable conditions
    cal_offsets[3:6] = gyro_offsets 
    mpu_offsets = accel_cal(tag) # calibrate accel offsets 
    cal_offsets[0:3] = mpu_offsets
    ak_offsets = mag_cal(tag) # calibrate mag offsets 
    cal_offsets[6:] = ak_offsets
    # save calibration coefficients to file
    with open(cal_filename,'w',newline='') as csvfile:
        writer = csv.writer(csvfile,delimiter=',')
        for param_ii in range(0,len(cal_offsets)):
            if param_ii>2:
                writer.writerow([cal_labels[param_ii],cal_offsets[param_ii]])
            else:
                writer.writerow(cal_labels[param_ii]+
                                [ii for ii in cal_offsets[param_ii]])
