import csv
import scipy
from cmath import pi
from ellipse import LsqEllipse
from matplotlib.patches import Ellipse
from math import  atan2, sin, cos,sqrt, asin,tan
import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick
import allantools

arquivo = open('src/Ararajuba/Devices/IMU-SensorTag_pkg/src/params/StableTest.csv')

linhas = csv.reader(arquivo)

DataCS = []

for linha in linhas:
    DataS = [float(linha[0])]
    for i in range(1,len(linha)):
        l=linha[i].replace('[','')
        l=l.replace(']','')
        index1= l.find(',')
        index2 = l[index1+1:].find(',')
        DataS.append([float(l[0:index1]),float(l[index1+2:index1+index2+1]),float(l[index1+index2+2:])])
    DataCS.append(DataS) 
#print(DataC)

timeS = np.array([DataCS[i][0] for i in range(len(DataCS))])
timeS = timeS - timeS[0]
GyoS  = np.array([DataCS[i][1] for i in range(len(DataCS))])
AccS  = np.array([DataCS[i][2] for i in range(len(DataCS))])
MagS  = np.array([DataCS[i][3] for i in range(len(DataCS))])


VarM = []
VarG = []
VarA = []
miM = []
miG = []
miA = []
stdM = []
stdG = []
stdA = []

## Normal distribuição
for i in range(3):

    miM.append(np.average(MagS[100:,i]))
    miG.append(np.average(GyoS[100:,i]))
    miA.append(np.average(AccS[100:,i]))
   
    MagS[100:,i]=(MagS[100:,i]-min(MagS[100:,i]))/(max(MagS[100:,i])-min(MagS[100:,i]))
    GyoS[100:,i]=(GyoS[100:,i]-min(GyoS[100:,i]))/(max(GyoS[100:,i])-min(GyoS[100:,i]))
    AccS[100:,i]=(AccS[100:,i]-min(AccS[100:,i]))/(max(AccS[100:,i])-min(AccS[100:,i]))
    
    VarM.append(np.var(MagS[100:,i]))
    VarG.append(np.var(GyoS[100:,i]))
    VarA.append(np.var(AccS[100:,i]))
    
    stdM.append(sqrt(np.var(MagS[100:,i])))
    stdG.append(sqrt(np.var(GyoS[100:,i])))
    stdA.append(sqrt(np.var(AccS[100:,i])))
   

print("Magnometer Data: \n averange: " + str(miM) + "\n variance: " + str(VarM) + "\n standard error: " + str(stdM) )
print("Gyroscopic Data: \n averange: " + str(miG) + "\n variance: " + str(VarG) + "\n standard error: " + str(stdG) )
print("Accelerometer Data: \n averange: " + str(miA) + "\n variance: " + str(VarA) + "\n standard error: " + str(stdA) )

### Histogram ###

arquivo = open('src/Ararajuba/Devices/IMU-SensorTag_pkg/src/params/Sample6.csv')

linhas = csv.reader(arquivo)
linhas =list(linhas)
DataC = []

for linha in linhas:

    Data=[]
    for i in linha:
        Data.append(float(i))
    DataC.append(Data)

MagX = [DataC[i][0] for i in range(len(DataC))]
MagY = [DataC[i][1] for i in range(len(DataC))]
MagZ = [DataC[i][2] for i in range(len(DataC))]
time = np.linspace(0,len(MagX[100:]) * 0.4, len(MagX[100:]))

ang = np.array([180*atan2(MagY[i],MagX[i])/pi for i in range(len(MagX[100:]))])

'''
plt.figure(1)
plt.plot( time, ang)
plt.figure(2)
plt.hist(ang[:], 20, rwidth=0.9)
'''
### Magwick filter

arquivo = open('src/Ararajuba/Devices/IMU-SensorTag_pkg/src/params/Sample7.csv')

linhas = csv.reader(arquivo)
linhas =list(linhas)
DataC = []

for linha in linhas:

    Data=[]
    for i in linha:
        Data.append(float(i))
    DataC.append(Data)

time = np.array([(float(DataC[i][0])%10000)+(float(DataC[i][1])/(10**9)) for i in range(len(DataC))])
time = time - time[0]

Gyo = np.array([DataC[i][2:5] for i in range(len(DataC))])

Acc = np.array([DataC[i][5:8] for i in range(len(DataC))])
print(Acc)

Mag = np.array([DataC[i][8:] for i in range(len(DataC))])

Gyo[:,0] = Gyo[:,0] - np.average(Gyo[:200,0])
Gyo[:,1] = Gyo[:,1] - np.average(Gyo[:200,1])
Gyo[:,2] = Gyo[:,2] - np.average(Gyo[:200,2])

angle = np.array([(Gyo[i,2])*(time[i]-time[i-1])  for i in range(1,len(time))])

for i in range(1,len(angle)):
    a = angle[i-1]
    b = angle[i]
    angle[i] = a+b

angleGyo = np.array([ atan2(sin(pi*angle[i]/180), cos(pi*angle[i]/180)) for i in range(len(angle))])
plt.plot(time[1:], angleGyo)



angleMag = np.array([atan2(Mag[i,1],Mag[i,0]) for i in range(len(Mag))])
plt.plot(time[:], angleMag)


from ahrs.filters import Madgwick
madgwick = Madgwick()
Q = np.tile([1., 0., 0., 0.], (len(Gyo), 1)) # Allocate for quaternions
for t in range(1, len(Gyo)):
   Q[t] = madgwick.updateMARG(Q[t-1], gyr=Gyo[t], acc=Acc[t], mag=Mag[t])

yaw = []

for i in range(len(Q)):
    t3 = +2.0 * (Q[i,0] * Q[i,3]  + Q[i,1]  * Q[i,2])
    t4 = +1.0 - 2.0 * (Q[i,2]  * Q[i,2]  + Q[i,3]  * Q[i,3])
    yaw.append(atan2(t3, t4))
     
plt.plot(time, yaw)
plt.legend(['angle_Gyo', 'angle_Mag', 'angle_Filter'])


plt.show()




'''

#############     Using Allan Variance 
timeS = np.array([DataCS[i][0] for i in range(len(DataCS))])
timeS = timeS - timeS[0]
GyoS  = np.array([DataCS[i][1] for i in range(len(DataCS))])
AccS  = np.array([DataCS[i][2] for i in range(len(DataCS))])
MagS  = np.array([DataCS[i][3] for i in range(len(DataCS))])

miM = []
miG = []
miA = []

for i in range(3):

    miM.append(np.average(MagS[100:,i]))
    miG.append(np.average(GyoS[100:,i]))
    miA.append(np.average(AccS[100:,i]))
   
    MagS[100:,i]=(MagS[100:,i]-min(MagS[100:,i]))/(max(MagS[100:,i])-min(MagS[100:,i]))
    GyoS[100:,i]=(GyoS[100:,i]-min(GyoS[100:,i]))/(max(GyoS[100:,i])-min(GyoS[100:,i]))
    AccS[100:,i]=(AccS[100:,i]-min(AccS[100:,i]))/(max(AccS[100:,i])-min(AccS[100:,i]))

n = 1000
taus = np.linspace(0.4,50,n)
plt.figure(2)
(t1, ad1, ade, adn) = allantools.oadev(MagS[100:,0], rate=1/0.4, data_type="freq", taus=taus)
(t2, ad2, ade, adn) = allantools.oadev(MagS[100:,1], rate=1/0.4, data_type="freq", taus=taus)
(t3, ad3, ade, adn) = allantools.oadev(MagS[100:,2], rate=1/0.4, data_type="freq", taus=taus)

plt.plot(t1,ad1,t2,ad2)
plt.xscale("log")
plt.yscale("log")
plt.xlabel('Time Cluster (sec)')
plt.ylabel('Allan Deviation')

## Calc Orientation using Giroscopic applied Error Bias

arquivo = open('src/Ararajuba/Devices/IMU-SensorTag_pkg/src/params/Test1000.csv')

linhas = csv.reader(arquivo)


time = []
Gyo  = []
Acc  = []
Mag  = []
DataC=[]

for linha in linhas:
    time.append((float(linha[0])%10000)+(float(linha[1])/(10**9)))
    Data=[]
    for i in range(2,len(linha)):
        l=linha[i].replace('[','')
        l=l.replace(']','')
        index1= l.find(',')
        index2 = l[index1+1:].find(',')
        
        Data.append([float(l[0:index1]),float(l[index1+2:index1+index2+1]),float(l[index1+index2+2:])])
    DataC.append(Data)


Gyo  = np.array([DataC[i][0] for i in range(len(DataC))])
Acc  = np.array([DataC[i][1] for i in range(len(DataC))])
Mag  = np.array([DataC[i][2] for i in range(len(DataC))])
time = np.array(time) - time[0]

X1, X2 = Mag[:,0], Mag[:,1]

X = np.array(list(zip(X1, X2)))
reg = LsqEllipse().fit(X)
center, width, height, phi = reg.as_parameters()

print(f'center: {center[0]:.3f}, {center[1]:.3f}')
print(f'width: {width:.3f}')
print(f'height: {height:.3f}')
print(f'phi: {phi:.3f}')


fig = plt.figure(1,figsize=(6, 6))
ax = plt.subplot()
ax.axis('equal')
ax.plot(X1, X2, 'ro', zorder=1)

ellipse = Ellipse(
    xy=center, width=2*width, height=2*height, angle=np.rad2deg(phi),
    edgecolor='b', fc='None', lw=2, label='Fit', zorder=2
)
ax.add_patch(ellipse)

plt.xlabel('$Mag_X$')
plt.ylabel('$Mag_Y$')
plt.legend(['Mag', 'Fitt'])

Mag[:,0] = Mag[:,0] - center[0] 
Mag[:,1] = Mag[:,1] - center[1] 


angle = np.array([(Gyo[i,2]-miG[2])*(time[i]-time[i-1])  for i in range(1,len(time))])


for i in range(1,len(angle)):
    a = angle[i-1]
    b = angle[i]
    angle[i] = a+b

seno = np.array([ sin(angle[i]) for i in range(len(angle))])
cosseno = np.array([ cos(angle[i]) for i in range(len(angle))])
angle = np.array([ 180*atan2(seno[i], cosseno[i])/pi for i in range(len(angle))])


ang = np.array([180*atan2(Mag[100+i,1],Mag[100+i,0])/pi for i in range(len(Mag[100:]))])

plt.figure(3)
plt.plot(ang[100:])
plt.suptitle('Imu - Yaw')
plt.xlabel('time - (s)')
plt.ylabel('Yaw - (°)')
plt.figure(4)
plt.plot(angle) 


plt.show()'''