import csv
from cmath import pi
from math import  atan2, asin
import matplotlib.pyplot as plt
import numpy as np
from ellipse import LsqEllipse
from matplotlib.patches import Ellipse
from ahrs.filters import Madgwick

arquivo = open('/home/robottraining03/Ararajuba/src/Extras/sensor_tag_pkg/src/params/ImuData.csv')

linhas = csv.reader(arquivo)

DataC = []

for linha in linhas:
    Data = []
    for i in range(len(linha)):
        l=linha[i].replace('[','')
        l=l.replace(']','')
        index1= l.find(',')
        index2 = l[index1+1:].find(',')
        Data.append([float(l[0:index1]),float(l[index1+2:index1+index2+1]),float(l[index1+index2+2:])])
    DataC.append(Data) 

MagX = [DataC[0][i][0] for i in range(len(DataC[0]))]
MagY = [DataC[0][i][1] for i in range(len(DataC[0]))]
MagZ = [DataC[0][i][2] for i in range(len(DataC[0]))]

GyoX = [DataC[1][i][0] for i in range(len(DataC[1]))]
GyoY = [DataC[1][i][1] for i in range(len(DataC[1]))]
GyoZ = [DataC[1][i][2] for i in range(len(DataC[1]))]

AccX = [DataC[2][i][0] for i in range(len(DataC[2]))]
AccY = [DataC[2][i][1] for i in range(len(DataC[2]))]
AccZ = [DataC[2][i][2] for i in range(len(DataC[2]))]


X1, X2 = MagX[0:280], MagY[0:280]

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



MagX = [MagX[i] - center[0] for i in range(len(MagX))]
MagY = [MagY[i] - center[1] for i in range(len(MagY))]
plt.figure(2)
plt.scatter(MagX[0:280], MagY[0:280])
plt.xlabel('$Mag_X$')
plt.ylabel('$Mag_Y$')
plt.legend()

MagX = [MagX[i]/height for i in range(len(MagX))]
MagY = [MagY[i]/width for i in range(len(MagY))]
plt.figure(3)
plt.scatter(MagX[0:280], MagY[0:280])
plt.xlabel('$Mag_X$')
plt.ylabel('$Mag_Y$')
plt.legend()



############### Using Filter of Magwick #################

madgwick = Madgwick(Dt=0.01)

Q = np.tile([1., 0., 0., 0.], (len(GyoX), 1)) # Allocate for quaternions
for t in range(1, len(GyoX)):
    Q[t] = madgwick.updateMARG(Q[t-1], gyr=[GyoX[t],GyoY[t],GyoZ[t]], acc=[AccX[t],AccY[t],AccZ[t]], mag=[MagX[t],MagY[t],MagZ[t]])

E = []

for i in Q:
    t0 = +2.0 * (i[0] * i[1] + i[2] * i[3])
    t1 = +1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2])
    roll_x = 180*atan2(t0, t1)/pi

    t2 = +2.0 * (i[0] * i[2] - i[3] * i[1])
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = 180*asin(t2)/pi

    t3 = +2.0 * (i[0] * i[3] + i[1] * i[2])
    t4 = +1.0 - 2.0 * (i[2] * i[2] + i[3] * i[3])
    yaw_z = 180*atan2(t3, t4)/pi

    E.append([roll_x,pitch_y,yaw_z])


plt.figure(4)
#plt.plot(Q)

plt.subplot(2,1,1)
plt.plot([180*atan2(MagY[i],MagX[i])/pi for i in range(len(MagX[0:280])) ])
plt.legend(['Direct Sensor'])
plt.subplot(2,1,2)
plt.plot([i[2] for i in E][0:280])
plt.legend(['Filter Data'])
plt.show()
