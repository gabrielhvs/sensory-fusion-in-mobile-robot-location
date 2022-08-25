import csv
import scipy
from cmath import pi
from ellipse import LsqEllipse
from matplotlib.patches import Ellipse
from math import  atan2, sin, cos,sqrt, asin
import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick

def Kalman_Filfer(magx, magy, xe):
        xe = np.array(xe)
        I=np.eye(2,2)
        P=I*10**6
        r=0.5
        R=I*r**2
        q=0.005
        Q=I*q**2


        
        
        U=np.array([[magx], [magy]]) #->Envio Velocidades U(n)

        
        yl = np.array([[magx], [magy]]) #
        
        ########## Build Matrixs ###########
        
        L =np.array([[1, 0],
                     [0, 1]])

        F = np.array([[1, 0],
                      [0, 1]])

        G = np.eye(2,2)

        #Γ = np.array([[1, 0, -v*dt*sin(xe[2][0])],
        #            [0, 1, v*dt*cos(xe[2][0])],
        #            [0, 0, 1]])
        
        #Λ = np.eye(3,3)
        
        ########### Filtro de Kalman #############

        #x[n+1|n] = f(x[n|n], u[n], 0)
        xp = xe + np.dot(L,U)

        #y[n|n−1] = g(x[n|n−1], 0)
        yp=np.dot(G,xp)

        #P[n+1|n] = F[n]PFT[n] + Γ[n]Q[n]ΓT[n]
        P = np.dot(np.dot(F,P),np.transpose(F)) + Q # np.dot(np.dot(Γ,Q),np.transpose(Γ))
        
        #K[n] = P[n|n−1]GT[n](G[n]P[n|n−1]G[n] + Λ[n]R[n]ΛT[n])−1

        PtG = np.dot(P,np.transpose(G))
        GPtGR= np.linalg.inv(np.dot(G, PtG) + R)
        K = np.dot(PtG,GPtGR)
        
        #x[n|n] = x[n|n−1] + K[n](y[n] − y[n|n−1])
        xe=xp+np.dot(K,(yl-yp))
        #P[n|n] = P[n|n−1] − K[n]G[n]P[n|n−1]
        P=P-np.dot(np.dot(K,G),P)

        return list(xe)

arquivo = open('/home/robottraining03/Ararajuba/src/Extras/IMU-SensorTag/src/params/StableTest.csv')

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

## Calc Orientation using Giroscopic applied Error Bias

arquivo = open('/home/robottraining03/Ararajuba/src/Extras/IMU-SensorTag/src/params/Test1000.csv')

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

seno = np.array([ sin(pi*angle[i]/180) for i in range(len(angle))])
angle = np.array([ 180*asin(sin(angle[i]))/pi for i in range(len(angle))])
cosseno = np.array([ cos(angle[i]) for i in range(len(angle))])

ang = np.array([180*atan2(Mag[100+i,1],Mag[100+i,0])/pi for i in range(len(Mag[100:]))])

plt.figure(2)
plt.plot(time[200:300], ang[100:200])
plt.suptitle('Imu - Yaw')
plt.xlabel('time - (s)')
plt.ylabel('Yaw - (°)')
plt.figure(3)
plt.plot(angle) 


plt.show()