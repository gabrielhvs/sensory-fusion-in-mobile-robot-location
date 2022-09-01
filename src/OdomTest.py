import csv
from cmath import pi
from math import  cos, sin
import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick

arquivo = open('src/Ararajuba/src/datas/test.csv')

linhas = csv.reader(arquivo)

time = []
x= []
y=[]
theta_odom=[]
theta_imu=[]

R = (11.16/2)/100 #in meters
L = (2*2.13+13.84+13.35)/200 #in meters

for linha in linhas:
    time.append((float(linha[0])%10000)+(float(linha[1])/(10**9)))
    x.append(float(linha[2]))
    y.append(float(linha[3]))
    theta_odom.append(float(linha[4]))
    theta_imu.append(float(linha[5]))

time = np.array(time) - time[0]

x = np.array(x[450:])
y = np.array(y[450:])

theta_imu = np.array(theta_imu) - theta_imu[0]


'''
plt.figure(2)
plt.plot(time, np.cos(theta_imu), time, np.cos(theta_odom))'''
theta_odom = np.array(theta_odom[450:])
time = np.array(time[450:])
theta_imu = (theta_imu[450:])

def drawRobot(x,y,q,s):

        p = np.array([[1       ,       1/7 ],    
                      [-3/7    ,        1  ],     
                      [-5/7    ,        6/7],     
                      [-5/7    ,        5/7],     
                      [-3/7    ,        2/7],     
                      [-3/7    ,        0  ],     
                      [-3/7    ,       -2/7],     
                      [-5/7    ,       -5/7],     
                      [-5/7    ,       -6/7],     
                      [-3/7    ,       -1  ],    
                      [1       ,      -1/7 ],    
                      [1       ,       1/7 ]])
    
    
        p = np.dot(s,p)
        p = np.hstack((p, np.ones((len(p),1))))
        
        r = [[ cos(q),sin(q)],
             [-sin(q),cos(q)],
             [   x   ,  y   ]]

        p = np.dot(p,r)

        X = p[:,0] 
        Y = p[:,1] 
        plt.figure(1)
        plt.plot(X,Y,'-r',1.5)
'''
for i in range(0, int(len(x)), int(len(x)/(10))):
        drawRobot(x[i],y[i],theta_imu[i],0.05)#,'--red')

plt.figure(1)
plt.plot(x, y)

plt.suptitle('Odometry-Pose')
plt.xlabel('Distance X - (m)')
plt.ylabel('Distance Y - (m)')
'''
dx =[0]
dy =[0]

for i in range(len(x)-1):
    dx.append(x[i+1]-x[i])
    dy.append(y[i+1]-y[i])

dx = np.array(dx)
dy = np.array(dy)

distance_center = dx/np.cos(theta_odom)

dx_imu = 0.06*np.cos(theta_imu)
dy_imu = 0.06*np.sin(theta_imu)

x_imu =[0]
y_imu =[0]

for i in range(len(dx)-1):
    x_imu.append(x_imu[-1]+dx_imu[i])
    y_imu.append(y_imu[-1]+dy_imu[i])

plt.figure(3)
#plt.plot(distance_center_X)
#plt.plot(x_imu,y_imu)
plt.plot(np.sin(theta_imu))

print(np.average(distance_center))
plt.show()
