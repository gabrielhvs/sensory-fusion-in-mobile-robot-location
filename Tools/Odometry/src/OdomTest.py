import csv
from cmath import pi
from math import  cos, sin
import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick

arquivo = open('/home/robottraining03/Ararajuba/src/Extras/IMU-SensorTag/src/params/Odom2000.csv')

linhas = csv.reader(arquivo)

timeLeft = []
timeRight = []
velLeft= []
velRight=[]
theta=[]

R = (11.16/2)/100 #in meters
L = (2*2.13+13.84+13.35)/200 #in meters

for linha in linhas:
    timeLeft.append((float(linha[0])%10000)+(float(linha[1])/(10**9)))
    timeRight.append((float(linha[3])%10000)+(float(linha[4])/(10**9)))
    velLeft.append((float(linha[2])*60)/(2*pi*R))
    velRight.append((float(linha[5])*60)/(2*pi*R))


timeLeft = np.array(timeLeft) - timeLeft[0]
timeRight = np.array(timeRight) - timeRight[0]

distLeft = [((velLeft[i]/60)*2*pi*R)*(timeLeft[i]-timeLeft[i-1]) for i in range (1,len(timeLeft))]
distRight = [((velRight[i]/60)*2*pi*R)*(timeRight[i]-timeRight[i-1]) for i in range (1,len(timeRight))]

Vl = [(((velLeft[i]/60)*2*pi*R) + ((velRight[i]/60)*2*pi*R))/2 for i in range (len(velRight))]
W = [(((velLeft[i]/60)*2*pi*R) - ((velRight[i]/60)*2*pi*R))/(2*L) for i in range (len(velRight))]

print(Vl[0],W[0])

distance_center = [(distLeft[i] + distRight[i])/2 for i in range(len(distLeft))]
dtheta = [(distRight[i] - distLeft[i])/(2*L) for i in range(len(distLeft))]

X = [0]
Y = [0] 
Theta = [0]

for i in range(len(distance_center)):
    X.append(X[-1]+distance_center[i]*cos(Theta[i]))
    Y.append(Y[-1]+distance_center[i]*sin(Theta[i]))
    Theta.append(Theta[-1]+dtheta[i])

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
        plt.plot(X,Y,'-r',1.5)


for i in range(0, int(len(distance_center)/7), int(len(distance_center)/(40*7))):
        drawRobot(X[i],Y[i],Theta[i],0.05)#,'--red')


plt.figure(1)
plt.plot(X[:int(len(distance_center)/7)],Y[:int(len(distance_center)/7)])
plt.figure(2)
plt.plot(X[:],Y[:])
plt.suptitle('Odometry-Pose')
plt.xlabel('Distance X - (m)')
plt.ylabel('Distance Y - (m)')

plt.show()