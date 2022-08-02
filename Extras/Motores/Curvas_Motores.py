import csv
from cmath import pi
from math import  atan2, asin
import matplotlib.pyplot as plt
import numpy as np
from ellipse import LsqEllipse
from matplotlib.patches import Ellipse
from sklearn.linear_model import LinearRegression

CurvaCaracteristica = open('/home/robottraining03/Ararajuba/src/Extras/Motores/data/Motor.csv')
CurvaPID = open('/home/robottraining03/Ararajuba/src/Extras/Motores/data/PID_Motores.csv')


linhasCC = csv.reader(CurvaCaracteristica)
linhasPID = csv.reader(CurvaPID)

DataCC = []
DataPID = []

for linha in linhasCC:
    if linha[0].find("Before") == -1:
        if len(linha) == 3:
            linha[0] = linha[0].replace("b'",'')
            linha[-1] = linha[-1].replace("\\r\\",'')
            DataCC.append([float(linha[0]),float(linha[1]),float(linha[2])])

for linha in linhasPID:
    if linha[0].find("Before") == -1:
        if len(linha) == 3:
            linha[0] = linha[0].replace("b'",'')
            linha[-1] = linha[-1].replace("\\r\\",'')
            DataPID.append([float(linha[0]),float(linha[1]),float(linha[2])])


tCC = np.array([DataCC[i][0]/1000.0 for i in range(len(DataCC))])
tCC = tCC - tCC[0]
tPID = np.array([DataPID[i][0]/1000.0 for i in range(len(DataPID))])
tPID = tPID - tPID[0]
directSensorRCC = np.array([DataCC[i][2] for i in range(len(DataCC))])
directSensorLCC = np.array([DataCC[i][1] for i in range(len(DataCC))])
directSensorRPID = np.array([DataPID[i][2] for i in range(len(DataPID))])
directSensorLPID = np.array([DataPID[i][1] for i in range(len(DataPID))])

stepCC = []
stepCC = 180*np.ones(len(DataCC))
stepPID = []
stepPID = 100*np.ones(len(DataPID))


plt.figure(1)
plt.plot(tCC[0:1000],directSensorRCC[0:1000])
plt.plot(tCC[0:1000],directSensorLCC[0:1000])
plt.plot(tCC[0:1000],stepCC[0:1000], '--r', 'o')
plt.xlabel('tempo (s)')
plt.ylabel('velociade de roda (rpm)')
plt.title('Curva Caracteristica - Motores')
plt.legend(["Motor Direito","Motor Esquerdo","PWM Aplicado"])

plt.figure(2)
plt.plot(tPID[0:1000],directSensorRPID[0:1000])
plt.plot(tPID[0:1000],directSensorLPID[0:1000])
plt.plot(tPID[0:1000],stepPID[0:1000], '--r', 'o')
plt.xlabel('tempo (s)')
plt.ylabel('velociade de roda (rpm)')
plt.title('PID - Motores')
plt.legend(["Motor Direito","Motor Esquerdo","set point"])
plt.show()