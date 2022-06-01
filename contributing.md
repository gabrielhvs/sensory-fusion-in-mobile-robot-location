## :hammer: Funcionalidades do projeto

### 1. Modelo Gazebo

`Gabriel`

Implementação de um modelo de robô de tração diferencial no Gazebo, utilização do modelo e especificações física do Pioneer 3-DX. Foi utilizado o 
pacote disponibilizado por Davi para execução das atividades da capacitação referentes a ROS.

##
### 2. Implementação de Sensores
 
Implementação dos sensores disponíveis na plataforma modelada no ambiente de simulação por meio de alterações no .xacro e .gazebo, incluindo modelos de ruídos.

#### 2.1. Encoder 

`Gabriel`

A implementação foi feita por meio do plugin libgazebo_ros_diff_drive.so, que já é utilizado para controle das velocidades de roda do modelo. Para que fosse atribuída medidas simulando medições de um encoder de interrupção real foi alterado o parâmetro OdometrySource. Assim a posição é calculada de acordo com a velocidade desenvolvida por cada uma das rodas.

#### 2.2. IMU

`Gabriel`

O plugin utilizado para aplicação deste sensor foi libgazebo_ros_imu_sensor.so que é um plugin generico para este tipo de sensor. Além disso, nas leituras geradas foram introduzidos ruídos gaussianos para simular os erros encontrados em uma leitura real. Para melhor visualização na plataforma Rviz e posterior filtragem foi adicionado o pacote imu_tools

#### 2.3. LiDAR

`Breno`


#### 2.4. Sonares

`Breno` `Marina`

#### 2.5. Câmera RGB-D

`Marina`

##
### 3. Fusão dos Sensores

#### 3.1. Filtro de Kalman Extendido - Odometria e IMU

`Gabriel`

Implementação do pacote robot_pose_ekf, que tem o intuito de aplicar um EKF por meio dos dados da odometria e do IMU gerando uma medida mais aproximada da posição real do modelo. Já que o IMU gera leituras de ângulos e de acelerações, esses dados auxiliam em uma melhor estimação. Já que um dos principais problemas da odometria são devido aos deslizamentos de roda.  

##
### Extra 

#### Teleop

`Breno`

Implementação de um código em Python para leitura de teclas pressionadas para controle da plataforma. A partir da publicação de velocidades lineares e 
angulares no tópico \cmd_vel. 

