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

Implementação das caracteristicas físicas e especificações de um modelo de LiDAR 2D, o LMS100, devido a disponibilidade de um modelo semelhante no laboratório. Foi  utilizando o plugin libgazebo_ros_laser. Em suas leituras foram introduzidos ruídos gaussianos para simular os erros de leitura do sensor real. Com objetivo de futura alteração para um modelo 3D da velodyme.

#### 2.4. Sonares

`Breno` `Marina`

Implementação das caracteristicas físicas e especificações de um modelo de sonar genérico, foi  utilizando o plugin libgazebo_ros_range com especificação de tipo de radiação, utilizando o tipo ultrasound. Em suas leituras foram introduzidos ruídos gaussianos para simular os erros de leitura do sensor real. Atualmente foram implementados apenas 3 sensores distribuídos ao redor da plataforma, no entanto posteriormente será implementado um cinturão de 10 sonares distribuidos em volta do robô. 

#### 2.5. Câmera RGB-D

`Marina`

Implementação do modelo Microsoft Kinect utilizando o plugin libgazebo_ros_openni_kinect.so no gazebo. Este modelo de sensor gera uma nuvem de pontos com informação de posição 3D associada a cada ponto. Além disso, por meio do RVIZ é possível vizualizar as imagens de profundidade e RGB.
No ROS, o modelo disponibiliza 2 tópicos principais para obter informações do sensor em 2D e 3D.
##
### 3. Fusão dos Sensores

#### 3.1. Filtro de Kalman Estendido - Odometria e IMU

`Gabriel`

Implementação do pacote robot_pose_ekf, que tem o intuito de aplicar um EKF por meio dos dados da odometria e do IMU gerando uma medida mais aproximada da posição real do modelo. Já que o IMU gera leituras de ângulos e de acelerações, esses dados auxiliam em uma melhor estimação. Já que um dos principais problemas da odometria são devido aos deslizamentos de roda.  

#### 3.2. Deteccão de objetos por cor - Câmera RGB (test_camera.py)

`Marina`

Código em python que utiliza openCV e o pacote ROS CV_Bridge para detectar objetos de cor amarela e publica a localização (x,y) deste. O código está sendo construido para posteriormente ser utilizado na identificação de marcadores no mapa e retornar dados que informem a posição do robô em relação ao marcador detectado. 

#### 3.3. Odometria LiDAR

`Breno`

Implementação do pacote laser scan matcher com o objetivo de utilizar as leituras do LiDAR para redundância da odometria. Além disso, utilização do IMU e Odometria de roda para melhor previsão.

##
### Extra 

#### Teleop

`Breno`

Implementação de um código em Python para leitura de teclas pressionadas para controle da plataforma. A partir da publicação de velocidades lineares e 
angulares no tópico \cmd_vel. 

#### Modularização

`Breno` `Gabriel` `Marina`

Para uma melhor organização do projeto realizou-se uma modularização dos arquivos para melhor manipulação dos sensores e plataforma, em virtude do robô utilizado atualmente não ser a plataforma Ararajuba.

