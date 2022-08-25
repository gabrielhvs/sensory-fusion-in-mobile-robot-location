# Gazebo ROS1

# Pré requisito
- Ubuntu 20.04
- ROS Noetic 

# instalação
```bash
Aplicar o source no ROS1
$ source /opt/ros/noetic/setup.bash

Criar o espaço de trabalho
$ mkdir -p RobotTraining_ROS1_ws/src && cd RobotTraining_ROS1_ws/src

Adicionar o repositório
$ git clone https://github.com/davijgsousa/RobotTraining_ROS1.git

Voltar para pasta raiz do espaço de trabalho
$ cd ~/RobotTraining_ROS1_ws

Rodar o catkin
$ catkin_make
```
# Para usar:
```
Abra 2 terminais diferentes e execute os comandos em cada um:
$ cd ~/RobotTraining_ROS1_ws
$ source devel/setup.bash

No terminal 1 execute o roscore
$ roslaunch gazebo_ros empty_world.launch

No terminal 2 execute o simulador
$ roslaunch p3dx_gazebo p3dx.launch

Argumentos p3dx.launch:
* ns:= (namespace) | default: p3dx
* x:= (x position) | default: 0.0
* y:= (y position) | default: 0.0
* z:= (z position) | default: 0.0

Gerando dois ou mais robôs
$ roslaunch p3dx_gazebo p3dx.launch ns:=p3dx_1
$ roslaunch p3dx_gazebo p3dx.launch ns:=p3dx_2 x:=2
$ roslaunch p3dx_gazebo p3dx.launch ns:=p3dx_3 x:=2 y:=2
```
