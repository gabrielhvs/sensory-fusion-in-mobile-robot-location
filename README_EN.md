<h1 align="center"><a>Sensory Fusion in Mobile Robot</a></h1>

<p align="center">
   <img alt="GitHub build" src="http://img.shields.io/static/v1?label=STATUS&message=UNDER%20DEVELOPMENT&color=GREEN&style=for-the-badge"></a>
</p>

<p align="center">
  <span>English</span> |
  <a href="README.md">Portugues</a>
</p>

## Description
In general, the project aims to apply the knowledge acquired within the eRobotics Laboratory of the Electrical Engineering Department of the Federal University of Campina Grande from classes given by professors Antonio Marcus Nogueira Lima, Marcos Ricardo Alcântara Morais and Gutemberg Gonçalves dos Santos Júnior. . Through the creation of a robotic differential traction platform with different sensors, study and finally solve the problem of location. Performed by reading and merging data from sensors, both proprioceptive and exteroceptive, for example, optical encoders, LiDAR, IMU, RGB-D Camera. This fusion was a priori, through the Extended Kalman Filter (EKF - Extend Kalman Filter). The simulations and initial studies were carried out on the Gazebo simulation platform, using the ROS software to perform the communication. Use and comparison of different mapping and location methods.

## Planning

| #        | Steps          | _Status_ |
|:--|:----------------------------|:------------------|
| 1 | Model implementation in Gazebo    | <img alt="Finished" src="https://img.shields.io/badge/-Finished-brightgreen">|
| 2 | Inclusion of sensors in the model | <img alt="Finished" src="https://img.shields.io/badge/-Finished-brightgreen"> |
| 3 | Use of physical actuators and sensors | <img alt="Finished" src="https://img.shields.io/badge/-Finished-brightgreen"> |
| 4 | Sensor data fusion | <img alt="Finished" src="https://img.shields.io/badge/-Em%20andamento-orange"> |
| 5 | Mapping the environment through exteroceptive sensors | <img alt="Finished" src="https://img.shields.io/badge/-Em%20andamento-orange"> |
| 5 | Platform Localization Using Map Matching Algorithms | <img alt="Finished" src="https://img.shields.io/badge/-Em%20andamento-orange"> |
| 6 | Carry out the comparative analysis between the results of different algorithms | <img alt="Finished" src="https://img.shields.io/badge/-Not%20Started-red"> |

## Sensors Used

- [Engine Encoders Óptico](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/Engines)
- [SensorTag 2 CC2650 (IMU)](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/IMU-SensorTag_pkg)
- [Neato XV-11 LiDAR](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/Lidar/xv_11_laser_driver)
- [Camera RGB-D - Realsense D435](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/RealSense)

## Other materials

- Jetson Nano 4 GB
- Arduino Mega
- Chassi do Robô
- Conversor Buck-boost 
- Bateria de 12V

## Requeriments

- [Python 3.10.x](https://www.python.org/)
 
- [NumPy](https://numpy.org/)
 
- [SciPy](https://scipy.org/)
 
- [Matplotlib](https://matplotlib.org/)
 
- [Bluepy](https://pypi.org/project/bluepy/)

- [Ahrs](https://pypi.org/project/AHRS/) 

- [Ros](http://wiki.ros.org/noetic)

## Documentation

This Readme.md contains only main and general topics of the project development, each section has secondary Readme's that illustrate the operation and how to use each device and tool that make up the project.

### Devices

Contains directories associated with each sensor and actuator that was applied to the project. Possessing files for environment configuration, examples and tests.

### Tools

Contains secondary packages used in parallel to those developed for each sensor. Either for better visualization and to assist in calculations made based on the data offered by the sensors.

## How to use

1. Clone this repository

   > git clone https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location.git


2. Install the necessary libraries

   Use the standard python installation:

   > pip install (library name)

   > pip3 install (library name)
 
   Or use the standard Linux system installation:

   > sudo apt-get install (library or module name)

Note: For better explanations about the execution of specific parts of the project, or problems in the installation, contact us through our networks.

## Team

| [<img src="https://avatars.githubusercontent.com/u/50165797?v=4" width=115><br><sub>Breno Meneses</sub>](https://github.com/brenopmeneses) |  [<img src="https://avatars.githubusercontent.com/u/80260754?v=4" width=115><br><sub>Gabriel Henrique</sub>](https://github.com/gabrielhvs) |  [<img src="https://avatars.githubusercontent.com/u/60625985?v=4" width=115><br><sub>Marina Batista</sub>](https://github.com/maarinaabatista) |
| :---: | :---: | :---: |
