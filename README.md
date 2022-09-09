<h1 align="center"><a>Fusão Sensorial em um Robô Móvel<a></h1>

<p align="center">
   <img alt="GitHub build" src="http://img.shields.io/static/v1?label=STATUS&message=EM%20DESENVOLVIMENTO&color=GREEN&style=for-the-badge"></a>
</p>

<p align="center">
  <span>Português</span> |
  <a href="README_EN.md">English</a>
</p>

## Descrição
De forma geral, o projeto visa a aplicação dos conhecimentos adquiridos dentro do Laboratório eRobótica do Departamento de Engenharia Elétrica da Universidade Federal de Campina Grande a partir de aulas ministradas pelos professores doutores Antonio Marcus Nogueira Lima, Marcos Ricardo Alcântara Morais e Gutemberg Gonçalves dos Santos Júnior. Por meio da criação de uma plataforma robótica de tração diferencial com diferentes sensores, estudar e por fim solucionar o problema de localização. Realizado por meio da leitura e fusão de dados advindos dos sensores, tanto proprioceptivos quanto exteroceptivos, por exemplo, encoders ópticos, LiDAR, IMU, Câmera RGB-D. Esta fusão foi a priori, por meio do Filtro de Kalman Estendido(EKF - Extend Kalman Filter). As simulações e estudos iniciais foram realizadas na plataforma de simulação Gazebo, utilizando o software ROS para realizar a comunicação. Utilização e comparação de diferentes métodos de mapeamento e localização.

## Planejamento

| #        | Etapas         | _Status_ |
|:--|:----------------------------|:------------------|
| 1 | Implementação do modelo no Gazebo     | <img alt="Finalizado" src="https://img.shields.io/badge/-Finalizado-brightgreen">| 
| 2 | Inclusão dos sensores no modelo       | <img alt="Finalizado" src="https://img.shields.io/badge/-Finalizado-brightgreen"> |  
| 3 | Uso dos atuadores e  sensores fisícos     | <img alt="Finalizado" src="https://img.shields.io/badge/-Finalizado-brightgreen"> |  
| 4 | Fusão dos dados dos sensores          | <img alt="Finalizado" src="https://img.shields.io/badge/-Em%20andamento-orange"> | 
| 5 | Mapeamento do ambiente por meio dos sensores exteroceptivos | <img alt="Finalizado" src="https://img.shields.io/badge/-Em%20andamento-orange"> | 
| 5 | Localização da plataforma por meio de algoritmos de correspondencia de mapa  | <img alt="Finalizado" src="https://img.shields.io/badge/-Em%20andamento-orange"> | 
| 6 | Realizar a análise comparativa entre os resultados de diferentes algoritmos         | <img alt="Finalizado" src="https://img.shields.io/badge/-Não%20iniciado-red"> | 
              

## Sensores Utilizados
- [Motor Encoders Óptico](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/Engines)
- [SensorTag 2 CC2650 (IMU)](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/IMU-SensorTag_pkg)
- [Neato XV-11 LiDAR](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/Lidar/xv_11_laser_driver)
- [Câmera RGB-D - Realsense D435](https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location/tree/master/Devices/RealSense)

## Outros Materiais
- Jetson Nano 4 GB
- Arduino Mega
- Chassi do Robô
- Conversor Buck-boost 
- Bateria de 12V

## Requerimentos

- [Python 3.10.x](https://www.python.org/)
 
- [NumPy](https://numpy.org/)
 
- [SciPy](https://scipy.org/)
 
- [Matplotlib](https://matplotlib.org/)
 
- [Bluepy](https://pypi.org/project/bluepy/)

- [Ahrs](https://pypi.org/project/AHRS/) 

- [Ros](http://wiki.ros.org/noetic)

## Documentação

Este Readme.md contém apenas tópicos principais e gerais do desenvolvimento do projeto, cada seção possui Readme’s secundários que ilustram o funcionamento e como utilizar cada dispositivo e ferramenta que compõem o projeto.

### Devices

Contém diretórios associados a cada sensor e atuador que foi aplicado no projeto. Possuindo arquivos para configuração do ambiente, exemplos e testes. 

### Ferramentas

Contém pacotes secundários usados em paralelo aos desenvolvidos para cada sensor. Seja para melhor visualização e auxiliar em cálculos feitos tomando como base os dados oferecidos pelos sensores.

## Como utilizar

1. Faça o clone deste repositório

   > git clone https://github.com/gabrielhvs/sensory-fusion-in-mobile-robot-location.git


2. Instale as bibliotecas necessárias

   Use a instalação padrão python:

   > pip install (nome da biblioteca)

   > pip3 install (nome da biblioteca)
 
   Ou use a instalação padrão do sistema LInux:

   > sudo apt-get install (nome da biblioteca ou módulo)

Obs: Para melhores explicações sobre execuções de partes específicas do projeto, ou problemas na instalação entre em contato, por meio de nossas redes. 

## Equipe

| [<img src="https://avatars.githubusercontent.com/u/50165797?v=4" width=115><br><sub>Breno Meneses</sub>](https://github.com/brenopmeneses) |  [<img src="https://avatars.githubusercontent.com/u/80260754?v=4" width=115><br><sub>Gabriel Henrique</sub>](https://github.com/gabrielhvs) |  [<img src="https://avatars.githubusercontent.com/u/60625985?v=4" width=115><br><sub>Marina Batista</sub>](https://github.com/maarinaabatista) |
| :---: | :---: | :---: |
