# pySatAttSim
Satellite Attitude Control Simulator using python

# How to start (Install by pipenv)
1. pinpenv install
1. python main.py

(PyOpenGL, pygame, scipy, pandas, matplotlib needs to be installed)

# Change parameter
- Simulation executes and visualize 3 axis attitude control
- parameters needs to be set at main.py

# Overview
## Class
- Satellite
  - Integrate Sensors, Actuateors, and Logics 
- Sensor
  - Get status from dynamics and tell it to logics
- Actuator
  - Get direction from logics and input output to dynamics
- Logic
  - Get information from sensors and calculate direction to logics
- Status
  - Manage parameters and their hisotry
- Timer
  - manage simulation time
- Dyanamics
  -  manage dynamics of satellite
