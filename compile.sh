#!/bin/bash

#g++ -lm -o FullTest rocketlander.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp PWM.cpp RCOutput_Navio2.cpp
g++ -lm -o modeTest gimbalModeTest.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp PWM.cpp RCOutput_Navio2.cpp
