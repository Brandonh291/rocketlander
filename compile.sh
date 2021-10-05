#!/bin/bash

#g++ -lm -o rocketlandtest RocketLandInProgress.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp 


#g++ -lm -o gimbalTest GimbalTest.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp PWM.cpp RCOutput_Navio2.cpp

g++ -lm -o gimbalTestNoServo gimbal_in_progress.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp PWM.cpp RCOutput_Navio2.cpp

#g++ -lm -o FullTest rocketlanderUpdated.cpp MS5611.cpp Util.cpp I2Cdev.cpp MPU9250.cpp LSM9DS1.cpp PWM.cpp RCOutput_Navio2.cpp
