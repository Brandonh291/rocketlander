#include "MS5611.h"
#include "Util.h"
#include "MPU9250.h"
#include "LSM9DS1.h"
#include "InertialSensor.h"
#include "PWM.h"
#include "RCOutput_Navio2.h"
#include "Util.h"
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ratio>

//Change to day of Launch Conditions
double seaLevelhPa = 1029.8008;

//data variables
double temp, press, alt, accel, height;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float gxunfil, gyunfil, gzunfil;



//defining thresholds and variables for modes
double ASCENT_THRESHOLD = 12; //m/s^2
double LAND_THRESHOLD = 20; //m/s^2
double DESCENT_THRESHOLD = 9;//m/s^2
double RELEASE_HEIGHT = 10; //19; //meters
double LAND_HEIGHT = 0.5; //meters
double IGNITION_HEIGHT = 8.829; //meters
double IGNITION_TIME = 1.45; //seconds

// defining modes
int mode;
#define MODE_INIT       0
#define MODE_IDLE       1
#define MODE_ASCENT     2
#define MODE_ALTITUDE   3
#define MODE_RELEASE    4
#define MODE_IGNITE     5
#define MODE_DECEL      6
#define MODE_LAND       7
#define MODE_SAFE       8

// variables and pins for servos
#define mtx_type double
#define PWM_OUTPUT1 1
#define PWM_OUTPUT2 2
#define ignitorPin 4
#define ledPin 6
#define SERVOMIN  1250
#define SERVOMAX  1750
#define SERVO_ZEROX 1500
#define SERVO_ZEROZ 1500
double servo_limit = 0.0873; 
double pulselen;
double pulselen1 = SERVO_ZEROX;
double pulselen2 = SERVO_ZEROZ;
double oldPulse1, oldPulse2, pulseChange1, pulseChange2;
double pulse1, pulse2;
double phi=0.0;
double theta=0.0;

//time variables
double curTime;
double ignitionTimer;
double dt=0.05;
double data = 0;
double control = 10;
double dt_test = 0;
int num = 0;

double accelval;
int blink = 0;
//quaternion integrator variables
mtx_type gyro[3][1];
mtx_type gbias[3]={0.0,0.0,0.0};
mtx_type q[4]={1.0,0.0,0.0,0.0};
mtx_type qdot[4];
mtx_type k1q[4];
mtx_type k2q[4];
mtx_type k3q[4];
mtx_type k4q[4];

// flags for finding time of each loop
bool gimbal = false;
bool first_dt = true;
bool notLanded = true;
bool startup = true;
bool off = true;
//gimbal angles in rad
int biasData = 0;

//declaration of all functions
void barometerReading(MS5611);
void Controller();
void Integrator();
double sign(double);
void Scale(mtx_type*, int, int, mtx_type);
void Multiply(mtx_type*, mtx_type*, int, int, int, mtx_type*);
void Copy(mtx_type*, int, int, mtx_type*);
void IgniteMotor();
void saveDataFirst();
void saveData();
void filter();

int main(){
    //set initial mode
    mode = MODE_INIT;
    saveDataFirst();
    //Set up barometer
    MS5611 barometer;
    barometer.initialize();
    barometerReading(barometer);
    double baseAlt = alt;

    //Set up IMU
    auto IMU_sensor = std::unique_ptr <InertialSensor>{ new MPU9250() };
    IMU_sensor->initialize();
    //printf("Initilized IMU \n");

    //Set up servos
    auto pwm = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
    pwm->initialize(PWM_OUTPUT1);
    pwm->initialize(PWM_OUTPUT2);
    //printf("Initialized Servo 1 \n");
    //printf("Initialized Servo 2 \n");
    pwm->set_frequency(PWM_OUTPUT1, 50);
    pwm->set_frequency(PWM_OUTPUT2, 50);

    //Set up ignitor pin on servo rail
    pwm->initialize(ignitorPin);
    pwm->set_frequency(ignitorPin, 50);

    //set up led pin
    
    pwm->initialize(ledPin);

    pwm->set_frequency(ledPin, 25);

    //should go on
    pwm->set_duty_cycle(ledPin, 0); //led on
    
    //pwm->set_duty_cycle(ledPin, 40000); //turns led off
    
    // Defining clock variables - see if I can remove now() and put up top as a definition
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto elapsed = (end-start);
    auto start_ignition_timer = std::chrono::system_clock::now();
    auto ignition_timer_current = std::chrono::system_clock::now();
    auto ignition_timer = (ignition_timer_current-start_ignition_timer);
    auto initialTime = std::chrono::system_clock::now();

    
    for(int i=0; i<=1000; i++){      //adds up first 1000 data points and finds the average value to find the gyroscope bias
        IMU_sensor->update();
        IMU_sensor->read_gyroscope(&gx, &gy, &gz);
        gbias[0]+=gx;
        gbias[1]+=gy;
        gbias[2]+=gz;
        if(i==100 || i==300 || i==500 || i==700 || i==900){
            pwm->set_duty_cycle(ledPin, 40000); //turns led off
        }
        if(i==200 || i==400 || i==600 || i==800 || i==1000){
            pwm->set_duty_cycle(ledPin, 0); //turns led on
        }
        //printf("bias: x: %f y: %f z: %f \n", gbias[0], gbias[1], gbias[2]);
        
    }
        
    gbias[0]=gbias[0]/1000.;
    gbias[1]=gbias[1]/1000.;
    gbias[2]=gbias[2]/1000.;
    
    while(notLanded){ 
        
        //tests to see if gimbal control has begun to start calculating the timesteps
        //verify that the acceleration is still primarily in the y direction, if not, safe mode.
        accelval=sqrt(ax*ax +az*az);
        if(accelval>8){
            mode = MODE_SAFE;
        }

        //printf("acceleration not in y %f \n", accelval);

        auto current_time = std::chrono::system_clock::now();
        auto totalElapsed = (current_time-initialTime);
        curTime = 0.000000001*(double)totalElapsed.count();
        //collect new data
        //printf("acceleration value %f \n", accel);
        while(dt_test < 0.05){
            end = std::chrono::system_clock::now();
            elapsed = (end-start);
            //change to seconds
            dt_test = 0.000000001*(double)elapsed.count();   
        }
        dt = dt_test;
        dt_test = 0;
        start = std::chrono::system_clock::now();
        
        barometerReading(barometer);
        height = alt-baseAlt;
        //printf("height %f \n", height);
        IMU_sensor->update();
        IMU_sensor->read_accelerometer(&ax, &ay, &az);
        IMU_sensor->read_gyroscope(&gx, &gy, &gz);

        accel = sqrt(ax*ax + ay*ay + az*az);
        if(data=20){
            startup=false;
        }
        gxunfil = gx - gbias[0];
        gyunfil = gy - gbias[1];
        gzunfil = gz - gbias[2];
        if(startup=false){
            filter();
        }
        data +=1;

        gx = gx - gbias[0];
        gy = gy - gbias[1];
        gz = gz - gbias[2];
        //printf("Rates: x: %f y: %f z: %f \n", gx, gy, gz);
        Integrator();
        saveData();
        

        switch (mode){            
            case MODE_INIT:
                //Servo Commands for initilization. Servos will move their full range in each direction to test that they are functioning
                //printf("Running Servo Test \n");
                for (uint16_t pulselen = SERVOMIN+10; pulselen < SERVOMAX-10; pulselen++) {
                    //printf("servo: %f \n", pulselen);
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulselen);
                    usleep(5000);
                }
                usleep(50000);
                for (uint16_t pulselen = SERVOMAX-10; pulselen > SERVOMIN+10; pulselen--) {
                    //printf("servo: %f \n", pulselen);
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulselen);
                    usleep(5000);
                }
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                usleep(50000);
                for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
                    //printf("servo: %f \n", pulselen);
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulselen);
                    usleep(5000);
                }
                usleep(500000);
                for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
                    //printf("servo: %f \n", pulselen);
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulselen);
                    usleep(5000);
                }
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                usleep(50000);
                
                mode = MODE_IDLE;
                
                break;

            case MODE_IDLE:
                if(blink < 10){
                    blink +=1;
                }
                else{
                    if(off){
                        pwm->set_duty_cycle(ledPin, 0); //turns led on
                        off=false;
                    }
                    else{
                        pwm->set_duty_cycle(ledPin, 40000); //turns led off
                        off = true;
                    }
                    blink = 0;
                }
                /*waiting to lift off, if off pad, change mode
                  Tests this by waiting for a change in acceleration*/
                if(accel > ASCENT_THRESHOLD) mode=MODE_ASCENT;
                break;

            case MODE_ASCENT:
                pwm->set_duty_cycle(ledPin, 0); //turns led on
                /*if barometer is at expected value, then mode change*/
                if(height >= RELEASE_HEIGHT) mode=MODE_ALTITUDE;
                break;

            case MODE_ALTITUDE:
                if(blink < 10){
                        blink +=1;
                }
                else{
                    if(off){
                        pwm->set_duty_cycle(ledPin, 0); //turns led on
                        off=false;
                    }
                    else{
                        pwm->set_duty_cycle(ledPin, 40000); //turns led off
                        off = true;
                    }
                    blink = 0;
                }
                /*waiting for release, changes mode when acceleration shows rocket has been dropped*/
                if(accel < DESCENT_THRESHOLD){ //need ability to sense pin
                    start_ignition_timer = std::chrono::system_clock::now();
                    mode=MODE_RELEASE;
                }

                break;

            case MODE_RELEASE:
                /*freefalling, start ignition timer, ignition occurs once the timer has reached the 
                needed delay time and the rocket has reached the maximum ignition height*/
                pwm->set_duty_cycle(ledPin, 0); //turns led on
                ignition_timer_current = std::chrono::system_clock::now();
                ignition_timer =  ignition_timer_current - start_ignition_timer;
                ignitionTimer = (double)ignition_timer.count();
                if(ignitionTimer > IGNITION_TIME) mode=MODE_IGNITE;
                break;

            case MODE_IGNITE:
                //ignite the motor by sending a high current through the ignition circuit
                pwm->set_duty_cycle(ignitorPin, 10000); //look up high value
                usleep(350000); //Leave high current for 350 miliseconds
                pwm->set_duty_cycle(ignitorPin, 0); //cut current off
                //printf("Begin Gimbal Control \n");
                mode=MODE_DECEL;
                break;

            case MODE_DECEL:
                //change boolean variable to true to indicate we reached gimbal control
                gimbal = true;
                if(blink < 10){
                        blink +=1;
                }
                else{
                    if(off){
                        pwm->set_duty_cycle(ledPin, 0); //turns led on
                        off=false;
                    }
                    else{
                        pwm->set_duty_cycle(ledPin, 40000); //turns led off
                        off = true;
                    }
                    blink = 0;
                }
                if(control < 10){
                    control +=1;
                    pulse1 = oldPulse1+control*pulseChange1;
                    pulse2 = oldPulse2+control*pulseChange2;
                    //printf("control: %f \n", control);
                    //printf("servo: 1: %f 2: %f \n", (pulse1-1500)/50, (pulse2-1500)/50);
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulse1);
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulse2);
                }
                else{
                    //run the controller to find the servo angles needed to correct vertical deviation and to null rates
                    oldPulse1 = pulselen1;
                    oldPulse2 = pulselen2;
                    Controller();
                    //servo commands
                    pulseChange1 = (pulselen1-oldPulse1)/10;
                    pulseChange2 = (pulselen2-oldPulse2)/10;
                    pulse1 = oldPulse1+pulseChange1;
                    pulse2 = oldPulse2+pulseChange2;
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulse1);
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulse2);
                    //printf("servo: 1: %f 2: %f \n", (pulse1-1500)/50, (pulse2-1500)/50);
                    //finding time elapsed for the loop
                    end = std::chrono::system_clock::now();
                    elapsed = (end-start);
                    //change to seconds
                    dt = 0.000000001*(double)elapsed.count();

                    //set the first loop to the average loop time as start time is defined up in the main loop once gimbal is true
                    if(first_dt == true){
                        dt = 0.05;
                        first_dt = false;
                    }
                    
                    control = 1;
                }

                if(accel > LAND_THRESHOLD) mode=MODE_LAND;
                break;

            case MODE_LAND:
                //set servos to zero positions to indicate end of control and to show landing has been sensed
                pwm->set_duty_cycle(ledPin, 0); //turns led on
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                notLanded = false;
                break;

            case MODE_SAFE:
                //set servos to zero positions in case of unsafe flight
                if(blink < 2){
                        blink +=1;
                }
                else{
                    if(off){
                        pwm->set_duty_cycle(ledPin, 0); //turns led on
                        off=false;
                    }
                    else{
                        pwm->set_duty_cycle(ledPin, 40000); //turns led off
                        off = true;
                    }
                    blink = 0;
                }
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                break;

            default:
                mode=MODE_SAFE;
                break;

        }
    }
}

void barometerReading(MS5611 barometer) {
    //finds pressure, temperature, and altitude as a function of pressure
    barometer.refreshPressure();
    usleep(10000); // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    temp = barometer.getTemperature(); //celcius
    press =  barometer.getPressure(); //millibars

    alt = 44330 * (1.0 - pow(press / seaLevelhPa, 0.1903)); //meters

}

void Controller(){
    //controller gains
    double Kp=0.4; //position gain
    double Kd=0.1; //rate gain

    //controller torques
    double tau_x,tau_z;
    //controller forces
    double f_x, f_z, f_real = 1.0; 
    double x;
    double lever_gimbal=0.10;

    // run PD controller to get desired torque
    tau_x = -Kp*q[1] - Kd*gx; //ideal torque about x body axis
    tau_z = -Kp*q[3] - Kd*gz; //ideal torque about z body axis

    f_x = tau_z /lever_gimbal; //force needed around x axis
    f_z = -tau_x / lever_gimbal; //force needed around z axis

    // get real servo angle phi (around x axis)
    x = -f_x/ f_real;
    if (abs(x) > sin(servo_limit)){
        phi = sign(-f_x) * servo_limit;
    }else{
        phi = asin(x); 
    }


    // get real servo angle theta (around z axis)
    x = f_z/(f_real*cos(phi));
    if (abs(x) > sin(servo_limit)){
        theta = sign(f_z) * servo_limit;
    }else{
        theta = asin(x);
    } 
    
    //convert angles from radians to degrees
    double th = theta*180/3.14;
    double ph = phi*180/3.14;
    //printf("Servo Angles Theta: %f phi: %f \n", th, ph);
    //set servo positions based on servo limits
    double angle1 = ph*250/5; 
    double angle2 = th*240/5;
    pulselen1 = (angle1) + SERVO_ZEROX;
    pulselen2 = (angle2) + SERVO_ZEROZ;
    
}

double sign(double x){
    //returns the sign of the variable
    if (x>0)
        return 1.0;
    else
        return -1.0;
}

void Integrator(){

    //init 4x3 element matrix
    mtx_type E[4][3];

    //quaternions are represented as q0 being the scalar component
    //quaternion rate of change: qdot = 0.5 *
    //     [-q1  -q2  -q3
    //       q0  -q3   q2
    //       q3   q0  -q1
    //      -q2   q1   q0] * omega ==> gyros

    E[0][0] = -q[1];
    E[0][1] = -q[2];
    E[0][2] = -q[3];
    E[1][0] = q[0];
    E[1][1] = -q[3];
    E[1][2] = q[2];
    E[2][0] = q[3];
    E[2][1] = q[0];
    E[2][2] = -q[1];
    E[3][0] = -q[2];
    E[3][1] = q[1];
    E[3][2] = q[0];
    gyro[0][0] = gx;
    gyro[1][0] = gy;
    gyro[2][0] = gz;

    //qdot= 0.5*E*gyro
    Multiply((mtx_type*)E,(mtx_type*)gyro,4,3,1,(mtx_type*)qdot);
    Scale((mtx_type*)qdot,4,1,0.5); 
    //set up runge kutta integrator for qdot
    //k1q=dt*qdot
    Scale((mtx_type*)qdot,4,1,dt); 
    Copy((mtx_type*)qdot,4,1,k1q);
            
    E[0][0] = -(q[1]+ 0.5*k1q[1]);
    E[0][1] = -(q[2]+ 0.5*k1q[2]);
    E[0][2] = -(q[3]+ 0.5*k1q[3]);
    E[1][0] =  (q[0]+ 0.5*k1q[0]);
    E[1][1] = -(q[3]+ 0.5*k1q[3]);
    E[1][2] =  (q[2]+ 0.5*k1q[2]);
    E[2][0] =  (q[3]+ 0.5*k1q[3]);
    E[2][1] =  (q[0]+ 0.5*k1q[0]);
    E[2][2] = -(q[1]+ 0.5*k1q[1]);
    E[3][0] = -(q[2]+ 0.5*k1q[2]);
    E[3][1] =  (q[1]+ 0.5*k1q[1]);
    E[3][2] =  (q[0]+ 0.5*k1q[0]);

    //k2q = dt*(qdot+.5*k1q)         
    Multiply((mtx_type*)E,(mtx_type*)gyro,4,3,1,(mtx_type*)qdot);
    Scale((mtx_type*)qdot,4,1,0.5);
    Scale((mtx_type*)qdot,4,1,dt);
    Copy((mtx_type*)qdot,4,1,k2q);
    
    E[0][0] = -(q[1]+ 0.5*k2q[1]);
    E[0][1] = -(q[2]+ 0.5*k2q[2]);
    E[0][2] = -(q[3]+ 0.5*k2q[3]);
    E[1][0] =  (q[0]+ 0.5*k2q[0]);
    E[1][1] = -(q[3]+ 0.5*k2q[3]);
    E[1][2] =  (q[2]+ 0.5*k2q[2]);
    E[2][0] =  (q[3]+ 0.5*k2q[3]);
    E[2][1] =  (q[0]+ 0.5*k2q[0]);
    E[2][2] = -(q[1]+ 0.5*k2q[1]);
    E[3][0] = -(q[2]+ 0.5*k2q[2]);
    E[3][1] =  (q[1]+ 0.5*k2q[1]);
    E[3][2] =  (q[0]+ 0.5*k2q[0]);

    //k3q = dt*(qdot+.5*k2q)            
    Multiply((mtx_type*)E,(mtx_type*)gyro,4,3,1,(mtx_type*)qdot);
    Scale((mtx_type*)qdot,4,1,0.5);
    Scale((mtx_type*)qdot,4,1,dt);
    Copy((mtx_type*)qdot,4,1,k3q);

    E[0][0] = -(q[1]+ k3q[1]);
    E[0][1] = -(q[2]+ k3q[2]);
    E[0][2] = -(q[3]+ k3q[3]);
    E[1][0] =  (q[0]+ k3q[0]);
    E[1][1] = -(q[3]+ k3q[3]);
    E[1][2] =  (q[2]+ k3q[2]);
    E[2][0] =  (q[3]+ k3q[3]);
    E[2][1] =  (q[0]+ k3q[0]);
    E[2][2] = -(q[1]+ k3q[1]);
    E[3][0] = -(q[2]+ k3q[2]);
    E[3][1] =  (q[1]+ k3q[1]);
    E[3][2] =  (q[0]+ k3q[0]);
  
    //k4q = dt*(qdot+k3q)         
    Multiply((mtx_type*)E,(mtx_type*)gyro,4,3,1,(mtx_type*)qdot);
    Scale((mtx_type*)qdot,4,1,0.5);
    Scale((mtx_type*)qdot,4,1,dt);
    Copy((mtx_type*)qdot,4,1, k4q);

    //integrate qdot to get q            
    q[0] = q[0] + (1.0/6.0) * ( k1q[0] + 2.0*k2q[0] + 2.0*k3q[0] + k4q[0] );
    q[1] = q[1] + (1.0/6.0) * ( k1q[1] + 2.0*k2q[1] + 2.0*k3q[1] + k4q[1] );
    q[2] = q[2] + (1.0/6.0) * ( k1q[2] + 2.0*k2q[2] + 2.0*k3q[2] + k4q[2] );
    q[3] = q[3] + (1.0/6.0) * ( k1q[3] + 2.0*k2q[3] + 2.0*k3q[3] + k4q[3] );

    double q_norm = sqrt((q[0]*q[0]) + (q[1]*q[1]) + (q[2]*q[2]) + (q[3]*q[3]));
    q[0] = q[0]/q_norm;
    q[1] = q[1]/q_norm;
    q[2] = q[2]/q_norm;
    q[3] = q[3]/q_norm;
}


void Scale(mtx_type* A, int m, int n, mtx_type k)
{
    //multiply a matrix by a value
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			A[n * i + j] = A[n * i + j] * k;
}
void Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
{
    //multiply two matrices
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			C[n * i + j] = 0;
			for (k = 0; k < p; k++)
				C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
		}
}

void Copy(mtx_type* A, int n, int m, mtx_type* B)
{
    //copy a matrix to another variable
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void saveDataFirst(){
    //set titles for data file
    std::ofstream myfile;
    printf("save first \n");
    myfile.open("data_storageV1.csv", std::ios::app);
    myfile << "Current Time" << "," << "Quaternion Scalar" << "," << "Quaternion x componenet" << "," << "Quaternion y componenet"
           << "," << "Quaternion z componenet" << "," << "gyroscope x-axis rate" << "," << "gyroscope y-axis rate" << "," 
           << "gyroscope z-axis rate" << "," << "accelertometer in x-axis" << "," << "accelertometer in y-axis"
           << "," << "accelertometer in z-axis" << "," << "Magnetometer in x-axis" << "," << "Magnetometer in y-axis" << "," << "Magnetometer in z-axis"
           << "Unfiltered gyroscope in x-axis" << "," << "Unfiltered gyroscope in y-axis" << "," << "Unfiltered gyroscope in z-axis" << "," 
           << "altitude" << "," << "height" << "," << "pressure" << "," << "acceleration"  << "," << "mode" << "\n";
}
void saveData(){
    //save data
    std::ofstream myfile;
    myfile.open("data_storageV1.csv", std::ios::app);
    myfile << curTime << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << gx << "," << gy << "," << gz << ","
           << ax << "," << ay << "," << az << "," << mx << "," << my << "," << mz << "," << gxunfil << "," << gyunfil << "," 
           << gzunfil << "," <<  alt << height << "," << "," << press << "," << accel << "," << mode << "\n";            
    myfile.close();
 }

void filter(){
    double gx_avg;
    double gy_avg;
    double gz_avg;
    double gx20sum;
    double gy20sum;
    double gz20sum;
    double gx20[20];
    double gy20[20];
    double gz20[20];
    double kn = 0.8;
    double ko = 1-kn;

    gx20[num]=gx;
    gy20[num]=gy;
    gz20[num]=gz;
    for(int i = 0; i<20 ; i++){
        gx20sum+=gx20[i];
        gy20sum+=gy20[i];
        gz20sum+=gz20[i];
    }

    gx_avg = gx20sum/20;
    gy_avg = gx20sum/20;
    gz_avg = gx20sum/20;
    gx = kn*gx + ko*gx_avg;
    gy = kn*gy + ko*gy_avg;
    gz = kn*gz + ko*gz_avg;

    if(num<19){
        num = num+1;
    }
    else{
        num = 0;
    }
}

