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
double seaLevelhPa = 1027.09;

//data variables
double temp, press, alt, accel, height;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

//defining thresholds and variables for modes
double ASCENT_THRESHOLD = 8.5; //m/s^2
double LAND_THRESHOLD = 11; //m/s^2
double DESCENT_THRESHOLD = 11;//m/s^2
double RELEASE_HEIGHT = 19; //meters
double LAND_HEIGHT = 0.5; //meters
double IGNITION_HEIGHT = 8.829; //meters
double IGNITION_TIME = 1.45; //seconds

// defining modes
int mode;
#define MODE_BIAS       0
#define MODE_INIT       1
#define MODE_IDLE       2
#define MODE_ASCENT     3
#define MODE_ALTITUDE   4
#define MODE_RELEASE    5
#define MODE_IGNITE     6
#define MODE_DECEL      7
#define MODE_LAND       8
#define MODE_SAFE       9

// variables and pins for servos
#define mtx_type double
#define PWM_OUTPUT1 1
#define PWM_OUTPUT2 2
#define ignitorPin 4
#define SERVOMIN  1250
#define SERVOMAX  1750
#define SERVO_ZEROX 1500
#define SERVO_ZEROZ 1500
double servo_limit = 0.0873; 
double pulselen, pulselen1, pulselen2;
double phi=0.0;
double theta=0.0;

//time variables
double curTime;
double ignitionTimer;
double dt=0.022;

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

int main(){
    //set initial mode
    mode = MODE_BIAS;

    //Set up barometer
    MS5611 barometer;
    barometer.initialize();
    barometerReading(barometer);
    double baseAlt = alt;
    //printf("Initialized Barometer \n");

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
    
    // Defining clock variables - see if I can remove now() and put up top as a definition
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto elapsed = (end-start);
    auto start_ignition_timer = std::chrono::system_clock::now();
    auto ignition_timer_current = std::chrono::system_clock::now();
    auto ignition_timer = (ignition_timer_current-start_ignition_timer);
    auto initialTime = std::chrono::system_clock::now();

    
    while(true){ 

        //tests to see if gimbal control has begun to start calculating the timesteps
        if(gimbal == true){
            start = std::chrono::system_clock::now();
        }
        //verify that the acceleration is still primarily in the y direction, if not, safe mode.
        if(sqrt(ax*ax +az*az)>4){
            mode = MODE_SAFE;
        }
        //collect new data
        //printf("collect data \n");
        barometerReading(barometer);
        height = alt-baseAlt;
        IMU_sensor->update();
        IMU_sensor->read_accelerometer(&ax, &ay, &az);
        IMU_sensor->read_gyroscope(&gx, &gy, &gz);
        IMU_sensor->read_magnetometer(&mx, &my, &mz);
        auto current_time = std::chrono::system_clock::now();

        auto totalElapsed = (current_time-initialTime);
        curTime = 0.000000001*(double)totalElapsed.count();
        accel = sqrt(ax*ax + ay*ay + az*az);
        if(mode != MODE_BIAS){
            gx = gx - gbias[0];
            gy = gy - gbias[1];
            gz = gz - gbias[2];
            Integrator();
            saveData();
        }

        switch (mode){
            case MODE_BIAS:
                //adds up first 1000 data points and finds the average value to find the gyroscope bias
                gbias[0]+=gx;
                gbias[1]+=gy;
                gbias[2]+=gz;
                biasData = biasData+1;

                if(biasData == 1000){
                    gbias[0]=gbias[0]/1000.;
                    gbias[1]=gbias[1]/1000.;
                    gbias[2]=gbias[2]/1000.;
                    mode = MODE_INIT;
                }
                break;
            
            case MODE_INIT:
                //Servo Commands for initilization. Servos will move their full range in each direction to test that they are functioning
                //printf("Running Servo Test \n");
                for (uint16_t pulselen = SERVOMIN+10; pulselen < SERVOMAX-10; pulselen++) {
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulselen);
                    usleep(5000);
                }
                usleep(50000);
                for (uint16_t pulselen = SERVOMAX-10; pulselen > SERVOMIN+10; pulselen--) {
                    pwm->set_duty_cycle(PWM_OUTPUT1, pulselen);
                    usleep(5000);
                }
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                usleep(50000);
                for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulselen);
                    usleep(5000);
                }
                usleep(500000);
                for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
                    pwm->set_duty_cycle(PWM_OUTPUT2, pulselen);
                    usleep(5000);
                }
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                usleep(50000);
                
                mode = MODE_IDLE;
                break;

            case MODE_IDLE:
                /*waiting to lift off, if off pad, change mode
                  Tests this by waiting for a change in acceleration*/
                if(accel < ASCENT_THRESHOLD) mode=MODE_ASCENT;
                break;

            case MODE_ASCENT:
                /*if barometer is at expected value, then mode change*/
                if(alt >= RELEASE_HEIGHT) mode=MODE_ALTITUDE;
                break;

            case MODE_ALTITUDE:
                /*waiting for release, changes mode when acceleration shows rocket has been dropped*/
                if(accel > DESCENT_THRESHOLD){
                    start_ignition_timer = std::chrono::system_clock::now();
                    mode=MODE_RELEASE;
                }

                break;

            case MODE_RELEASE:
                /*freefalling, start ignition timer, ignition occurs once the timer has reached the 
                needed delay time and the rocket has reached the maximum ignition height*/
                ignition_timer_current = std::chrono::system_clock::now();
                ignition_timer =  ignition_timer_current - start_ignition_timer;
                ignitionTimer = (double)ignition_timer.count();
                if(ignitionTimer > IGNITION_TIME && alt < IGNITION_HEIGHT) mode=MODE_IGNITE;
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
                
                //run the controller to find the servo angles needed to correct vertical deviation and to null rates
                Controller();
                //servo commands
                pwm->set_duty_cycle(PWM_OUTPUT1, pulselen1);
                pwm->set_duty_cycle(PWM_OUTPUT2, pulselen2);

                //finding time elapsed for the loop
                end = std::chrono::system_clock::now();
                elapsed = (end-start);
                //change to seconds
                dt = 0.000000001*(double)elapsed.count();

                //set the first loop to the average loop time as start time is defined up in the main loop once gimbal is true
                if(first_dt == true){
                    dt = 0.022;
                    first_dt = false;
                }
                //printf("time(seconds): %f \n", dt);

                if(accel < LAND_THRESHOLD && height < LAND_HEIGHT) mode=MODE_LAND;
                break;

            case MODE_LAND:
                //set servos to zero positions to indicate end of control and to show landing has been sensed
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                break;

            case MODE_SAFE:
                //set servos to zero positions in case of unsafe flight
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
    myfile.open("data_storage.csv", std::ios::app);
    myfile << "Current Time" << "," << "Quaternion Scalar" << "," << "Quaternion x componenet" << "," << "Quaternion y componenet"
           << "," << "Quaternion z componenet" << "," << "gyroscope x-axis rate" << "," << "gyroscope y-axis rate" << "," 
           << "gyroscope z-axis rate" << "," << "accelertometer in x-axis" << "," << "accelertometer in y-axis"
           << "," << "accelertometer in z-axis" << "," << "altitude" << "," << "pressure" << "\n";
}
void saveData(){
    //save data
    std::ofstream myfile;
    myfile.open("data_storage.csv", std::ios::app);
    myfile << curTime << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << gx << "," << gy << "," << gz << ","
           << ax << "," << ay << "," << az << "," <<  alt << "," << press << "\n";            
    myfile.close();
}
