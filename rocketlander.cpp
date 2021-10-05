#include "MS5611.h"
#include "Util.h"
#include "MPU9250.h"
#include "LSM9DS1.h"
#include "Util.h"
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include "InertialSensor.h"
#include "PWM.h"
#include "RCOutput_Navio2.h"
#include <chrono>
#include <ctime>
#include <ratio>

double temp;
double press;
double alt;
double accel;
float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float mx;
float my;
float mz;
int mode;
double pulselen;
double pulselen1;
double pulselen2;
double curTime;
double num = 0;

double ASCENT_THRESHOLD = 8.5; //m/s^2
double LAND_THRESHOLD = 11;
double DESCENT_THRESHOLD = 11;
double RELEASE_HEIGHT = 19; //meters
double LAND_HEIGHT = 0.5; //meters
double IGNITION_HEIGHT = 8.829;
double IGNITION_TIME = 1.45;

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


#define mtx_type double
#define PWM_OUTPUT1 1
#define PWM_OUTPUT2 2
#define ignitorPin 4
#define SERVOMIN  1250
#define SERVOMAX  1750
#define SERVO_ZEROX 1500
#define SERVO_ZEROZ 1500

double servo_limit = 0.0873; 

double dt_1;
double ignitionTimer;
double dt=0.022;

mtx_type gyro[3][1];
//gyro bias initialization
mtx_type gbias[3]={0.0,0.0,0.0};

mtx_type q[4]={1.0,0.0,0.0,0.0};
mtx_type qdot[4];
mtx_type k1q[4];
mtx_type k2q[4];
mtx_type k3q[4];
mtx_type k4q[4];

//moment of inertia
bool gimbal = false;
bool first_dt = true;

//gimbal angles in rad
double phi=0.0;
double theta=0.0;
int biasData = 0;
double seaLevelhPa = 1027.09;
double height;
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
    mode = MODE_BIAS;
    // Sensor Initilization
    MS5611 barometer;
    barometer.initialize();
    barometerReading(barometer);
    double baseAlt = alt;
    printf("Initialized Barometer \n");

    auto IMU_sensor = std::unique_ptr <InertialSensor>{ new MPU9250() };
    IMU_sensor->initialize();
    printf("Initilized IMU \n");

    auto pwm = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
    pwm->initialize(PWM_OUTPUT1);
    pwm->initialize(PWM_OUTPUT2);
    printf("Initialized Servo 1 \n");
    //printf("Initialized Servo 2 \n");
    pwm->set_frequency(PWM_OUTPUT1, 50);
    pwm->set_frequency(PWM_OUTPUT2, 50);

    pwm->initialize(ignitorPin);
    pwm->set_frequency(ignitorPin, 50);
    printf("set up clocks \n");
    // Defining clock variables - see if I can remove now() and put up top as a definition
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto elapsed = (end-start);
    auto start_ignition_timer = std::chrono::system_clock::now();
    auto ignition_timer_current = std::chrono::system_clock::now();
    auto ignition_timer = (ignition_timer_current-start_ignition_timer);
    auto initialTime = std::chrono::system_clock::now();
    while(true){ 
        if(gimbal == true){
            start = std::chrono::system_clock::now();
        }
        //collect new data
        printf("collect data \n");
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
            saveData();
        }

        switch (mode){
            case MODE_BIAS:
                //printf("Calculating Gyroscope Bias \n");
                gbias[0]+=gx;
                gbias[1]+=gy;
                gbias[2]+=gz;
                printf("gyro: x %f y %f z %f bias: x %f y %f z %f \n", gx, gy, gz, gbias[0], gbias[1], gbias[2]);
                //usleep(10000);
                biasData = biasData+1;

                if(biasData == 1000){
                    gbias[0]=gbias[0]/1000.;
                    gbias[1]=gbias[1]/1000.;
                    gbias[2]=gbias[2]/1000.;
                    mode = MODE_INIT;
                }
                break;
            
            case MODE_INIT:
            // Servo Commands for initilization
                printf("Running Servo Test \n");
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
                /*waiting to lift off, if off pad, change mode*/
                if(accel < ASCENT_THRESHOLD) mode=MODE_ASCENT;
                break;

            case MODE_ASCENT:
                /*if ascent stops AND barometer is at expected value, then mode change*/
                if(accel < ASCENT_THRESHOLD && alt >= RELEASE_HEIGHT) mode=MODE_ALTITUDE;
                break;

            case MODE_ALTITUDE:
                /*waiting for release*/
                if(accel > DESCENT_THRESHOLD){
                    start_ignition_timer = std::chrono::system_clock::now();
                    mode=MODE_RELEASE;
                }

                break;

            case MODE_RELEASE:
                /*freefalling, start ignition timer*/
                ignition_timer_current = std::chrono::system_clock::now();
                ignition_timer =  ignition_timer_current - start_ignition_timer;
                ignitionTimer = (double)ignition_timer.count();
                if(ignitionTimer > IGNITION_TIME && alt < IGNITION_HEIGHT) mode=MODE_IGNITE;
                break;

            case MODE_IGNITE:
                //ignite the motor
                pwm->set_duty_cycle(ignitorPin, 10000); //look up high value
                usleep(1); //change to 350 miliseconds
                pwm->set_duty_cycle(ignitorPin, 0);
                //printf("Begin Gimbal Control \n");
                mode=MODE_DECEL;
                break;

            case MODE_DECEL:
                
                gimbal = true;
                
                Integrator();
                Controller();
                //servo commands
                pwm->set_duty_cycle(PWM_OUTPUT1, pulselen1);
                pwm->set_duty_cycle(PWM_OUTPUT2, pulselen2);
                // set to zero initally then set it to something huge
                end = std::chrono::system_clock::now();
                elapsed = (end-start);
                dt_1 = (double)elapsed.count();
                dt = 0.000000001*dt_1;
                if(first_dt == true){
                    dt = 0.022;
                    first_dt = false;
                }
                printf("time(seconds): %f \n", dt);
                if(accel < LAND_THRESHOLD && height < LAND_HEIGHT) mode=MODE_LAND;
                break;

            case MODE_LAND:
                /*on the ground, upright?*/
                pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_ZEROX);
                pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_ZEROZ);
                break;

            case MODE_SAFE:
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
        
    barometer.refreshPressure();
    usleep(10000); // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    temp = barometer.getTemperature();
    press =  barometer.getPressure(); //millibars

    alt = 44330 * (1.0 - pow(press / seaLevelhPa, 0.1903));

}

void Controller(){
    //controller gains
    double Kp=0.4; //was 0.01 0.01
    double Kd=0.1;
    //controller torques
    double tau_x,tau_z;
    //controller forces
    double f_x, f_z, f_real = 1.0; //can change to force of thrust
    double x;

    double lever_gimbal=0.10; // update length
    
    // grab force at time in thrust
    // f_real = thrust[]
    // run PD controller to get desired torque
    tau_x = -Kp*q[1] - Kd*gx; //ideal torque about x body axis
    tau_z = -Kp*q[3] - Kd*gz; //ideal torque about z body axis

    f_x = tau_z /lever_gimbal; 
    f_z = -tau_x / lever_gimbal;

    // get real servo angle phi (around x axis)
    x = -f_x/ f_real;
    if (abs(x) > sin(servo_limit)){
        phi = sign(-f_x) * servo_limit;
    }else{
        phi = asin(x); //asin return -pi/2 ~ pi/2
    }


    // get real servo angle theta (around z axis)
    x = f_z/(f_real*cos(phi));
    if (abs(x) > sin(servo_limit)){
        theta = sign(f_z) * servo_limit;
    }else{
        theta = asin(x); //asin return -pi/2 ~ pi/2
    } 
    
    
    double th = theta*180/3.14;
    double ph = phi*180/3.14;
    //printf("Servo Angles Theta: %f phi: %f \n", th, ph);
    //set servo positions
    double angle1 = ph*250/5; 
    double angle2 = th*240/5;
    pulselen1 = (angle1) + SERVO_ZEROX;
    pulselen2 = (angle2) + SERVO_ZEROZ;
    
}

double sign(double x){
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

    //qdot= |E|*gyro
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
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			A[n * i + j] = A[n * i + j] * k;
}
void Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
{
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
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void saveDataFirst(){
    std::ofstream myfile;
    myfile.open("data_storage.csv", std::ios::app);
    myfile << "Current Time" << "," << "Quaternion Scalar" << "," << "Quaternion x componenet" << "," << "Quaternion y componenet"
           << "," << "Quaternion z componenet" << "," << "gyroscope x-axis rate" << "," << "gyroscope y-axis rate" << "," 
           << "gyroscope z-axis rate" << "," << "accelertometer in x-axis" << "," << "accelertometer in y-axis"
           << "," << "accelertometer in z-axis" << "," << "altitude" << "," << "pressure" << "\n";
}
void saveData(){
    std::ofstream myfile;
    /*myfile.open("data_storage2.txt", std::ios::app);
    myfile <<  gx << " " << gy << " " << gz << " " <<
              q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " <<
              E[0] << " " << E[1] << " " << E[2] << "\n";

              */
            /*attitude, angles and mode, fixed fields, gimbal angles, 
             rocket attitude, rocket rates, accelerometers, mode that its
             in, each line should be proceeded by the clock time.  At the 
             start, have it print the names with commas so that it is readable, 
             want all that data at once. Print out available storage left. 
             somewhere you need altitude and pressure. Anythign I can think 
             of that Im collecting, throw it in here.*/

             // change landing to account for not straight up.

             // st a timer for after release, can go back to low rate after 10 seconds
             // also landing should be less than 10 seconds after

    myfile.open("data_storage.csv", std::ios::app);
    myfile << curTime << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << gx << "," << gy << "," << gz << ","
           << ax << "," << ay << "," << az << "," <<  alt << "," << press << "\n";            
    myfile.close();
}
