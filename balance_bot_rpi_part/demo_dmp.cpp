#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <time.h>
#include <sys/time.h>
#include <system_error>
#include <iostream>
#include <fstream>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

//#define OUTPUT_DISPLACEMENT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
int gg[3];         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_[3];
float dis[3];
float acc_filt[3];
float acc_filt_old[3] = {0.0f, 0.0f, 0.0f};
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
unsigned long last_read_time;

inline unsigned long get_last_time() {return last_read_time;}

void set_last_read_time(unsigned long time) {
	last_read_time = time;
}

int GetTimeStamp(){
	struct timeval tv;
	unsigned long long curTime;
	gettimeofday(&tv, NULL);
	curTime = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec)/1000;
	return curTime; //the time intervals passed millisec
}


void setup() {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
	
    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
	set_last_read_time(GetTimeStamp());

}

void set_ypr(float *Ypr){
	
	ypr_[0] = Ypr[0]* 180/M_PI;
	ypr_[1] = Ypr[1]* 180/M_PI;
	ypr_[2] = Ypr[2]* 180/M_PI;
}

/*
float compFilter(float angularRate, float accAngle, float angle, float gain)
{
    angle   = gain*(angle+angularRate) + (1-gain) * accAngle;
    return angle;
}
*/

float get_yaw() {return ypr_[0];}
float get_pitch() {return ypr_[1];}
float get_roll() {return ypr_[2];}
	
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

float acc_x_ms, acc_y_ms;
float pos_x, pos_y;
float vel_x, vel_y;
   
float pos_x_old = 0.0f;
float pos_y_old = 0.0f;
   
float vel_x_old = 0.0f;
float vel_y_old = 0.0f;
   
float acc_x_ms_old = 0.0f;
float acc_y_ms_old = 0.0f;
float alpha = 0.7;

float angle = -85.00;

void loop() {

	
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //printf("ypr  %7.2f %7.2f %7.2f	\n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
			set_ypr(ypr);
			/*
			unsigned long t_now = GetTimeStamp();
			float dt =(t_now - get_last_time())/1000.0; // 0.05 sec 
			if (dt < 0){ throw std::runtime_error("Time went backwards."); }
			set_last_read_time(GetTimeStamp());
			*/
			
        #endif
		/*
		#ifdef OUTPUT_DISPLACEMENT
		
			unsigned long t_now = GetTimeStamp();
			float dt =(t_now - get_last_time())/1000.0; // 0.05 sec 
			if (dt < 0){ throw std::runtime_error("Time went backwards."); }
			set_last_read_time(GetTimeStamp());
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			mpu.dmpGetDisplacement(dis, &aaWorld);
			
			
			// exponential moving average filter (EMA), 1st order low pass
			//acc_filt[0] = (float)(1 - alpha)* acc_filt_old[0] + alpha* dis[0];
			//acc_filt[1] = (float)(1 - alpha)* acc_filt_old[1] + alpha* dis[1];

			//acc_filt_old[0] = acc_filt[0];
			//acc_filt_old[1] = acc_filt[1];
			
			
			//vector offset; 
			//acc_min[0] = - 4217;  //filtered raw values
			//acc_max[0] = 4179;
			//acc_min[1] = - 3800;
			//acc_max[1] = 4300;
			//acc_min[2] = - 4148;
			//acc_max[2] = 4100;
			  
			//offset(0) = (float)(acc_max(0) + acc_min(0))/ 2;
			//offset(1) = (float)(acc_max(1) + acc_min(1))/ 2;
			//offset(2) = (float)(acc_max(2) + acc_min(2))/ 2;
			  
			//return (acc_filt - offset) * get_acc_scale(); //return in g's
			
			acc_x_ms = dis[0]*9.81;
			acc_y_ms = dis[1]*9.81;
			
			vel_x = vel_x_old + 0.5*(acc_x_ms + acc_x_ms_old)*dt;   
			vel_y = vel_y_old + 0.5*(acc_y_ms + acc_y_ms_old)*dt;
     
			acc_x_ms_old = acc_x_ms;
			acc_y_ms_old = acc_y_ms;
     
			vel_x_old = vel_x;
			vel_y_old = vel_y;
			
			pos_x = pos_x_old + 0.5*(vel_x + vel_x_old)*dt;
			pos_y = pos_y_old + 0.5*(vel_y + vel_y_old)*dt;
         
			pos_x_old = pos_x;
			pos_y_old = pos_y;
			
            printf("disp %7.3f %7.3f  %7.3f \n ", dis[0], dis[1], dt);

        #endif
        */

        //printf("\n");
    }
}

