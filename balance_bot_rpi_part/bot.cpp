#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <system_error>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <poll.h>
#include <sys/timerfd.h>

#include "demo_dmp.h"

#include "pi2c.h"

#define STX 0x02
#define ETX 0x03
#define A 0x01
#define B 0x04
#define C 0x05

// blutetooth
int client;
struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
char buf[64] = { 0 }; //1024

int s, bytes_read;
int ret;
socklen_t opt = sizeof(rem_addr);
bdaddr_t my_bdaddr_any = { 0, 0, 0, 0, 0, 0 };
//bdaddr_t my_bdaddr_all = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
//bdaddr_t my_bdaddr_local = {0, 0, 0, 0xff, 0xff, 0xff};

// array for user input
char input[2];

//PID angle
float angleNow = 0.0;


// PID IMU coeff
float kp = 12.0; //5
float ki = 0.0;
float kd = 0.0; //19
int pidOut_IMU = 0;

// PID VEL coeff
float kp_v = 0.0;
float ki_v = 0.0;
float kd_v = 0.0;
float pidOut_Speed = 0.0;

//bluetooth
char foo[] = "000000";
char displayStatus[] = "kesle";
char buffer[64]; //1024

int p = 0;
int* joyS = {&p};


// arduino
Pi2c arduino(9); //Create a new object "arduino" using address "0x09"
int16_t r = 0;
int16_t* receive  = {&r};

// IMU
//usleep(100000);
//float yaw, pitch, roll;

pthread_mutex_t count_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condition_var = PTHREAD_COND_INITIALIZER;

sem_t sec_mutex, thrd_mutex;

int fl = 0;
int count = 0;
//int flag= 0;

int* getJoystickState(char data[8])
{

    static int Joy[2];
    int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain an Int from the ASCII representation
    int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
    joyX = joyX - 200; // Offset to avoid
    joyY = joyY - 200; // transmitting negative numbers
    //printf("joyX: %d	joyY: %d \n", joyX, joyY);
    //printf("received %d, %d\n", data[1], data[2]);
    Joy[0] = joyX;
    Joy[1] = joyY;
    return Joy;
}

int GetdataInt1()
{ // Data dummy values sent to Android device for demo purpose
    static int i = -30; // Replace with your own code
    i++;
    if (i > 0)
        i = -30;
    return i;
}

float GetdataFloat2()
{ // Data dummy values sent to Android device for demo purpose
    static float i = 50; // Replace with your own code
    i -= .5;
    if (i < -50)
        i = 50;
    return i;
}

/*
String getButtonStatusString()  {
 String bStatus = "";
 for(int i=0; i<6; i++)  {
   if(buttonStatus & (B100000 >>i))      bStatus += "1";
   else                                  bStatus += "0";
 }
 return bStatus;
}
*/

void sendBlueToothData()
{

    //get_attitude();
    //printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);

    sprintf(buffer, "%c %s %c %d %c %6.3f %c %s %c",
        STX, foo, A, GetdataInt1(), B, GetdataFloat2(), C, displayStatus, ETX);

    write(client, buffer, strlen(buffer));

    /*
   mySerial.print((char)STX); // Start of Transmission
   mySerial.print(getButtonStatusString());  // getButtonStatusString()
   mySerial.print((char)0x1);   // buttons status feedback
   mySerial.print(GetdataInt1());
   mySerial.print((char)0x4);   // datafield #1
   mySerial.print(GetdataFloat2());
   mySerial.print((char)0x5);   // datafield #2
   mySerial.print(displayStatus);// datafield #3
   mySerial.print((char)ETX); // End of Transmission 
   */
}

void timespec_add_us(struct timespec* t, long us)
{
    t->tv_nsec += us * 1000;
    if (t->tv_nsec > 1000000000) {
        t->tv_nsec = t->tv_nsec - 1000000000; // + ms*1000000;
        t->tv_sec += 1;
    }
}

int mymillis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
}

struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
};

static int make_periodic (unsigned int period, struct periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	//fd = timerfd_create (CLOCK_REALTIME, 0);
	fd = timerfd_create (CLOCK_MONOTONIC, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	return ret;
}

static void wait_period (struct periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read (info->timer_fd, &missed, sizeof (missed));
	if (ret == -1)
	{
		perror ("read timer");
		return;
	}

	/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	if (missed > 0)
		info->wakeups_missed += (missed - 1);
}

void* function1(void* arg)
{

    //  struct timespec timeoutStart, timeoutEnd;
    //  double mainTdiff = 0.0;
		
	struct periodic_info info;
	make_periodic (10000, &info);

	float pitch;
	
    while (fl == 0) {
        sem_wait(&sec_mutex);

    //  clock_gettime(CLOCK_MONOTONIC, &timeoutStart);
	loop();
        //yaw = get_yaw();
        pitch = get_pitch();
        //roll = get_roll();
	angleNow = pitch;
        //printf("pitch  %7.2f %7.2f %7.2f \n", yaw, pitch, roll);
	//printf("angleNow %7.2f \n", angleNow);

        //pthread_mutex_unlock(&count_mutex);
	sem_post(&sec_mutex);
	wait_period (&info);

    //  clock_gettime(CLOCK_MONOTONIC, &timeoutEnd);
    //  mainTdiff = (timeoutEnd.tv_sec - timeoutStart.tv_sec) + (timeoutEnd.tv_nsec - timeoutStart.tv_nsec) / 1E9; //sec
    //  long diff_us = mainTdiff * 1000000L; //us

    //  printf( "Elasped Time1: %lu \n", diff_us);
    }
    return NULL;
}

void* function2(void* arg)
{
    //  struct timespec timeoutStart, timeoutEnd;
    //  double mainTdiff = 0.0;

	struct periodic_info info;
	make_periodic (10000, &info);
	
	joyS[0] = 0;

    while (fl == 0) {

    //  clock_gettime(CLOCK_MONOTONIC, &timeoutStart);
        // read data from the client (bluetooth)
        bytes_read = read(client, buf, sizeof(buf));

        if (bytes_read > 0) {

            //printf("bytes_read %d\n", bytes_read); // 8 bytes
            //printf("received [%s]\n", buf);

            joyS = getJoystickState(buf);
			
            if (errno == ERANGE) {
                printf("range error, got ");
                errno = 0;
            }
            // If a conversion error occurred, display a message and exit
            if (errno == EINVAL) {
                printf("Conversion error occurred: %d\n", errno);
                exit(0);
            }
        }
	//printf("target1: %d	target2: %d \n", joyS[0], joyS[1]);

        //sendBlueToothData();  //send data to phone
	wait_period (&info);
        //pthread_mutex_unlock(&count_mutex);

    //  clock_gettime(CLOCK_MONOTONIC, &timeoutEnd);
    //  mainTdiff = (timeoutEnd.tv_sec - timeoutStart.tv_sec) + (timeoutEnd.tv_nsec - timeoutStart.tv_nsec) / 1E9;
    //  long diff_us = mainTdiff * 1000000L;
	
    //  printf( "Elasped Time2: %lu \n", diff_us);
    }

    return NULL;
}

void* function3(void* arg)
{

    //struct timespec timeoutStart, timeoutEnd;
    //double mainTdiff = 0.0;
	
	struct periodic_info info;
	make_periodic (1000, &info);
	
	float ITerm = 0.0;
	float DTerm = 0.0;
	int outMax = 550;
	int outMin = -550;
	float targetAngle = 0.0;
	float angleZero = 1.4;
	
	float angleLast = 0;

	float errAngle = 0.0;
	//float lastErrAngle = 0.0;
	
	int16_t speedTarget1 = 0;
	int16_t speedTarget2 = 0;

    while (fl == 0) {
		
	//clock_gettime(CLOCK_MONOTONIC, &timeoutStart);
        sem_wait(&sec_mutex);
        // Lock mutex and then wait for signal to relase mutex
        //pthread_mutex_lock(&count_mutex);
		
	if ( ((angleNow > 0) && (angleNow - angleZero) > 40.0) || ((angleNow < 0) &&  (angleNow + angleZero) < -40.0) ) {
				
			DTerm = 0.0;
			ITerm = 0.0;
			pidOut_IMU = 0;
			pidOut_Speed = 0.0;
			//printf("	pidOut_IMU_off  %d\n", pidOut_IMU );
		}
	else{
			
		targetAngle = angleZero - pidOut_Speed ;
		errAngle = targetAngle - angleNow;				
		ITerm += ki * errAngle;
			
		if(ITerm > outMax) ITerm = outMax;
		else if(ITerm < outMin) ITerm = outMin;
			
		DTerm = angleNow - angleLast;
		//DTerm = errAngle - lastErrAngle;	
			
		//pidOut_IMU = round(ITerm - (kp * DTerm) - (kd * DTerm));			 
		pidOut_IMU = round((kp * errAngle) + ITerm - (kd * DTerm));			
		if (pidOut_IMU > outMax){
		//ITerm -= pidOut_IMU - outMax;
		pidOut_IMU = outMax;
		}
		else if (pidOut_IMU < outMin){
		//ITerm += outMin -  pidOut_IMU;
		pidOut_IMU = outMin;
		}
			
		//lastErrAngle = errAngle;
		angleLast = angleNow;
		//printf("	pidOut_IMU  %d\n", pidOut_IMU );
	}
		
		
	speedTarget1 = pidOut_IMU + (joyS[0]*5); // re-scale 0-100 ->  0-500
	speedTarget2 = pidOut_IMU - (joyS[0]*5);
		
	if (speedTarget1 > 550) speedTarget1 = 550;
	else if (speedTarget1 < -550) speedTarget1 = -550;
	if (speedTarget2 > 550) speedTarget2 = 550;
	else if (speedTarget2 < -550) speedTarget2 = -550;		
		
	//speedTarget1 = joyS[0]*5;
	//speedTarget2 = joyS[0]*5;
       
	arduino.i2cWriteArduino2Int(speedTarget1, speedTarget2 );  // max 520

	//printf("target1: %d	target2: %d \n", speedTarget1, speedTarget2);
		
        //Send an 8 bit integer
        //arduino.i2cWriteArduinoInt(pidOut_IMU);
	sem_post(&sec_mutex);
	wait_period (&info);
        //pthread_mutex_unlock(&count_mutex);

    //  clock_gettime(CLOCK_MONOTONIC, &timeoutEnd);
    //  mainTdiff = (timeoutEnd.tv_sec - timeoutStart.tv_sec) + (timeoutEnd.tv_nsec - timeoutStart.tv_nsec) / 1E9; //sec
    //  long diff_us = mainTdiff * 1000000L; //us

    //  printf( "Elasped Time3: %lu \n", diff_us);
		
    }
    return NULL;
}

void* function4(void* arg)
{

    //struct timespec timeoutStart, timeoutEnd;
    //double mainTdiff = 0.0;

	struct periodic_info info;
	make_periodic (5000, &info);

	int speedNow = 0;
	int targetSpeed = 0;
	int errSpeed;
	int lastErrSpeed = 0;
	float ITerm = 0.0;
	int DTerm = 0;
	float outMax = 30.0;
	float outMin = -30.0;

	
	receive[0] = 0;
	joyS[1] = 0;

    while (fl == 0) {
        // sem_wait(&sec_mutex);
        // clock_gettime(CLOCK_MONOTONIC, &timeoutStart);

        // Lock mutex and then wait for signal to relase mutex
        // pthread_mutex_lock(&count_mutex);
		
	//Receive from the Arduino and put the contents into the "receive" char array
        receive = arduino.i2cReadArduinoInt();
		
        //Print out what the Arduino is sending...
        //printf("rcv1: %d \n", receive[0]);
		
	speedNow = receive[0];
	targetSpeed = joyS[1]*5;
		
	//printf("target speed: %d speed now: %d \n", targetSpeed , speedNow);
		                            
	errSpeed = speedNow - targetSpeed;				
	ITerm += (ki_v * errSpeed);
	DTerm = errSpeed - lastErrSpeed;	
	//if(ITerm > outMax) ITerm = outMax;
	//else if(ITerm < outMin) ITerm = outMin;			 
	pidOut_Speed = (kp_v * errSpeed) + ITerm + (kd_v * DTerm);
		
		
	if (pidOut_Speed > outMax){
		ITerm -= pidOut_Speed - outMax;
		pidOut_Speed = outMax;
	}
	else if (pidOut_Speed < outMin){
		ITerm += outMin - pidOut_Speed;
		pidOut_Speed = outMin;
	}
		
	lastErrSpeed = errSpeed;
	//printf("	pidOut_Speed  %d\n", pidOut_Speed );

	wait_period (&info);
    //    clock_gettime(CLOCK_MONOTONIC, &timeoutEnd);
    //    mainTdiff = (timeoutEnd.tv_sec - timeoutStart.tv_sec) + (timeoutEnd.tv_nsec - timeoutStart.tv_nsec) / 1E9; //sec
    //    long diff_us = mainTdiff * 1000000L; //us

    //    printf( "Elasped Time4: %lu \n", diff_us);
    }
    return NULL;
}

void set_nonblock(int socket)
{
    int flags;
    flags = fcntl(socket, F_GETFL, 0);
    //assert(flags != -1);
    fcntl(socket, F_SETFL, flags | O_NONBLOCK);
}

int main(int argc, char* argv[])
{

    pid_t pid = 0;
    int policy = SCHED_FIFO;
    //int maxpr = 0; // 0-99
    struct sched_param param;
    int maxpr = sched_get_priority_max(policy);
    param.sched_priority = maxpr;
    sched_setscheduler(pid, policy, &param);

    pthread_t thread1, thread2, thread3, thread4;

    pthread_attr_t my_attr;
    pthread_attr_init(&my_attr);

    sem_init(&sec_mutex,0,1);

    setup(); //IMU

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = (bdaddr_t&)my_bdaddr_any; //*BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t)1;
    ret = bind(s, (struct sockaddr*)&loc_addr, sizeof(loc_addr));

    if (ret == -1) {
        printf("Bluetooth bind failed. ERRNO=%d\n", errno);
        char* errorMessage = strerror_r(errno, buf, 1024);
        printf("%s\n", errorMessage);
        return 0;
    }

    // put socket into listening mode
    listen(s, 1); // 1 = queue

    // accept one connection
    client = accept(s, (struct sockaddr*)&rem_addr, &opt);

    ba2str(&rem_addr.rc_bdaddr, buf);
    fprintf(stderr, "accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    set_nonblock(client);

    int chk;
    // Initialze the encoder mutex (mutual exclusion)
    chk = pthread_mutex_init(&count_mutex, NULL);
    if (chk) {
        perror("pthread_mutex create failed\n");
        pthread_exit(NULL);
    }

    param.sched_priority = 5;
    pthread_attr_setschedparam(&my_attr, &param);
    chk = pthread_create(&thread1, &my_attr, &function1, NULL);
    if (chk) {
        printf("unable to create poll thread1");
    }
	
    param.sched_priority = 15;
    pthread_attr_setschedparam(&my_attr, &param);
    chk = pthread_create(&thread2, &my_attr, &function2, NULL);
    if (chk) {
        printf("unable to create poll thread2");
    }
	
    param.sched_priority = 6;
    pthread_attr_setschedparam(&my_attr, &param);
    chk = pthread_create(&thread3, &my_attr, &function3, NULL);
    if (chk) {
        printf("unable to create poll thread3");
    }

    param.sched_priority = 7;
    pthread_attr_setschedparam(&my_attr, &param);
    chk = pthread_create(&thread4, &my_attr, &function4, NULL);
    if (chk) {
        printf("unable to create poll thread4");
    }

	
    do {
        printf("Enter pid params or 0 to exit\n");
        scanf("%s", input);
		
        //printf("Entered %s\n", input);
        if (strcmp(input, "1+") == 0) {
            kp += 0.5;
            printf("kp = %f\n", kp);
        }
        else if (strcmp(input, "1-") == 0) {
            kp -= 0.5;
            printf("kp = %f\n", kp);
        }

        if (strcmp(input, "2+") == 0) {
            ki += 0.001;
            printf("ki = %f\n", ki);
        }
        else if (strcmp(input, "2-") == 0) {
            ki -= 0.001;
            printf("ki = %f\n", ki);
        }

        if (strcmp(input, "3+") == 0) {
            kd += 0.5;
            printf("kd = %f\n", kd);
        }
        else if (strcmp(input, "3-") == 0) {
            kd -= 0.5;
            printf("kd = %f\n", kd);
        }
		printf( "pidOut_IMU: %d \n", pidOut_IMU);

    } while (0 != strcmp(input, "0")); // key 0
    fl = 1;
	
    arduino.i2cWriteArduino2Int(0,0); 

    // close connection
    close(client);
    close(s);

    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);
    pthread_join(thread4, NULL);
	
	sem_destroy(&sec_mutex);
    //sem_getvalue(&sec_mutex, &value);
    //printf("The value of the semaphors is %d\n", value);
    pthread_attr_destroy(&my_attr);
	
    printf("EXITINGGGGG\n");
    exit(EXIT_SUCCESS);
    return 0;
}
