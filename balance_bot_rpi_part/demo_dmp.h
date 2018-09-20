#ifndef _DEMO_DMP_H_
#define _DEMO_DMP_H_

void setup();
void loop();
void set_ypr(float *Ypr);

float get_yaw();
float get_pitch();
float get_roll();

float compFilter(float angularRate, float accAngle, float angle, float gain);
#endif
