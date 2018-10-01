#include <Wire.h>

#define SLAVE_ADDRESS 0x09 // arduino slave addr

#define InA1 7 // INA motor pin
#define InB1 8 // INB motor pin
#define PWM1 9 // PWM motor pin
#define encodPinA1 3 // encoder A pin
#define encodPinB1 12 // encoder B pin

#define InA2 6 // INA motor pin
#define InB2 4 // INB motor pin
#define PWM2 10 // PWM motor pin
#define encodPinA2 2 // encoder A pin
#define encodPinB2 5 // encoder B pin

#define LOOPTIME 2000 // PID loop time

unsigned long lastMilli = 0; // loop timing
unsigned long lastMilliPrint = 0; // loop timing
int speed_req1 = 0; // speed (Set Point)
int speed_req2 = 0; 
//int speed_act1 = 0;                              // speed (actual value)
//int speed_act2 = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;

//volatile long count1 = 0;                        // rev counter
//volatile long count2 = 0;                        // rev counter

float Kp1 = 0.7; // 2.0
float Ki1 = 0.03; // 0.2
float Kp2 = 0.7;
float Ki2 = 0.03;

float outMax = 1023.0;
float outMin = -1023.0;

byte receivedCommands[4] = { 0 };

volatile byte rev1, rev2;
volatile long dTime1, dTime2, timeold1, timeold2;
unsigned int rpm1, rpm2;

int speedAv = 0;

unsigned long lastMillitime =0;
unsigned long timechck = 0;
void setup()
{

    Serial.begin(115200);
    //Serial.begin(9600);

    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
	  Wire.setClock(400000L);
    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    setupPWM16();

    dTime1 = 0, rev1 = 0, rpm1 = 0;
    dTime2 = 0, rev2 = 0, rpm2 = 0;
    timeold1 = 0, timeold2 = 0;

    pinMode(InA1, OUTPUT);
    pinMode(InB1, OUTPUT);
    pinMode(PWM1, OUTPUT);

    pinMode(encodPinA1, INPUT);
    pinMode(encodPinB1, INPUT);
    //digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
    //digitalWrite(encodPinB1, HIGH);
    attachInterrupt(1, rencoder1, RISING);

    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite16(PWM1, PWM_val1);

    pinMode(InA2, OUTPUT);
    pinMode(InB2, OUTPUT);
    pinMode(PWM2, OUTPUT);

    pinMode(encodPinA2, INPUT);
    pinMode(encodPinB2, INPUT);
    //digitalWrite(encodPinA2, HIGH);             // turn on pullup resistor
    //digitalWrite(encodPinB2, HIGH);
    attachInterrupt(0, rencoder2, RISING);

    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite16(PWM2, PWM_val2);
}

void loop()
{

    sei(); // enable interupts
    //getParams();
    MotorData();

    if ((micros() - lastMilli) >= LOOPTIME) { // enter tmed loop
        lastMilli = micros();
        
        if ((speed_req1 > 0) && (speed_req1 <= 550)) {
            PWM_val1 = updatePid1(speed_req1, rpm1);
            //digitalWrite(InA1, HIGH);
            PORTD |= B10000000;	
            //digitalWrite(InB1, LOW);      
            PORTB &= B11111110;
            }
        else if ((speed_req1 < 0) && (speed_req1 >= -550))  {
	    PWM_val1 = updatePid1(speed_req1, rpm1);
            //digitalWrite(InA1, LOW);
            PORTD &= B01111111;
            //digitalWrite(InB1, HIGH);
            PORTB |= B00000001;
            }
        else {
            //digitalWrite(InA1, LOW);
            PORTD &= B01111111;
            //digitalWrite(InB1, LOW);
            PORTB &= B11111110;
            }
    				
        if ((speed_req2 > 0) && (speed_req2 <= 550)) {
            PWM_val2 = updatePid2(speed_req2, rpm2);
            //digitalWrite(InA2, LOW);
	    PORTD &= B10111111;
            //digitalWrite(InB2, HIGH);
	    PORTD |= B00010000;
            }
        else if ((speed_req2 < 0) && (speed_req2 >= -550))  {
            PWM_val2 = updatePid2(speed_req2, rpm2);
            //digitalWrite(InA2, HIGH);
            PORTD |= B01000000;
            //digitalWrite(InB2, LOW);
            PORTD &= B11101111;

            }
        else {
            //digitalWrite(InA2, LOW);
            PORTD &= B10111111;
            //digitalWrite(InB2, LOW);
            PORTD &= B11101111;
            }
        
        analogWrite16(PWM1, PWM_val1);
        analogWrite16(PWM2, PWM_val2);

        if (rpm1 <= 20) rpm1 = 0;
        if (rpm2 <= 20) rpm2 = 0;
        
	if (speed_req1 > 0  && speed_req2 > 0) speedAv = speed_av(rpm1,rpm2);
	else if (speed_req1 < 0  && speed_req2 < 0) speedAv = speed_av(-rpm1,-rpm2);
	else speedAv = 0;
        //sei();
    }
	
  //printInfo();

}

int speed_av( int rpmw1 , int rpmw2){
	float speed1, speed2, speedav;
	float radious = 0.09;
	
	speed1 = rpmw1 * 2 * PI * radious;  // (m/min)
	speed2 = rpmw2 * 2 * PI * radious;
	speedav = (speed1 + speed2)/2;
	
	if (speedav > 0)
        speedav += 0.5;
  	else if (speedav < 0)
        speedav -= 0.5;
		
	return int(speedav);
}

// Configure digital pins 9 and 10 as 16-bit PWM outputs.
void setupPWM16()
{
    DDRB |= _BV(PB1) | _BV(PB2); // set pins as outputs
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) // non-inverting PWM
        | _BV(WGM11); // mode 14: fast PWM, TOP=ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10); // no prescaling
    ICR1 = 0x03ff; // TOP counter value 0xffff = 65535, 0x03ff = 1023
}

// 16-bit version of analogWrite(). Works only on pins 9 and 10.
void analogWrite16(uint8_t pin, uint16_t val)
{
    switch (pin) {
    case 9:
        OCR1A = val;
        break;
    case 10:
        OCR1B = val;
        break;
    }
}

// callback for received data
void receiveData(int byteCount)
{
  int pckg = 4;
  for (int a = 0; a < byteCount; a++) {
        if (a < pckg) { //4
            receivedCommands[a] = Wire.read();
        }
        else {
			      while (Wire.available () > 0) Wire.read(); // if we receive more data then allowed just throw it away
        }
    
    }
  speed_req1 = ((int)receivedCommands[0] | ((int)receivedCommands[1]) << 8);
  speed_req2 = ((int)receivedCommands[2] | ((int)receivedCommands[3]) << 8);

}

// callback for sending data
void sendData()
{

	  byte value[2];
    value[0] = speedAv & 0xff;
    value[1] = (speedAv >> 8) & 0xff;

    Wire.write(value,2);
  
    // send int
    //Wire.write(speedAv);
    //Wire.write(",");
    //Wire.write(rpm2);
    // or string
    //unsigned char char_ar[16] = "Hi Raspberry Pi"; //Create String
    //Wire.write(char_ar,16); //Write String to Pi.
}


void MotorData()
{
    /*                                         // calculate speed, volts and Amps
 static long countAnt1 = 0;
 static long countAnt2 = 0;                                      // last count
 cli();
 speed_act1 = (abs(count1 - countAnt1)*(60*(1000/LOOPTIME)))/(300); // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt1 = count1;  
 speed_act2 = (abs(count2 - countAnt2)*(60*(1000/LOOPTIME)))/(300);  // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt2 = count2;   
 sei();
*/
    if (rev1 > 3) {
        cli(); //disable interrupts while we're calculating
        if (dTime1 > 0) //check for timer overflow
        {
            rev1 -= 1; //subtract one since the first revolution is not measured
            rpm1 = ((60000000 * rev1) / (dTime1)) / 300;
            // freq=rpm/60;
            // Serial.print(rev);
            //  Serial.print(" ");
            //  Serial.print(rpm);            //a bit of serial for the debugging (not really needed at this point, perhaps one day for some graphs)
            //  Serial.print(" ");
            //  Serial.println(dTime);
            rev1 = 0;
        }
        sei(); //re-enable interrupts
    }

    if (rev2 > 3) {
        cli();
        if (dTime2 > 0) //check for timer overflow
        {
            rev2 -= 1; //subtract one since the first revolution is not measured
            rpm2 = ((60000000 * rev2) / (dTime2)) / 300;
            rev2 = 0;
        }
        sei();
    }
}

int updatePid1(int targetValue, int currentValue)
{
    static float ITerm = 0;
    static int lastCurrentValue = 0;
    
    int error = abs(targetValue) - currentValue;
    int dterm = currentValue - lastCurrentValue;
    ITerm += Ki1 * error;
    ITerm -= Kp1 * dterm;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;

    float pidTerm = ITerm;
   // if (pidTerm > 0)
    pidTerm += 0.5;
   // else if (pidTerm < 0)
   //     pidTerm -= 0.5;
   lastCurrentValue  = currentValue;
   //Serial.println(pidTerm);
    return constrain(int(pidTerm), 0, 1023);
}

int updatePid2(int targetValue, int currentValue)
{

    static float ITerm = 0;
	static int lastCurrentValue = 0;

    int error = abs(targetValue) - currentValue;
	int dterm = currentValue - lastCurrentValue;
    ITerm += Ki2 * error;
	ITerm -= Kp2 * dterm;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;

    float pidTerm = ITerm;
   // if (pidTerm > 0)
    pidTerm += 0.5;
    //else if (pidTerm < 0)
    //    pidTerm -= 0.5;
	lastCurrentValue  = currentValue;

    return constrain(int(pidTerm), 0, 1023);
}

void printInfo()
{ // display data
    if ((millis() - lastMilliPrint) >= 50) {
        lastMilliPrint = millis();
        Serial.print("SP1:");
        Serial.print(speed_req1);
        Serial.print("  RPM1:");
        Serial.print(rpm1);
        Serial.print("  RPM2:");
        Serial.println(rpm2);
    }
}

/*
void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
 if (PIND & 0b00000100)    count1++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}


void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
 if (PIND & 0b00001000)    count2++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count2--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}
*/

void rencoder1()
{
    if (rev1 == 0) {
        timeold1 = micros(); //first measurement is unreliable since the interrupts were disabled
        rev1++;
    }
    else {
        dTime1 = (micros() - timeold1); //'micros()' is not incrementing while inside the interrupt so it should be safe like this right?
        rev1++;
    }
}

void rencoder2()
{
    if (rev2 == 0) {
        timeold2 = micros(); //first measurement is unreliable since the interrupts were disabled
        rev2++;
    }
    else {
        dTime2 = (micros() - timeold2); //'micros()' is not incrementing while inside the interrupt so it should be safe like this right?
        rev2++;
    }
}

int getParams()
{
    char param, cmd;
    if (!Serial.available())
        return 0;
    delay(10);
    param = Serial.read(); // get parameter byte
    if (!Serial.available())
        return 0;
    cmd = Serial.read(); // get command byte
    Serial.flush();
    switch (param) {
    case 'v': // adjust speed
        if (cmd == '+') {
            speed_req1 += 20;
            speed_req2 += 20;
            if (speed_req1 > 550) speed_req1 = 550;
            if (speed_req2 > 550) speed_req2 = 550;
        }
        if (cmd == '-') {
            speed_req1 -= 20;
            speed_req2 -= 20;
            if (speed_req1 < -550) speed_req1 = -550;
            if (speed_req2 < -550) speed_req2 = -550;
        }
        break;

    case 's': // adjust direction
        if (cmd == '+') {
            digitalWrite(InA1, LOW);
            digitalWrite(InB1, HIGH);
            digitalWrite(InA2, HIGH);
            digitalWrite(InB2, LOW);
        }
        if (cmd == '-') {
            digitalWrite(InA1, HIGH);
            digitalWrite(InB1, LOW);
            digitalWrite(InA2, LOW);
            digitalWrite(InB2, HIGH);
        }
        break;

    case 'o': // user should type "oo"
        digitalWrite(InA1, LOW);
        digitalWrite(InB1, LOW);
        digitalWrite(InA2, LOW);
        digitalWrite(InB2, LOW);
        speed_req1 = 0;
        speed_req2 = 0;
        break;
		
	case 'p': // adjust direction
        if (cmd == '+') Kp1 += 0.1;
        if ((cmd == '-') && (Kp1 > 0)) Kp1 -= 0.1;
        Serial.print("                  Kp1: ");
        Serial.println(Kp1,4);
        break;
		
	case 'i': // adjust direction
        if (cmd == '+') Ki1 += 0.01;
        if (cmd == '-') Ki1 -= 0.01;
        Serial.print("                  Ki1: ");
        Serial.println(Ki1,5);
        break;
		
    default:
        Serial.println("..");
    }
}
