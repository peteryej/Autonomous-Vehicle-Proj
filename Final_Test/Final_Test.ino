/*
    *Arduino Autonomous Vehicle
    *It incorporates signals from RaspberryPi GPIO pins, Arduino Ultrasonic sensors, 
    * GPS contorller board and send data to Arduino PWM shild to control the motors' speeds
    *@author: Peter Ye
    *Date: May 10, 2017
	
*/

//Includes libraries
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//-------------------------------------------------------------------------------------------
//define constants
#define trigPinr 13 //ultrasonic sensor trig pin
#define echoPinr 12 //ultrasonic sensor echo pin
#define trigPinl 11 //ultrasonic sensor trig pin
#define echoPinl 10 //ultrasonic sensor echo pin
const int Joystick_INPINX = 0; //joystick pin to A0
const int Potentialmeter_PinIn = 2; //Potentialmeter pin to A2
const int CameraPIN = 9; //camera digitalread pin
byte PWM_PIN1 = 8; //pixhawk pin 3 for throttle
byte PWM_PIN2 = 7; //pixhawk pin 1 for turn
const int IRPin2 = 2; //IR sensor INPUT
const int IRPin1 = 3; //IR sensor INPUT
int pwm_value1 = 0;  //left motor's intermediate mapped pwm value
int pwm_value2 = 0;  //right motor's intermediate mapped pwm value
int pwm_out1 = 0;   //left motor's output pwm value
int pwm_out2 = 0;   //right motor's output pwm value
int x_position;    //joystick analog value
int potentialmeter;  //potentialmeter analog value
int pwm1;            //input speed value from gps controller
int pwm2;            //input direction value from gps controller

int vehiclespeed = 0;  //initiate vehiclespeed variable
int cameravalue = 0;   //initiate cameravalue variable
int LIDARvalue = 0;    //initiate LIDARvalue variable
int pwmMIN = 2270;  //the min and max accepted value is found by testing with different 
int pwmMAX = 3140;  //pwm value for the motors' pwm controller
long durationr, distancer;  //variables used for ultrasonic sensor on the left side
long durationl, distancel;  //variables used for ultrasonic sensor on the right side
float counter1=0.0;  //counter variable used for IR speed sensor on the left wheel
float counter2=0.0;  //counter variable used for IR speed sensor on the right wheel
uint8_t servonum1 = 0; //left motor variable
uint8_t servonum2 = 1;//right motor variable
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  //initiate Adafruit_PWMServoDriver variable

//-------------------------------------------------------------------------------------------
//set up pinmodes, timer and start serial communication
void setup() {
pinMode(IRPin2,INPUT);
pinMode(IRPin1, INPUT) ;
pinMode(PWM_PIN1, INPUT);
pinMode(PWM_PIN2, INPUT);
pinMode(CameraPIN, INPUT);
//pinMode(Joystick_INPINX, INPUT);  //for testing the speed of motors
//pinMode(Potentialmeter_PinIn, INPUT);
//define ultrosonic sensor pins
pinMode(echoPinl, INPUT);
pinMode(trigPinr, OUTPUT);
pinMode(echoPinr, INPUT);
pinMode(trigPinl, OUTPUT);

//set up the timer
Timer1.initialize(2000000); // set timer for 1sec
attachInterrupt(0, docount1, FALLING); // increase counter when speed sensor pin goes low
attachInterrupt(1, docount2, FALLING); // increase counter when speed sensor pin goes low
Timer1.attachInterrupt( timerIsr ); // enable the timer

//start the serial communication
Serial.begin(9600);
pwm.begin();
pwm.setPWMFreq(400); // Analog servos run at ~0 Hz updates
// yield();
}

// counts from the left IR speed sensor
void docount1() 
{
 counter1++; // increase +1 the counter value
}

// counts from the right IR speed sensor
void docount2() 
{
 counter2++; // increase +1 the counter value
}

//set up timer interrupt routine
void timerIsr()
{
 Timer1.detachInterrupt(); //stop the timer
// Serial.print("Motor1 Speed: ");
 float rotation1 = (counter1 / 2); // divide by number of holes in Disc
// Serial.println(rotation1,2);     //serial print for testing purpose
// Serial.println(" Rotation per seconds");
 counter1=0; // reset counter to zero
// Serial.print("Motor2 Speed: ");
 float rotation2 = (counter2 / 2); // divide by number of holes in Disc
// Serial.println(rotation2,2);
// Serial.println(" Rotation per seconds");
 counter2=0; // reset counter to zero
 Timer1.attachInterrupt( timerIsr ); //enable the timer
}

//process signals from left and right ultrasonic sensors
void checkUltrasonics()
{
  //if left distance senses object, the vehicle will turn right by slowing down the right motor
 if (distancer >= 50 && distancer < 130 ) {
     pwm_out1 = pwm_out1 - (vehiclespeed/2);
     pwm_out2 = pwmMIN;
 }
 //if right distance senses object, the vehicle will turn left by slowing down the left motor
 if (distancel >= 50 && distancel < 130 ) {
     pwm_out2 = pwm_out2 - (vehiclespeed/2);
     pwm_out1 = pwmMIN;
 }

}

//start the mian function loop
void loop() {
  //read signals from GPS controller
pwm1 = pulseIn(PWM_PIN1, HIGH, 25000);
pwm2 = pulseIn(PWM_PIN2, HIGH, 25000);

//calculate the distance of the ultrasonic sensors
digitalWrite(trigPinr, LOW);
delayMicroseconds(2);
digitalWrite(trigPinr, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinr, LOW);
durationr = pulseIn(echoPinr, HIGH);
distancer = (durationr/2) / 29.1; //get distance in cm
digitalWrite(trigPinl, LOW);
delayMicroseconds(2);
digitalWrite(trigPinl, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinl, LOW);
durationl = pulseIn(echoPinl, HIGH);
distancel = (durationl/2) / 29.1; //get distance in cm

//Joystick and potentialmeter are used for testing the motor reponsiveness
// x_position = analogRead(Joystick_INPINX); //between 0 to 1023
// potentialmeter = analogRead(Potentialmeter_PinIn);
// pwm_value1 = map(potentialmeter, 0,1023, 2250, pwmMAX); //stop at 2240-2340

//calculate the right pwm value for the Arduino motor control shield
pwm_value1 = map(pwm1, 1050,1950, pwmMIN, pwmMAX);
pwm_value2= map(pwm2, 1050,1950, -50, 50);
pwm_value1 = constrain(pwm_value1, pwmMIN, pwmMAX); //the value range is between pwmmin and pwmMAX
pwm_value2 = constrain(pwm_value2, -50, 50); //the -50 coora the most left, and 50 is the most right direction
vehiclespeed = pwm_value1 - pwmMIN;

//change the direction data to values controlling left and right motors' speeds
//if the direction is to the right, the left motor is proportionately faster than the right motor
if(pwm_value2 > 5){
pwm_out2 = vehiclespeed*(50-pwm_value2)/50 + pwmMIN;
pwm_out1 = pwm_value1;
}
//if the direction is to the left, the right motor is proportionately faster than the left motor
else if(pwm_value2 < -5){
pwm_out1 = vehiclespeed*(50+pwm_value2)/50 + pwmMIN;
pwm_out2 = pwm_value1;
}
//when the direction is neutral, left motor speed equals the right motor speed
else {
pwm_out1 = pwm_value1;
pwm_out2 = pwm_value1;
}

//incorporate data from ultrasonic sensors
checkUltrasonics();

//incorporate data from camera, if camera detects stop sign, stop for 3 seconds
if (digitalRead(CameraPIN) == HIGH) {
pwm.setPWM(servonum1, 0, pwmMIN);
pwm.setPWM(servonum2, 0, pwmMIN);
delay(3000);
}
else {
pwm.setPWM(servonum1, 0, pwm_out1);
pwm.setPWM(servonum2, 0, pwm_out2);
}

//Serial print for testing purpose

// Serial.print("pwm_value1 ");
// Serial.println(pwm_out1);
// Serial.print("pwm_value2 ");
// Serial.println(pwm_out2);
// Serial.print("pixhawk throttle ");
// Serial.println(pwm1);
// Serial.print("vehicle speed ");
// Serial.println(vehiclespeed);
// Serial.print("turn ");
// Serial.println(pwm_value2);
// Serial.print("potentialmeter ");
// Serial.println(pwm_out1);
// Serial.print("right distance ");
// Serial.println(distancer);
// Serial.print("left distance ");
// Serial.println(distancel);
//
// delay(500);
}
