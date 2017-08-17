/*
    *Arduino Autonomous Vehicle
    *It incorporates signals from RaspberryPi GPIO pins, Arduino Ultrasonic sensors,
    *
    *@author: Peter Ye
    *Date: Fianl edit: May 10, 2017
	
*/

//Includes libraries
//#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// -------------------------------------------------------------------------
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
int pwm_value1 = 0;
int pwm_value2 = 0;
int pwm_out1 = 0;
int pwm_out2 = 0;
int x_position;
int potentialmeter;
int pwm1;
int pwm2;
int vehiclespeed = 0;
int cameravalue = 0;
int LIDARvalue = 0;
int pwmMIN = 2270;
long durationr, distancer;
long durationl, distancel;
float counter1=0.0;
float counter2=0.0;
uint8_t servonum1 = 0; //left motor
uint8_t servonum2 = 1;//right motor, value needs to higher than left by 10
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// -------------------------------------------------------------------------
void setup() {
// pinMode(IRPin2,INPUT);
// pinMode (IRPin1, INPUT) ;
pinMode(Joystick_INPINX, INPUT);
pinMode(Potentialmeter_PinIn, INPUT);
pinMode(PWM_PIN1, INPUT);
pinMode(PWM_PIN2, INPUT);
pinMode(trigPinr, OUTPUT);
pinMode(echoPinr, INPUT);
pinMode(trigPinl, OUTPUT);
pinMode(echoPinl, INPUT);
pinMode(CameraPIN, INPUT);
// Timer1.initialize(2000000); // set timer for 1sec
// attachInterrupt(0, docount1, FALLING); // increase counter when speed sensor pin goes low
// attachInterrupt(1, docount2, FALLING); // increase counter when speed sensor pin goes low
// Timer1.attachInterrupt( timerIsr ); // enable the timer
Serial.begin(9600);
pwm.begin();
pwm.setPWMFreq(400); // Analog servos run at ~0 Hz updates
// yield();
}
//void docount1() // counts from the speed sensor
//{
// counter1++; // increase +1 the counter value
//}
//
//void docount2() // counts from the speed sensor
//{
// counter2++; // increase +1 the counter value
//}
//void timerIsr()
//{
// Timer1.detachInterrupt(); //stop the timer
//// Serial.print("Motor1 Speed: ");
// float rotation1 = (counter1 / 2); // divide by number of holes in Disc
//// Serial.println(rotation1,2);
//// Serial.println(" Rotation per seconds");
// counter1=0; // reset counter to zero
//// Serial.print("Motor2 Speed: ");
// float rotation2 = (counter2 / 2); // divide by number of holes in Disc
//// Serial.println(rotation2,2);
//// Serial.println(" Rotation per seconds");
// counter2=0; // reset counter to zero
// Timer1.attachInterrupt( timerIsr ); //enable the timer
//}
void checkUltrasonics()
{
// if (distancer >= 50 && distancer < 130 ) {
// pwm_out1 = pwm_out1 - (vehiclespeed/2);
// }
// if (distancel >= 50 && distancel < 130 ) {
// pwm_out2 = pwm_out2 - (vehiclespeed/2);
// }
if ((distancer >= 5 && distancer < 50) || (distancel >= 5 && distancel < 50)) {
pwm_out1 = pwmMIN;
pwm_out2 = pwmMIN;
}
}
void loop() {
pwm1 = pulseIn(PWM_PIN1, HIGH, 25000);
pwm2 = pulseIn(PWM_PIN2, HIGH, 25000);
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
// x_position = analogRead(Joystick_INPINX); //between 0 to 1023
// potentialmeter = analogRead(Potentialmeter_PinIn);
// pwm_value1 = map(potentialmeter, 0,1023, 2250, 3140); //stop at 2240-2340
pwm_value1 = map(pwm1, 1050,1950, pwmMIN, 3140);
pwm_value2= map(pwm2, 1050,1950, -50, 50);
pwm_value1 = constrain(pwm_value1, pwmMIN, 3140);
pwm_value2 = constrain(pwm_value2, -50, 50);
vehiclespeed = pwm_value1 - pwmMIN;
if(pwm_value2 > 5){
pwm_out2 = vehiclespeed*(50-pwm_value2)/50 + pwmMIN;
pwm_out1 = pwm_value1;
}
else if(pwm_value2 < -5){
pwm_out1 = vehiclespeed*(50+pwm_value2)/50 + pwmMIN;
pwm_out2 = pwm_value1;
}
else {
pwm_out1 = pwm_value1;
pwm_out2 = pwm_value1;
}
//control the PWM driver
checkUltrasonics();
if (digitalRead(CameraPIN) == HIGH) {
pwm.setPWM(servonum1, 0, pwmMIN);
pwm.setPWM(servonum2, 0, pwmMIN);
delay(3000);
}
else {
pwm.setPWM(servonum1, 0, pwm_out1);
pwm.setPWM(servonum2, 0, pwm_out2);
}
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
