/*
    *Arduino Autonomous Vehicle
    *It incorporates signals from RaspberryPi GPIO pins, Arduino Ultrasonic sensors,
    *
    *@author: Peter Ye
    *Date: Fianl edit: May 10, 2017
	
*/

//Includes libraries

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

byte PWM_PIN1 = 8; //Arduino pin 8 connecting with pixhawk pin 3 for throttle
byte PWM_PIN2 = 7; //Arduino pin 8 connecting with pixhawk pin 1 for turn
byte PWM_OUTPIN1 = 10; //Arduino pin 10 to control left motor
byte PWM_OUTPIN2 = 9; //Arduino pin 9 to control right motor
byte CameraPIN = 11; //Arduino pin 9 connecting with camera signal
byte LIDARPIN = 12; //Arduino pin 12 connecting with LIDAR signal


int pwm_value1; 
int pwm_value2;
int pwm_out1;
int pwm_out2;

int change = 0;
int pwm1;
int pwm2;
int vehiclespeed = 0; 
int cameravalue = 0;
int LIDARvalue = 0;
uint8_t servonum1 = 1;
uint8_t servonum2 = 0;
 
void setup() {
  pinMode(CameraPIN, INPUT); // Set our input pins as such
  pinMode(LIDARPIN, INPUT);
  pinMode(PWM_PIN1, INPUT);
  pinMode(PWM_PIN2, INPUT); 
  
  pinMode(PWM_OUTPIN1, OUTPUT); // Set our output pins as such 
  pinMode(PWM_OUTPIN2, OUTPUT); 
  Serial.begin(9600);       //for testing purpose, confirm output is right before connecting circuits
}
 
void loop() {
   
  cameravalue = digitalRead(CameraPIN); //read camera pin value, high signal shows camera detectes a stop sign.
  LIDARvalue = digitalRead(LIDARPIN); //read LIDAR pin value, high signal shows LIDAR detectes a obstacle within range.
  
  pwm1 = pulseIn(PWM_PIN1, HIGH, 25000);
  pwm2 = pulseIn(PWM_PIN2, HIGH, 25000);

//  pwm_value1 = map(pwm1, 1050,1950, 180, 215);  
  pwm_value1 = map(pwm1, 1050,1950, 2240, 3140);
  pwm_value2= map(pwm2, 1050,1950, -10, 10);
//  pwm_value2= pwm2- 1500; //-445 to 445
  
  pwm_value1 = constrain(pwm_value1, 2240, 3140);
//  pwm_value1 = constrain(pwm_value1, 183, 215);
  pwm_value2 = constrain(pwm_value2, -10, 10);
//  pwm_value2 = constrain(pwm_value2, -445, 445);
  
//  vehiclespeed = pwm_value1 - 183;
  vehiclespeed = pwm_value1 - 2240;
//  if (cameravalue == HIGH ) {
//   change = 1; 
//  }
 
//    if(pwm_value2 > 0){
//    pwm_out2 = vehiclespeed*(15-pwm_value2)/15 + 183;
//    pwm_out1 = pwm_value1;
//    }
//    else if(pwm_value2 < 0){
//    pwm_out1 = vehiclespeed*(15+pwm_value2)/15 + 183;
//    pwm_out2 = pwm_value1;
//    }   
    if(pwm_value2 > 0){
    pwm_out2 = vehiclespeed*(15-pwm_value2)/15 + 2240;
    pwm_out1 = pwm_value1;
    }
    else if(pwm_value2 < 0){
    pwm_out1 = vehiclespeed*(15+pwm_value2)/15 + 2240;
    pwm_out2 = pwm_value1;
    } 
    else {
    pwm_out1 = pwm_value1;
    pwm_out2 = pwm_value1;
    } 
//|| LIDARvalue == HIGH
//  if (change == 1 ) {
//      analogWrite(PWM_OUTPIN2, 185);
//      analogWrite(PWM_OUTPIN1, 185);
//      delay(3000);
//      change = 0;
//  }
//  else{
//    analogWrite(PWM_OUTPIN2, pwm_out2);
//    analogWrite(PWM_OUTPIN1, pwm_out1);
  pwm.setPWM(servonum1, 0, pwm_out1);
  pwm.setPWM(servonum2, 0, pwm_out2);
//  }

  // Below code is for testing purspose, thus commented out when values are confirmed to behave correctly.    
//  Serial.print("pwm_value1  ");
//  Serial.println(pwm_out1);
//  Serial.print("pwm_value2  ");
//  Serial.println(pwm_out2);
//  Serial.print("turn  ");
//  Serial.println(pwm_value2);
//  Serial.print("speed  ");
//  Serial.println(vehiclespeed); 
//  delay(1000);
}
