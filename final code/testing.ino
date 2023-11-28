#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

//US
#define trigPin 11  //US trigger on PB3
#define echoPin 2  //US echo on PD2

//mpu
#define YAW_THRESHOLD 90.0
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180


//servo
Servo myservo;
int servoAngle=90;

//lift fan
const int Lfan = 5; //p3 

//thrust fan
const int Tfan = 6; //p4


//MPU global variables
MPU6050 mpu(Wire);
unsigned long timer = 0;
float yaw;

// Constants
const float ACCELERATION_THRESHOLD = 0.1;  // Adjust as needed
const int STOPPED_DURATION = 2000;         // Time in milliseconds to consider it stopped

// Variables
unsigned long stopStartTime = 0;
bool hcStop =false;


//US global variables
long duration;
double distance;




void setup() {
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(Lfan, OUTPUT);
  pinMode(Tfan, OUTPUT);

  
  Serial.begin(9600);
  Wire.begin();
  myservo.attach(9);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // Stop if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  _delay_ms(1000);
  mpu.calcOffsets(true, true); // Gyro and accelerometer
  Serial.println("Done!\n");
  myservo.write(servoAngle);
  
  delay(1000);
}

void loop() {
  mpu.update();

  //add first loop bool
  
  yaw=-mpu.getAngleZ();
  map(yaw,-180,180,0,180);    //mapping yaw ***check yaw range
  servoAngle -= yaw;  //every loop will update the servo angle according to yaw  **check +-
  servoAngle=constrain(servoAngle,0,180);    //keeping it in servo range

  myservo.write(servoAngle);  //setting direction every loop

  delay(100);

  digitalWrite(Tfan,HIGH);
  digitalWrite(Lfan,HIGH);

  
  if(incomingWall()){
  
  analogWrite(Lfan,100);
  digitalWrite(Tfan,LOW);
    hcStop= true;    //global variable is false
  }
  
 
  //to be reworked
  if(hcStop|| isHcStopped()){
    
    servoAngle=sweep();  //angle of servo to go in longest direction
    myservo.write(servoAngle);  //updating new global servo direction after sweep
    delay(200);

    digitalWrite(Lfan,HIGH);    //starts fans again once new angle is found
    digitalWrite(Tfan,HIGH);
    hcStop=false;
  }
  
 
  
  
  
  /* mpu.update();

  yaw = -mpu.getAngleZ();
  Serial.print("Roll: ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tACCELERO  X: ");
  Serial.print(mpu.getAccX());
  Serial.print("\tPitch: ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tYaw: ");
  Serial.println(yaw);
  
  map(yaw, 180, -180, 0, 180); // Modify servo angle as needed
   myservo.write(90); // Update the servo angle
   delay(3000);
  myservo.write(sweep()); // Update the servo angle

  delay(10000);
  timer = millis();
  */
}


//idea
/* int getYaw(){
  
} 
*/


int sweep(){
  bool firstDelay =true;
  int Theta = 0;
  double tempDist = 0;
  double longestDist = 0;
  for (int i=0; i<181; i+=30){
   
    myservo.write(i);
     if(firstDelay){    //adding delay for first servo movement to 0degrees
      delay(200);
      firstDelay=false;
    }
  delay(300);  // delays and degrees to be fine tuned 300 delay works
    tempDist = USdist();
    Serial.print("Servo angle:");
    Serial.println(i);
    
    if( tempDist > longestDist){
      longestDist = tempDist;  
      Theta = i;
        //check with yaw value 
    }
    
  }
  return Theta;
  
}

bool isHcStopped(){
   mpu.update();  // Update IMU data

  float aX = mpu.getAccX();
  float aY = mpu.getAccY();
  float aZ = mpu.getAccZ();

  // Calculate total acceleration magnitude
  float totalAcceleration = sqrt(sq(aX) + sq(aY) + sq(aZ));

  // Check if the acceleration is below the threshold
  if (totalAcceleration < ACCELERATION_THRESHOLD) {
    // If it's below the threshold, start or update the timer
    if (stopStartTime == 0) {
      stopStartTime = millis();
    }
  } else {
    // If the acceleration is above the threshold, reset the timer
    stopStartTime = 0;
  }

  // Check if the hovercraft has been below the threshold for the specified duration
  return (stopStartTime != 0) && (millis() - stopStartTime >= STOPPED_DURATION);
}



bool incomingWall(){

  if(USdist()<15){
    return true;
  }
  return false;
}

double USdist(){

  digitalWrite(trigPin, LOW); //clear trig
  delayMicroseconds(2); // Wait for 2 microseconds
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // pulse for 10 microseconds
  
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);	//read input echo
  
  distance= duration*0.034/2;	//distance calc using 0.034cm/microsec
  
  
  Serial.print("Distance: ");
  Serial.println(distance);
  
  return distance;
}