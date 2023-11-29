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
int forwardYaw;
int currentYaw;
float yaw;

// Constants
const float ACCELERATION_THRESHOLD = 0.1;  // Adjust as needed
const int STOPPED_DURATION = 2000;         // Time in milliseconds to consider it stopped

// Variables
unsigned long stopStartTime = 0;  //timer in isHcStopped()
int FanOnTime=2000;  //for delays
int FanOffTime=2000;  //for delays

//boolean logic
bool firstLoop = true; //first loop bool to start fans
bool hcStop =false;    //if hc stopped 


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
  while (status != 0) { } // stop if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  _delay_ms(1000);
  mpu.calcOffsets(true, true); // gyro and accelerometer offset calc.
  Serial.println("Done!\n");
  myservo.write(servoAngle);
  
  delay(1000);
}

void loop() {
  mpu.update();
  currentYaw=getYaw();    //set every loop
  
  
  
  
  //add first loop instructions
  if(firstLoop){
    
   digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    delay(FanOnTime);
    
  digitalWrite(Tfan,HIGH);  //then thrust fan turns on once skirt is inflated
  forwardYaw=currentYaw;
  firstLoop=false;
    
  }else if(hcStop){    //to be reworked
    
    //testing with recalibration when stopped

    mpu.calcOffsets(true, true);
    delay(1000);


    servoAngle=sweep();           //angle of servo to go in longest direction
    myservo.write(servoAngle);    //updating new global servo direction after sweep
    delay(300);
    
    digitalWrite(Lfan,HIGH);     //starts fans again once new angle is found
    //delay(FanOnTime);
    analogWrite(Tfan,255);
    hcStop=false;
  }
  
  
  servoAngle = currentYaw + 90;  //every loop will update the servo angle according to yaw  **check +-
  servoAngle=constrain(servoAngle,0,180);    //keeping it in servo range

  myservo.write(servoAngle);  //setting direction every loop

  if(incomingWall()){

    analogWrite(Lfan,0);
    digitalWrite(Tfan,LOW);
    delay(FanOffTime);
    hcStop= true;    //global variable is false


  }
 delay(200);



  Serial.print("yaw: ");
  Serial.println(yaw);
  Serial.print("forwardYaw: ");
 Serial.println(forwardYaw);
 Serial.print("currentYaw: ");
 Serial.println(currentYaw);
 
  
 
  
  
  
 
}


//idea
int getYaw(){
  yaw =mpu.getAngleZ();
  map(yaw,-180,180,0,180); 
 //yaw=yaw%360;
  
  
 /* if(yaw<-90){
    yaw=0;
  }else if(yaw >90){
    yaw=0;
  }
  */
  return (yaw);  //check if difference is good
} 



void turnCW(){

myservo.write(180);

int currentYaw = forwardYaw;

while(forwardYaw>(currentYaw+90)){
  forwardYaw=getYaw();


}

myservo.write(forwardYaw);


}


void adjustments(){



}

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
 
 //forwardYaw=currentYaw + Theta -90;
  //currentYaw=currentYaw + Theta -90;

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

  if(USdist()<50){
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
