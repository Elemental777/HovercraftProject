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

//servo
Servo myservo;
int servoAngle;

//lift fan
const int Lfan=4; //pd4 (DC)

//thrust fan
const int Tfan = 6; //p4 (PWM)


//MPU global variables
MPU6050 mpu(Wire);
float forwardYaw;   //where we want to go
float currentYaw;   //where we are pointing currently
float yaw;         //yaw value

// Constants
const float ACCELERATION_THRESHOLD = 0.5;  //value to consider HoverCraft stopped


// Variables
unsigned long accelTimer = 0;  //timer in isHcStopped()
int FanOnTime=2000;  //for delays
int FanOffTime=2000;  //for delays
float prevtA=3;  //first time calculating avg will use 3 as previous

// //boolean logic
bool firstLoop = true; //first loop bool to start fans



//US global variables
long duration;
int distance;
unsigned long sweepTimer;



void setup() {
  //setting pins to US sensor and fans
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(Lfan, OUTPUT);
  pinMode(Tfan, OUTPUT);

  //setting servo and IMU
  Serial.begin(9600);
  Wire.begin();
  myservo.attach(9);

  byte status = mpu.begin();
  while (status != 0) { } // stop if could not connect to MPU6050

  _delay_ms(1000);
  mpu.calcOffsets(true, true); // gyro and accelerometer offset calc.
  

  //default servo to 90 and set forward to current.
  myservo.write(90);
  currentYaw=getYaw(); 
  forwardYaw = currentYaw;
 
  delay(500);
  accelTimer=millis();
}


void loop() {

//starting fans 
if(firstLoop){

digitalWrite(Lfan,HIGH);
delay(500);
analogWrite(Tfan,255);

firstLoop=false;

}

  
  digitalWrite(Lfan,HIGH); //keeping lift fan on 

  currentYaw=getYaw();    //get the pointing direction.
  
  // forward yaw - current yaw is the different from (-90 to currentyaw to +90 )
  servoAngle = forwardYaw - currentYaw + 90;  //every loop will update the servo angle according to yaw changes
 
 //keeping it in servo range
 if(servoAngle < 0){
    servoAngle = 0;
  }
  if(servoAngle > 180){
    servoAngle = 180;
  }

 
myservo.write(servoAngle);  //setting direction every loop


 //if hovercraft is doing a turn greater than 60 degrees full speed thrust
  if(abs(forwardYaw-currentYaw)>60){

    analogWrite(Tfan,255);
    if(turningWall()&& (millis()-sweepTimer)>3000){   //avoid checking for walls for 3 sec from starting turn
      hcStop();
    }
      



  }else if(abs(forwardYaw-currentYaw)<15){  //reduced thrust speed for straights

    analogWrite(Tfan,200);

    if(incomingWall()&& (millis()-sweepTimer)>3000){ //avoid checking for walls for 3 sec from starting turn
       hcStop();
    }
     



  }else{    //thrust speed is lowest between 60 degree and 15 degree turns
      analogWrite(Tfan,180);
      if(turningWall()&& (millis()-sweepTimer)>3000){  //avoid checking for walls for 3 sec from starting turn
      hcStop();
    }

  }
    
 if(millis()-accelTimer>9000)  {   //checks accelerometers after 9 seconds of starting fans to check if the hovercraft is stuck not moving forward
 isHcStopped();
 
 }


}


// stop and calculate the next desired direction to turn into
void hcStop(){
    
    digitalWrite(Lfan,LOW);
    digitalWrite(Tfan,LOW);
    delay(1000);
    forwardYaw = forwardYaw + sweep() - 90;           //angle of servo to go in longest direction
    accelTimer=millis();
    sweepTimer=millis();
    firstLoop=true;
  }




//get the current yaw value
float getYaw(){
  mpu.update();
  yaw =-mpu.getAngleZ();
  return (yaw);  
} 


//sweeping servo motor from 0 to 180 degrees checking for longest distance every 90 degrees
//if the longest distance is less than 100 it means its a U turn so we return a -90 or 270 degree turn
int sweep(){
  bool firstDelay =true;
  int Theta;
  double tempDist;
  double longestDist = 0;
  for (int i=0; i<181; i+=90)
  {
   
    myservo.write(i);
     if(firstDelay){    //adding delay for first servo movement to 0degrees
      delay(300);
      firstDelay=false;
    }
    delay(500);  // delay for accurate readings
    tempDist = USdist();  //get distance
    
    if( tempDist > longestDist){  //comparing present value with longest distance and setting them accordingly
      longestDist = tempDist;  
      Theta = i;
     }
    delay(100);

  }

 //checking for U turn
  if(longestDist<100){
        if(Theta < 90 ){
          Theta = -90;
        }

        if(Theta > 90){
          Theta = 270;
        }
  }else{         //case of no U turn
          if(Theta < 90 ){
          Theta = 0;
        }

        if(Theta > 90){
          Theta = 180;
        }
        
    
  }

 
  return Theta;
}


//checking if acceleration magnitude average is less than 0.5 meaning we're at a constant speed, 
//works after 9sec of starting to avoid stopping HC too soon if we reach a steady speed 
void isHcStopped(){
   mpu.update();  // Update IMU data
  
  float aX = mpu.getAccX();
  float aY = mpu.getAccY();

  float tA = sqrt(sq(aX)+sq(aY));
  // Calculate avg acceleration

  float avgA= (prevtA + tA)/2;

  // Check if the acceleration is below the threshold
  if (avgA < ACCELERATION_THRESHOLD) {   //acceleration threshold is 0.5
    
    digitalWrite(Lfan,LOW);  //stopping and starting fans before returning to main loop
    analogWrite(Tfan,0);
    delay(800);
 
    digitalWrite(Lfan,HIGH);
    delay(400);
    analogWrite(Tfan,150);
    int stuckTimer=millis();

   //keep going in same direction for 1.5 seconds
    while(millis()-stuckTimer<1500){
         analogWrite(Tfan,255);
         currentYaw=getYaw();    //get the pointing direction.
  
          servoAngle = forwardYaw - currentYaw + 90;  //every loop will update the servo angle according to yaw  **check +-
        if(servoAngle < 0){
            servoAngle = 0;
          }
          if(servoAngle > 180){
            servoAngle = 180;
          }
        
        myservo.write(servoAngle);  //setting direction every loop

    }
      
   
     
  }
   
    prevtA = tA;
    accelTimer=millis();

}


//check for distance smaller than 18 when turning
bool turningWall(){

  if(USdist()<18){
      return true;
    }
    return false;
}


//check for distance smaller than 30 when going straight
bool incomingWall(){

  if(USdist()<30){
    return true;
  }
  return false;

}


//get distance from Ultrasonic Sensor
int USdist(){

  digitalWrite(trigPin, LOW); //clear trig
  delayMicroseconds(2); // Wait for 2 microseconds
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // pulse for 10 microseconds
  
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);	//read input echo
  
  distance= duration*0.034/2;	//distance calc using 0.034cm/microsec
 
  return distance;
}
