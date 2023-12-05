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
int servoAngle;

//lift fan
//const int Lfan = 5; //p3 
const int Lfan=4; //DC pd4

//thrust fan
const int Tfan = 6; //p4


//MPU global variables
MPU6050 mpu(Wire);
unsigned long timer = 0;
float forwardYaw;
float currentYaw;
float yaw;

// Constants
const float ACCELERATION_THRESHOLD = 0.5;  // Adjust as needed
const int STOPPED_DURATION = 6000;         // Time in milliseconds to consider it stopped

// Variables
unsigned long accelTimer = 0;  //timer in isHcStopped()
int FanOnTime=2000;  //for delays
int FanOffTime=2000;  //for delays
float prevtA=3;

// //boolean logic
 bool firstLoop = true; //first loop bool to start fans
// bool hcStop =false;    //if hc stopped 
bool sweepCheck=false;


//US global variables
long duration;
int distance;
long sweepTime;



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

  //default servo to current 0  set forward to current.
  myservo.write(90);
  currentYaw=getYaw(); 
  forwardYaw = currentYaw;
  
  

  delay(500);
 accelTimer=millis();
}


void loop() {

if(firstLoop){

digitalWrite(Lfan,HIGH);
delay(500);
analogWrite(Tfan,255);

firstLoop=false;

}


  digitalWrite(Lfan,HIGH);

  currentYaw=getYaw();    //get the pointing direction.
  
  // forward yaw - current yaw is the different from (-90 to currentyaw to +90 )
  // +90 to convert to 0 to 180 degree.  90 is the currentYaw point. 
  
  servoAngle = forwardYaw - currentYaw + 90;  //every loop will update the servo angle according to yaw  **check +-
  //servoAngle = constrain(servoAngle,0,180);    //keeping it in servo range
 if(servoAngle < 0){
    servoAngle = 0;
    
  }
  if(servoAngle > 180){
    servoAngle = 180;
  }
 
myservo.write(servoAngle);  //setting direction every loop

  if(abs(forwardYaw-currentYaw)>60){

    analogWrite(Tfan,255);
    if(turningWall())
      hcStop();



  }else if(abs(forwardYaw-currentYaw)<15){

    analogWrite(Tfan,200);

    if(incomingWall()){
       hcStop();
    }
     



  }else{
      analogWrite(Tfan,180);
      if(turningWall()){
      hcStop();
    }

  }
    
  
//   if(abs(forwardYaw-currentYaw)>50)
//   {
//     //delay(FanOnTime);
//     analogWrite(Tfan,230);

//     if(turningWall())
//       hcStop();
//   }
//   else if (abs(forwardYaw-currentYaw)>20)
//   {
//     //analogWrite(Lfan,255);
//     //delay(FanOnTime);  
//     analogWrite(Tfan,180);  //then thrust fan turns on once skirt is inflated 
//     if(turningWall())
//       hcStop();
//   }
//   else{

// analogWrite(Tfan, min(255,100+USdist())); 
// if(incomingWall()){           //to fix 
//         hcStop();   
//         }


//   }
    //analogWrite(Lfan, min(255,240+USdist()));
    // analogWrite(Tfan, min(255,100+USdist())); 
    //delay(200);  
    // if((abs(forwardYaw-currentYaw)<5)){
    //    if(incomingWall()){           //to fix 
    //     hcStop();   
    //     }
    // }
    
  // if(incomingWall()){           //to fix 
  //       hcStop();   
  //       }


  //if(){
        
    // digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    // //delay(FanOnTime);  
    // analogWrite(Tfan,100);  //then thrust fan turns on once skirt is inflated
       
  // }
   //else{
  //   digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
  //   //delay(FanOnTime);  
  //   analogWrite(Tfan,255);  //then thrust fan turns on once skirt is inflated
  // }
 
// if((millis() - accelTimer)> STOPPED_DURATION)
//   isHcStopped();

//debug info
// Serial.print("yaw: ");
// Serial.println(yaw);
// Serial.print("forwardYaw: ");
// Serial.println(forwardYaw);
// Serial.print("currentYaw: ");
// Serial.println(currentYaw);

}


// stop and calaulate the next forwardYaw from (-90 current +90)
void hcStop(){
    
    digitalWrite(Lfan,LOW);
    // delay(100);
    digitalWrite(Tfan,LOW);
    delay(1000);
    forwardYaw = forwardYaw + sweep() - 90;           //angle of servo to go in longest direction
    accelTimer=millis();
    firstLoop=true;
  }




//get the current yaw?
float getYaw(){
  mpu.update();
  yaw =-mpu.getAngleZ();
 // map(yaw,-180,180,0,180); 
  return (yaw);  //check if difference is good
} 


// do the sweep to find the next direction. return a angle that is -90 to +90 degree.
int sweep(){
  bool firstDelay =true;
  int Theta = 90;
  double tempDist = -10;
  double longestDist = -10;
  for (int i=0; i<181; i+=90)
  {
   
    myservo.write(i);
     if(firstDelay){    //adding delay for first servo movement to 0degrees
      delay(300);
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
    delay(100);

  }

// sweepTime = millis();              //timer for checkwalltimer idea
//  sweepCheck=true;                   //bool for checkwalltimer idea
  // return Theta - 90;
  
  if(longestDist<100){
    if(Theta < 90 ){
      Theta = 0;
    }

    if(Theta > 90){
      Theta = 180;
    }
  }else{
      if(Theta < 90 ){
      Theta = -90;
    }

    if(Theta > 90){
      Theta = 270;
    }
    
  }


  return Theta;
}


//this works
void isHcStopped(){
   mpu.update();  // Update IMU data
  
  float aX = mpu.getAccX();
  float aY = mpu.getAccY();

  float tA = sqrt(sq(aX)+sq(aY));
  // Calculate avg acceleration

  float avgA= (prevtA + tA)/2;

  // Check if the acceleration is below the threshold
  if (avgA < ACCELERATION_THRESHOLD) {   //acceleration threshold is 0
    
     digitalWrite(Lfan,LOW);
    analogWrite(Tfan,0);
    delay(100);
    digitalWrite(Lfan,HIGH);
     delay(1300);
    // forwardYaw=getYaw();
    // firstLoop=true;
    prevtA = tA;
    // hcStop();  
    accelTimer=millis();         
    return;
     
  }
   
    prevtA = tA;


}


bool turningWall(){

if(USdist()<15){
    return true;
  }
  return false;


}

bool incomingWall(){

  if(USdist()<45){
    return true;
  }
  return false;

}



int USdist(){

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
