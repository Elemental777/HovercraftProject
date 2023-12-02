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
const float ACCELERATION_THRESHOLD = 0.3;  // Adjust as needed
const int STOPPED_DURATION = 5000;         // Time in milliseconds to consider it stopped

// Variables
unsigned long stopStartTime = 0;  //timer in isHcStopped()
int FanOnTime=2000;  //for delays
int FanOffTime=2000;  //for delays

// //boolean logic
bool firstLoop = true; //first loop bool to start fans
// bool hcStop =false;    //if hc stopped 
bool sweepCheck=false;


//US global variables
long duration;
int distance;
long sweepTime;

int TurnInArow=0;


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
  myservo.write(currentYaw+90);
  forwardYaw = currentYaw;
  

  delay(1000);
 
}

void loop() {


  mpu.update();
  currentYaw=getYaw();    //get the pointing direction.
  
  
  if(firstLoop){

    digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    delay(1000);  
    analogWrite(Tfan,255); 
    firstLoop=false;
  }
  
    
  // forward yaw - current yaw is the different from (-90 to currentyaw to +90 )
  // +90 to convert to 0 to 180 degree.  90 is the currentYaw point. 
  
  servoAngle = forwardYaw - currentYaw +90;  //every loop will update the servo angle according to yaw  **check +-
  //servoAngle = constrain(servoAngle,0,180);    //keeping it in servo range
 if(servoAngle <0){
    servoAngle=0;
    
  }
  if(servoAngle>180){
    servoAngle=180;
  }
 
  myservo.write(servoAngle);  //setting direction every loop
  




  if(abs(forwardYaw - currentYaw)>70){
    digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    //delay(FanOnTime);
    analogWrite(Tfan,250);
    
  }
  else if (abs(forwardYaw - currentYaw)>45){
    digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    //delay(FanOnTime);  
    analogWrite(Tfan,200);  //then thrust fan turns on once skirt is inflated
    
  

  }
  else{

    digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    //delay(FanOnTime);  
    //analogWrite(Tfan,180); 
    analogWrite(Tfan,min(USdist(),255));                      //change this maybe to fix straight line

      if(incomingWall()){           //to fix

            
                  //call stop 
                  hcStop();   
                }


   
  }


  
   
  
   
  
 


//debug info
Serial.print("yaw: ");
Serial.println(yaw);
Serial.print("forwardYaw: ");
Serial.println(forwardYaw);
Serial.print("currentYaw: ");
Serial.println(currentYaw);

}

// stop and calaulate the next forwardYaw from (-90 current +90)
void hcStop(){
    

    analogWrite(Lfan,0);
    digitalWrite(Tfan,LOW);
    delay(FanOffTime);
    forwardYaw = currentYaw + sweep() -90;           //angle of servo to go in longest direction
    firstLoop=true;
    
  }






//get the current yaw?
int getYaw(){
  yaw =-mpu.getAngleZ();
 // map(yaw,-180,180,0,180); 
 
  return (yaw);  //check if difference is good
} 


// do the sweep to find the next direction. return a angle that is -90 to +90 degree.
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
  delay(400);  // delays and degrees to be fine tuned 300 delay works
    tempDist = USdist();
  
    Serial.print("Servo angle:");
    Serial.println(i);
    
    if( tempDist > longestDist){
      longestDist = tempDist;  
      Theta = i;
        //check with yaw value 
    }
    
  }

// sweepTime = millis();              //timer for checkwalltimer idea
//  sweepCheck=true;                   //bool for checkwalltimer idea
  // return Theta - 90;
  
  
  if(longestDist<1500){
    if(Theta<75 ){
      Theta = 0;
    }
    if(Theta> 105){
      Theta = 180;
    }
  }
  // if(Theta==0 || Theta ==180){
  //   if(check90Turn()){
  //     return Theta;
  //   }
  //   return 90;

  // }
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


bool check90Turn(){

  if(TurnInArow>1){
    TurnInArow=0;
    return false;
    }
    else{
    TurnInArow++;
    return true;

  }
    
}


bool incomingWall(){

  if(USdist()<40){
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
