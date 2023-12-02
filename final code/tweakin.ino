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

int angleDiff;

int stopCount=0;
// Constants
const float ACCELERATION_THRESHOLD = 1;  // Adjust as needed
const int STOPPED_DURATION = 5000;         // Time in milliseconds to consider it stopped

// Variables
unsigned long accelTimer = 0;  //timer in isHcStopped()
int FanOnTime=2000;  //for delays
int FanOffTime=2000;  //for delays

// //boolean logic
bool firstLoop = true; //first loop bool to start fans
// bool hcStop =false;    //if hc stopped 
bool sweepCheck=false;

//sweep variables
  bool firstDelay;
  int Theta;
  double tempDist;
  double longestDist;
  double sumDist;
  int count;
  bool sweepLR=false;

  

//US global variables
long duration;
double distance;
//long accelTimer;

int TurnInArow=0; //counting 90 degree turns in a row

int adjustSpeed;

float prevtA = 3;





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



if(firstLoop){

    digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
    //delay(1000);  
    analogWrite(Tfan,255); 
    delay(500);
    firstLoop=false;
    accelTimer=millis();    // important to reset timer for isHcStopped()
  }
  



  currentYaw=getYaw();    //get the pointing direction.
  
  
  // forward yaw - current yaw is the different from (-90 to currentyaw to +90 )
  // +90 to convert to 0 to 180 degree.  90 is the currentYaw point. 

  angleDiff = forwardYaw - currentYaw;  //playing around with setting a variable angleDiff so it doesnt do fyaw-cyaw every time

  servoAngle = angleDiff +90;

  //servoAngle = forwardYaw - currentYaw +90;  //every loop will update the servo angle according to yaw  **check +-
 // servoAngle = constrain(servoAngle,0,180);    //keeping it in servo range



 //constraining
  if(servoAngle>180)
    servoAngle = 180;
  if(servoAngle<0)
    servoAngle=0;
 
  myservo.write(servoAngle);  //setting direction every loop
  




  if(abs(angleDiff)>70){
      digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
      //delay(FanOnTime);
      analogWrite(Tfan,255);      //adjusting turning speed WORKING ON
    
  }
  else if (abs(angleDiff)>45){
      digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
      //delay(FanOnTime);  
      analogWrite(Tfan,220);  //adjusting turning speed WORKING ON play with this
      
  

  }
  else{
            digitalWrite(Lfan,HIGH);  //Lift Fan turns on first
            //delay(FanOnTime); 
          

          //these lines below were to test full speed thrust

          // adjustSpeed = min(USdist(),255);          
            // if(accelTimer - millis()<4000){
            //   analogWrite(Tfan,255);

            // }else
          // analogWrite(Tfan,min(USdist(),255));    //variable Tfan
            analogWrite(Tfan,255); 

              if(incomingWall()){        

                    
                          //call stop 
                          hcStop();   
                        }

  }


  
   
   isHcStopped(); //works well, needs acceltimers 
  
   
  
 


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
    
    myservo.write(90);          //this is for checking forward dist for sweepLR
    
    analogWrite(Lfan,150);
    analogWrite(Tfan,50);
    delay(200);
    
    if(USdist()<10)                     //sets LR
      sweepLR=true;
   
    forwardYaw = currentYaw + sweep() -90;           //angle of servo to go in longest direction
   
    delay(50);
    
    firstLoop=true;
    sweepLR=false;                    //thismakes the servo only check LR
    
    accelTimer=millis();      //to reset timer for isHcStopped()
  }







//good dont change
int getYaw(){
  mpu.update();
  yaw =-mpu.getAngleZ();
 // map(yaw,-180,180,0,180); 
 
  return (yaw);  
} 






//sweeping function work in progress  
int sweep(){
  firstDelay =true;
  longestDist = 0;
  tempDist =0;
  sumDist = 0;
  count = 0;

    //if true it means forward distance is less than 10 SEE HCSTOP for setting
    if(sweepLR){    

        myservo.write(0);
        delay(800);

        longestDist = USdist();

        myservo.write(180);
        delay(800);

        tempDist=USdist();

          if(tempDist>longestDist){
            longestDist=tempDist;
            Theta=180;
          }
          else
          Theta=0;


      if(longestDist<25)    //test changing dist threshold for going straight instead of L/R
        Theta=90;
    }
    else{                   //no sweepLR means normal 30 degrees sweep
      
      
      for (int i=0; i<181; i+=30){
      
        myservo.write(i);
        
        if(firstDelay){    //adding delay for first servo movement to 0degrees
          delay(800);
          firstDelay=false;
        }
        
        delay(100);  // delays and degrees to be fine tuned 300 delay works
        tempDist = USdist();
        
        // Serial.print("Servo angle:");
        // Serial.println(i);
        
        sumDist += tempDist;
        ++count;
        if( tempDist > longestDist){
          longestDist = tempDist;  
          Theta = i;
            
        }
        delay(100);     //delay before and after usdist()
        
      }
    }


        //this was working i just commented it out and added the next if statement

      //could make this check in the check90Turn()
      // if(longestDist<1500){
      //   if(Theta<31 ){
      //     if(check90Turn()){
      //     return 0;
      //   }
      //   TurnInArow=0;
      //      return 90;
      //   }
      //   if(Theta> 149){
      //     if(check90Turn()){
      //     return 180;
      //   }  TurnInArow=0;
      //      return 90;

      //   }
      // }
    
    //added this instead of ^
    if(Theta == 0 || Theta ==180){
      if(check90Turn()){
      return Theta;

      }else{
        TurnInArow=0;
        return 90;

      }

    }

    //in case all distances are rly similar
    if(abs(longestDist - avgDist(sumDist,count))<5){
      TurnInArow=0;
      return 90;
    }

          
          //final return
          
          delay(100);
          TurnInArow=0;
          return Theta;



    }



//calculating avg in sweep()
double avgDist(double sum, int c){

return sum/c;

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
  if (avgA < ACCELERATION_THRESHOLD && (millis() - accelTimer)> STOPPED_DURATION) {   //acceleration threshold is 0
    
    // analogWrite(Lfan,0);
    // analogWrite(Tfan,50);
    // delay(100);
    // forwardYaw=getYaw();
    firstLoop=true;
    prevtA = tA;
    hcStop();           
    return;
     
  }
   
    prevtA = tA;


}



//checks for no more than 2 consecutive 90 turns, TO BE CHECKED might not need
bool check90Turn(){

  if(TurnInArow>1){
    return false;
    }
    else{
    TurnInArow++;
    return true;

  }
    
}


//
bool incomingWall(){

  if(USdist()<45){      //play around with this distance
    
    return true;
  }
  return false;
}



//works perfect

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
