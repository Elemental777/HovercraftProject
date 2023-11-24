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


//MPU global variables
MPU6050 mpu(Wire);
unsigned long timer = 0;
float yaw;
Servo myservo;
int pwmValue;
float xAcceleration; 

//US global variables
long duration;
double distance;




void setup() {
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
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

  
}

void loop() {
  mpu.update();

  
 

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
}


//idea
/* int getYaw(){
  
} 
*/


int sweep(){
int tempTheta = 0;
  double tempDist = 0;
  double longestDist = 0;
  for (int i=0; i<181; i+=2){
  myservo.write(i);
  delay(40);  // delays and degrees to be fine tuned
    tempDist = USdist();
    Serial.print("Servo angle:");
    Serial.println(i);
    delay(10);
    if( tempDist > longestDist){
      longestDist = tempDist;  
      tempTheta = i;   //check with yaw value 
    }
    
  }
  return tempTheta;
  
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
