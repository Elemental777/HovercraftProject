#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <Servo.h>

#include "Wire.h"
#include <MPU6050_light.h>

#define YAW_THRESHOLD 90.0
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

#define PWM_PIN 11 // Pin 11 (D3) is PD7 for PWM control

MPU6050 mpu(Wire);
unsigned long timer = 0;
float yaw;
Servo myservo;
int pwmValue;
float xAcceleration; 

void setup() {
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

  

//myservo.write(map(yaw, -180, 180, 180, 0)); //THIS MIGHT MAKE THE SERVO TURN THE CORRECT DIRECTION
  map(yaw, 180, -180, 180, 0); // Modify servo angle as needed
  myservo.write(yaw + 90); // Update the servo angle
  timer = millis();
}
