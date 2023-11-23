#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servoMotor;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();

  servoMotor.attach(11); // Attach the servo to pin 11
}

void loop() {
  int16_t gx, gy, gz;

  mpu.getRotation(&gx, &gy, &gz);

  // Map the gyroscope reading to the servo range
  int servoPosition = map(gy, -32768, 32767, 0, 180);

  // Constrain the servo position to prevent issues
  servoPosition = constrain(servoPosition, 0, 180);

  // Set the servo position
  servoMotor.write(servoPosition);

  // Print the data to Serial Monitor
  Serial.print("Yaw (gy): ");
  Serial.print(gy);
  Serial.print("    Servo Position: ");
  Serial.println(servoPosition);

  delay(50); // Adjust the delay as needed
}
