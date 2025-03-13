#include <Servo.h>
#include "robot_car_config.h"
#include "robot_car_functions.h"
#include "robot_car_pid.h"
#include "robot_object_detection.h"

Servo servoMotor;

void setup() {
  setupMotors();
  servoMotor.attach(SERVO_PIN);
  initializePID();
}

void loop() {
  // Read sensor values
  int ultrasonicDistance = readUltrasonicDistance();
  int rightIRSensorValue = readIRSensor(IR_RIGHT_PIN);
  int leftIRSensorValue = readIRSensor(IR_LEFT_PIN);

  // Line-following logic
  int lineError = rightIRSensorValue - leftIRSensorValue;
  int correction = calculatePID(lineError);
  int leftSpeed = 100 - correction;
  int rightSpeed = 100 + correction;

  // Object detection and pick-up
  if (detectObject()) {
    pickUpObject();
  }

  // Control motors
  controlMotors(leftSpeed, rightSpeed);
}
