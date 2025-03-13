// File: robot_car_functions.h

#include "robot_car_config.h"

// Function prototypes
void setupMotors();
void controlMotors(int leftSpeed, int rightSpeed);
void stopMotors();
int readUltrasonicDistance();
int readIRSensor(int sensorPin);
void adjustServoAngle(int angle);
