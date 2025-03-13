#include <Servo.h>

// Motor Pins
#define MOTOR_IN1_PIN 5
#define MOTOR_IN2_PIN 4
#define MOTOR_IN3_PIN 3
#define MOTOR_IN4_PIN 6


// Ultrasonic Sensor Pins
#define ULTRASONIC_TRIGGER_PIN 10
#define ULTRASONIC_ECHO_PIN 11

// Servo Motor Pin
#define SERVO_PIN 9

// Servo Angles
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSE_ANGLE 0

// IR Sensor Pins
#define IR_RIGHT_PIN A1
#define IR_LEFT_PIN A0

// Distance Sensor Pin
#define DISTANCE_SENSOR_PIN 5

// Motor Speeds
#define MOTOR_SPEED_A 180 // Maximum speed for motor A
#define MOTOR_SPEED_B 180 // Maximum speed for motor B
#define MOTOR_LEFT_ROTATION_SPEED 190 // Maximum speed for left motor rotation
#define MOTOR_RIGHT_ROTATION_SPEED 190 // Minimum speed for right motor rotation

// Global Variables
int distanceThreshold = 1;
int distanceFront;
Servo gripperServo;

// Function Prototypes

void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMoving();
long readDistance();
void closeGripper();
void openGripper();
void setup();
void loop();

void moveForward() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, HIGH);

}

void moveBackward() {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, HIGH);
    digitalWrite(MOTOR_IN4_PIN, LOW);
}

void turnRight() {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);

}

void turnLeft() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, HIGH);

}

void stopMoving() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);

}

long readDistance() {
    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2000);
    digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    long distanceCm = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    return distanceCm / 29 / 2;
}

void closeGripper() {
    gripperServo.write(SERVO_CLOSE_ANGLE);
}

void openGripper() {
    gripperServo.write(SERVO_OPEN_ANGLE);
}

void setup() {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(MOTOR_IN3_PIN, OUTPUT);
    pinMode(MOTOR_IN4_PIN, OUTPUT);
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
    pinMode(DISTANCE_SENSOR_PIN, INPUT);

    gripperServo.attach(SERVO_PIN);
}


void loop() {

    distanceFront = readDistance();
    delay(500); // Introducing a delay for sensor stability

    int leftSensor = digitalRead(IR_LEFT_PIN);
    int rightSensor = digitalRead(IR_RIGHT_PIN);

    if (rightSensor == 0 && leftSensor == 1) {
        if (distanceFront > distanceThreshold) {
            moveForward();       
        }
        else {
            stopMoving();
            delay(500); // Introducing a delay for gripper stability
            closeGripper();
        }
    } else if (rightSensor == 0 && leftSensor == 1) {
        turnRight();
    
    } else if (rightSensor == 1 && leftSensor == 0) {
        turnLeft();
      
    } else if (rightSensor == 1 && leftSensor == 1) {
        stopMoving();
        delay(500);
        openGripper();
    }
}