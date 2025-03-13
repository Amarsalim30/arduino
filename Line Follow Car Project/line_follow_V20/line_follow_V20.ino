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
#define IR_RIGHT_PIN A0
#define IR_LEFT_PIN A1

// Motor Speeds
#define MOTOR_SPEED_A 200 // Maximum speed for motor A
#define MOTOR_SPEED_B 200 // Maximum speed for motor B
#define MOTOR_LEFT_ROTATION_SPEED 180 // Maximum speed for left motor rotation
#define MOTOR_RIGHT_ROTATION_SPEED 180 // Maximum speed for right motor rotation

// PID Constants
#define KP 0.05 // Proportional constant
#define KI 0.1 // Integral constant
#define KD 0.1 // Derivative constant

// Global Variables
int distanceThreshold = 3;
int distanceFront;
Servo gripperServo;
unsigned long previousTime = 0;
float previousError = 0;
float integral = 0;
float dt = 0; // Time difference in seconds

// Function Prototypes
void setup();
void loop();
void turnRight();
void turnLeft();
void stopMoving();
long readDistance();
void closeGripper();
void openGripper();
void controlSpeed(int leftSpeed, int rightSpeed);

void setup() {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(MOTOR_IN3_PIN, OUTPUT);
    pinMode(MOTOR_IN4_PIN, OUTPUT);
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);

    gripperServo.attach(SERVO_PIN);
}

void loop() {
    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0; // Time difference in seconds
    previousTime = currentTime;

    distanceFront = readDistance();
    delay(500); // Introducing a delay for sensor stability

    int leftSensor = digitalRead(IR_LEFT_PIN);
    int rightSensor = digitalRead(IR_RIGHT_PIN);

    if (rightSensor == 0 && leftSensor == 0) {
        if (distanceFront > distanceThreshold) {
            controlSpeed(MOTOR_SPEED_A, MOTOR_SPEED_B);
        } else {
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
        openGripper();
    }
}

void turnRight() {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, HIGH);
}

void turnLeft() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    digitalWrite(MOTOR_IN3_PIN, HIGH);
    digitalWrite(MOTOR_IN4_PIN, LOW);
}

void stopMoving() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);
}

long readDistance() {
    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
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

void controlSpeed(int leftSpeed, int rightSpeed) {
    float error = leftSpeed - rightSpeed;
    integral += error;
    float derivative = (error - previousError) / dt;
    previousError = error;

    int leftPWM = leftSpeed + (KP * error) + (KI * integral) + (KD * derivative);
    int rightPWM = rightSpeed - (KP * error) - (KI * integral) - (KD * derivative);

    analogWrite(MOTOR_ENA_PIN, leftPWM);
    analogWrite(MOTOR_ENB_PIN, rightPWM);
}
