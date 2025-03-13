#include <Servo.h>

// Define motor pins for left and right motors
#define LEFT_MOTOR_FORWARD_PIN 5
#define LEFT_MOTOR_BACKWARD_PIN 3
#define RIGHT_MOTOR_FORWARD_PIN 6
#define RIGHT_MOTOR_BACKWARD_PIN 4

// Define pins for ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN 10
#define ULTRASONIC_ECHO_PIN 9

// Define pin for servo motor
#define SERVO_PIN 8

// Define servo angles
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSE_ANGLE 0

// Define pins for IR sensors
#define IR_RIGHT_PIN A0
#define IR_LEFT_PIN A1

// Define maximum speeds for left and right motors
#define MAX_LEFT_SPEED 100
#define MAX_RIGHT_SPEED 100

// Define PID constants
#define KP 0.05
#define KI 0.1
#define KD 0.1

// Define minimum distance threshold to obstacle
#define MIN_DISTANCE_TO_OBSTACLE 3

// Define global variables
int distanceFront;
Servo gripperServo;
unsigned long previousTime = 0;
float previousError = 0;
float integral = 0;
float dt = 0;

// Function prototypes
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
    // Initialize serial communication for debugging
    Serial.begin(9600);
    
    // Initialize pin modes
    pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);

    // Attach servo to pin
    gripperServo.attach(SERVO_PIN);
}

void loop() {
    // Calculate time difference
    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Read front distance from ultrasonic sensor
    distanceFront = readDistance();
    delay(500); // Introduce delay for sensor stability
    Serial.print("Distance: ");
    Serial.println(distanceFront);
    // Read IR sensor inputs
    int leftSensor = digitalRead(IR_LEFT_PIN);
    int rightSensor = digitalRead(IR_RIGHT_PIN);
    Serial.print("Right_IR: ");
    Serial.println(rightSensor);
    Serial.print("Left_IR: ");
    Serial.println(leftSensor);

    controlSpeed(MAX_LEFT_SPEED, MAX_RIGHT_SPEED);
    // Perform obstacle avoidance and gripping actions  
    if (rightSensor == 0 && leftSensor == 0) {
        if (distanceFront > MIN_DISTANCE_TO_OBSTACLE) {
            controlSpeed(MAX_LEFT_SPEED, MAX_RIGHT_SPEED);
        } else {
            stopMoving();
            delay(1000); // Introduce delay for gripper stability
            closeGripper();
        }
    } else if (rightSensor == 0 && leftSensor == 1) {
        turnLeft();
    } else if (rightSensor == 1 && leftSensor == 0) {
        turnRight();
    } else if (rightSensor == 1 && leftSensor == 1) {
        stopMoving();
        delay(1000); // Introduce delay for gripper stability
        openGripper();
    }
}

void turnLeft() {
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void turnRight() {
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void stopMoving() {
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

long readDistance() {
    // Send trigger pulse to ultrasonic sensor
    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    
    // Measure pulse duration
    long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    
    // Convert duration to distance
    return duration / 29 / 2;
}

void closeGripper() {
    gripperServo.write(SERVO_CLOSE_ANGLE);
}

void openGripper() {
    gripperServo.write(SERVO_OPEN_ANGLE);
}

void controlSpeed(int leftSpeed, int rightSpeed) {
    // PID control for motor speeds
    float error = leftSpeed - rightSpeed;
    integral += error;
    float derivative = (error - previousError) / dt;
    previousError = error;

    // Adjust PWM signals based on PID control
    int leftPWM = leftSpeed + (KP * error) + (KI * integral) + (KD * derivative);
    int rightPWM = rightSpeed - (KP * error) - (KI * integral) - (KD * derivative);

    // Set motor speeds
    analogWrite(LEFT_MOTOR_FORWARD_PIN, leftPWM);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, rightPWM);
}
