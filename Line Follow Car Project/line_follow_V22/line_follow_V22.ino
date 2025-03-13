#include <Servo.h>
const int ENA = 3;
const int ENB = 11;

// Pin assignments for motor control
const int LEFT_MOTOR_FORWARD_PIN = 5;
const int LEFT_MOTOR_BACKWARD_PIN = 12;
const int RIGHT_MOTOR_FORWARD_PIN = 6;
const int RIGHT_MOTOR_BACKWARD_PIN = 4;

// Define pins for IR sensors
const int IR_RIGHT_PIN = A0;
const int IR_LEFT_PIN = A1;

// Pin assignments for ultrasonic sensor
const int ULTRASONIC_TRIGGER_PIN = 10;
const int ULTRASONIC_ECHO_PIN = 9;

// Pin assignment for servo motor
const int SERVO_PIN = 8;

// Servo angle constants
const int SERVO_OPEN_ANGLE = 90;
const int SERVO_CLOSE_ANGLE = 0;

// Maximum motor speeds
const int MAX_LEFT_SPEED = 250;
const int MAX_RIGHT_SPEED = 250;

// PID control constants
const float KP = 0.05;
const float KI = 0.1;
const float KD = 0.1;

// Minimum distance threshold for obstacle detection
const int MIN_DISTANCE_TO_OBSTACLE = 5;

// Global variables
int distanceFront;
Servo gripperServo;
unsigned long previousTime = 0;
float previousError = 0;
float integral = 0;
float dt = 0;

// PID-controlled motor PWM values
int leftPWM = 0;
int rightPWM = 0;

// Global variables for distance tracking and movement control
int initialDistance = 0;
int targetDistance = 50; // Desired distance to move forward (in cm)


// Function prototypes
void setup();
void loop();
void moveForwardFixedDistance();
void stopMoving();
long readDistance();
void openGripper();
void controlSpeed();

void setup() {
    Serial.begin(9600); // Initialize serial communication
    
    // Motor pin configurations
    pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);

    // Ultrasonic sensor pin configurations
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    // Servo motor pin configuration
    gripperServo.attach(SERVO_PIN);
}

void loop() {
    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    distanceFront = readDistance();
    delay(500);

    
        if (!movingForward) {
            initialDistance = distanceFront;
            movingForward = true;
       yy
        moveForwardFixedDistance();
    } else {
        stopMoving();
        delay(1000);
        openGripper();
    }
}

void moveForwardFixedDistance() {
    int currentDistance = distanceFront;
    int distanceMoved = initialDistance - currentDistance;
    if (distanceMoved >= targetDistance) {
        stopMoving();
        movingForward = false;
    } else {
        moveForward();
    }
}

void moveForward() {
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);

    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);
}

void stopMoving() {
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

long readDistance() {
    digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    
    long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    return duration / 29 / 2;
}

void openGripper() {
    gripperServo.write(SERVO_OPEN_ANGLE);
}

void controlSpeed() {
    int leftSpeed = MAX_LEFT_SPEED;
    int rightSpeed = MAX_RIGHT_SPEED;

    float error = leftSpeed - rightSpeed;
    integral += error;
    float derivative = (error - previousError) / dt;
    previousError = error;

    leftPWM = leftSpeed + (KP * error) + (KI * integral) + (KD * derivative);
    rightPWM = rightSpeed - (KP * error) - (KI * integral) - (KD * derivative);

    Serial.println("left SPEED:");
    Serial.println(leftPWM);
    Serial.println("right SPEED:");
    Serial.println(rightPWM);
}
