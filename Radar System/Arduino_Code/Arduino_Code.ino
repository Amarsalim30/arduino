#include <Servo.h>
Servo myservo;

#define echo 13     //Echo pin
#define trigger 12  //Trigger pin

#define G_led 3  // choose the pin for the Green Led
#define R_led 4  // choose the pin for the Red Led

int distance;
int degree;

void setup() {  // put your setup code here, to run once
  Serial.begin(9600);

  myservo.attach(A2);  // declare Servo Motor as output
  myservo.write(90);

  pinMode(echo, INPUT);      // declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT);  // declare ultrasonic sensor Trigger pin as Output

  pinMode(R_led, OUTPUT);  // declare Red LED as output
  pinMode(G_led, OUTPUT);  // declare Green LED as output

  delay(1000);
}


void loop() {

  for (degree = 1; degree < 180; degree += 1) {
    myservo.write(degree);
    delay(100);
    data();
  }

  for (degree = 180; degree > 1; degree -= 1) {
    myservo.write(degree);
    data();
    delay(100);
  }
}

void data() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn(echo, HIGH);
  distance = time / 28.5 / 2;

  if (distance > 150) { distance = 150; }

  if (distance > 100) {
    digitalWrite(G_led, HIGH);  // LED Turn On
    digitalWrite(R_led, LOW);   // LED Turn Off
  } else {
    digitalWrite(G_led, LOW);   // LED Turn Off
    digitalWrite(R_led, HIGH);  // LED Turn On
  }
  Serial.println("degree");
  Serial.println(degree);
  Serial.println(",");
  Serial.println("distance");
  Serial.println(distance);
  Serial.print("\r");
}
