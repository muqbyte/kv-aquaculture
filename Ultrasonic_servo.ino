#include <ESP32Servo.h>

#define TRIG_PIN 12 //connect trig pin from ultrasonic to esp32 D12
#define ECHO_PIN 14 //connect echo pin from ultrasonic to esp32 D14
#define SERVO_PIN 13 //connect servo pin to esp32 D13

Servo myservo; // create servo object to control a servo

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  myservo.attach(SERVO_PIN); // attaches the servo on D13 pin to the servo object
}

void loop() {
  // Read distance from ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);

  // Control servo motor based on distance
  if (distance <= 30) {
    myservo.write(90); // set servo position to 90 degrees
  } else if (distance > 30) {
    myservo.write(0); // set servo position to 0 degrees
  }
  
  delay(500); // wait for half a second before taking another measurement
}