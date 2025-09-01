#include <ESP32Servo.h>

Servo myServo;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(32);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String angel = Serial.readStringUntil('\n');
    angel.trim();
    // Serial.print(angel);
    if (angel == "C") {
      myServo.write(0);
      Serial.print("0");
    } else if (angel == "10") {
      myServo.write(10);
      Serial.print("10");
    } else if (angel == "20") {
      myServo.write(20);
      Serial.print("20");
    } else if (angel == "30") {
      myServo.write(30);
      Serial.print("30");
    } else if (angel == "40") {
      myServo.write(40);
      Serial.print("40");
    } else if (angel == "50") {
      myServo.write(50);
      Serial.print("50");
    } else if (angel == "60") {
      myServo.write(60);
      Serial.print("60");
    } else if (angel == "70") {
      myServo.write(70);
      Serial.print("70");
    } else if (angel == "80") {
      myServo.write(80);
      Serial.print("80");
    } else if (angel == "90") {
      myServo.write(90);
      Serial.print("90");
    }
  }


}