// hearder
#include <ESP32Servo.h> 

Servo myServo; //ตั้งชื่อตัวแปรให้กับ Servo


void setup() {
  Serial.begin(9600);
  myServo.attach(32); // เชื่อมต่อservoที่ขา D32
}

void loop() {

  if (Serial.available() > 0) {
    int angel = Serial.parseInt(); //ส่งค่าไปยังPython
    if(angel >= 0 && angel <= 90){
      myServo.write(angel);
    }
  }

}
