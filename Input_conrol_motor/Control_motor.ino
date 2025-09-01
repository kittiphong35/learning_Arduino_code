#include <ESP32Servo.h>

Servo myservo;
int user_input = 0;
bool normalMode = false; // ตัวแปรสถานะโหมด

void setup() {
  Serial.begin(9600);
  myservo.attach(22); // กำหนดขา 22
  myservo.write(0);   // เริ่มที่ 0°
  Serial.println("Enter angle (0-180) or >180 for normal mode:");
}

void loop() {
  // ถ้ามี input เข้ามาทาง Serial
  if (Serial.available() > 0) {
    int select_mode = Serial.parseInt();
    user_input = select_mode;

    if (user_input >= 0 && user_input <= 180) {
      normalMode = false;  // ออกจากโหมด normal
      myservo.write(user_input);
      Serial.print("Servo angle set to: ");
      Serial.println(user_input);
    }
    else if (user_input > 180) {
      Serial.println("Normal mode activated!");
      normalMode = true;   // เข้าสู่โหมด normal
      
    }
    else {
      normalMode = false;
      Serial.println("Invalid input, servo stopped at 0");
      myservo.write(0);
    }
  }

  // ถ้าอยู่ใน normal mode → วน servo อัตโนมัติ
  if (normalMode) {
    normal();
  }
}



void normal(){
  myservo.write(0);
  delay(1000);


  myservo.write(45);
  delay(1000);

  myservo.write(90);
  delay(1000);

  myservo.write(135);
  delay(1000);

  myservo.write(180);
  delay(1000);
}