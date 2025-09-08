// hearder
#include <ESP32Servo.h> 

Servo myServo; //ตั้งชื่อตัวแปรให้กับ Servo
char status = 'c';

void setup() {
  Serial.begin(9600);
  myServo.attach(32); // เชื่อมต่อservoที่ขา D32
}

void loop() {

  if (Serial.available() > 0) {
    char status_read = Serial.read();
    if(status_read != status){
      status = status_read;
    }
  }

  if(status == 'c'){
    myServo.write(0);
    printf("Close The Door\n");
  }else if(status == 'o'){
    myServo.write(90);
    printf("Open The Door\n");
  }


}
