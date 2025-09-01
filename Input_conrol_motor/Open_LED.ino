// กำหนดขาที่ต่อกับ LED
#define LED_BUTLIN 2

void setup() {
  // ตั้งค่า LED_BUTLIN เป็น OUTPUT
  pinMode(LED_BUTLIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUTLIN, HIGH);   // เปิดไฟ LED
  delay(500);                  // รอ 1 วินาที
  digitalWrite(LED_BUTLIN, LOW);    // ปิดไฟ LED
  delay(500);                  // รอ 1 วินาที
}
