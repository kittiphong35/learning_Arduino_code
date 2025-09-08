import cv2
import mediapipe as mp
import math
import time
import serial

# ====== CONFIG ======
COM_PORT = 'COM5'   # พอร์ตที่เชื่อมต่อกับ ESP32/Arduino (ต้องเปลี่ยนตามเครื่อง)
BAUDRATE = 9600     # ความเร็วในการสื่อสาร Serial
EMA_ALPHA = 0.2     # ค่าถ่วงน้ำหนักของ EMA smoothing (ลดความสั่นของค่า)
SHOW_LINES = True   # กำหนดว่าจะโชว์เส้นระหว่างข้อศอก-ข้อมือไหม

# ==== SERIAL =====
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)# เปิดพอร์ต Serial
time.sleep(2)  # รอให้ Serial พร้อมก่อนเริ่มส่งข้อมูล


# สร้าง object ของ Mediapipe สำหรับตรวจจับมือ
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# เปิดกล้อง
cap = cv2.VideoCapture(0)
ema_angle = None  # ตัวแปรสำหรับเก็บค่า EMA smoothing

# ใช้ Mediapipe Hands
with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
    while cap.isOpened():
        ret, frame = cap.read()# อ่านภาพจากกล้อง
        if not ret:
            break

        frame = cv2.flip(frame, 1)  # กลับภาพเหมือนกระจก
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # แปลง BGR → RGB
        results = hands.process(rgb) # ประมวลผลภาพหามือ

        if results.multi_hand_landmarks: # ถ้าพบมือ
            for hand_landmarks in results.multi_hand_landmarks:
                # วาดโครงร่างมือบนเฟรม
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # ดึงขนาดภาพ (สูง, กว้าง, ช่องสี)
                h, w, c = frame.shape

                # จุด landmark: นิ้วโป้ง, นิ้วชี้, ข้อมือ
                thumb_tip = hand_landmarks.landmark[2]   # นิ้วโป้ง
                index_tip = hand_landmarks.landmark[8]   # นิ้วชี้
                wrist = hand_landmarks.landmark[0]       # ข้อมือ

                # แปลงพิกัดจาก normalized (0-1) → พิกัด pixel จริง
                x1, y1 = int(thumb_tip.x * w), int(thumb_tip.y * h)
                x2, y2 = int(index_tip.x * w), int(index_tip.y * h)
                x0, y0 = int(wrist.x * w), int(wrist.y * h)

                # สร้างเวกเตอร์จากข้อมือไปยังนิ้วโป้ง และ ข้อมือไปยังนิ้วชี้
                v1 = [x1 - x0, y1 - y0]
                v2 = [x2 - x0, y2 - y0]

                # คำนวณ dot product
                dot = v1[0]*v2[0] + v1[1]*v2[1]

                # ความยาวเวกเตอร์
                mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
                mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

                # คำนวณมุมระหว่างเวกเตอร์ (องศา)
                if mag1*mag2 != 0:
                    angle = math.degrees(math.acos(dot / (mag1 * mag2)))
                else:
                    angle = 0

                # ===== EMA smoothing เพื่อลดการสั่นของมุม =====
                if ema_angle is None:
                    ema_angle = angle
                else:
                    ema_angle = EMA_ALPHA*angle + (1-EMA_ALPHA)*ema_angle

                # ===== แปลงค่ามุมเป็นองศาของ servo =====
                # ตัวอย่าง: มุม 0–90 → map ไปที่ servo 0–180
                servo_deg = int(max(0, min(180, round(ema_angle*2))))
                ser.write(f"{servo_deg}\n".encode()) # ส่งค่าไปที่ ESP32/Arduino ผ่าน Serial


                # แสดงผลบนจอ
                cv2.putText(frame, f"Angle: {int(angle)} deg", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # วาดจุดที่นิ้วโป้งและนิ้วชี้
                cv2.circle(frame, (x1, y1), 8, (255, 0, 0), -1)
                cv2.circle(frame, (x2, y2), 8, (0, 0, 255), -1)

        # แสดงภาพบนหน้าต่าง
        cv2.imshow("Hand Angle Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # กด ESC เพื่อออก
            break

# ปิดกล้องและหน้าต่าง
cap.release()
cv2.destroyAllWindows()