import cv2  # ใช้ OpenCV สำหรับเปิดกล้องและวาดภาพ
import mediapipe as mp  # ใช้ Mediapipe Pose สำหรับตรวจจับตำแหน่งร่างกาย
import math  # ใช้สำหรับคำนวณทางคณิตศาสตร์ เช่น มุม
import serial  # ใช้สำหรับสื่อสารกับบอร์ด (ESP32/Arduino) ผ่าน Serial Port
import time  # ใช้หน่วงเวลา



# ====== CONFIG ======
COM_PORT = 'COM5'   # พอร์ตที่เชื่อมต่อกับ ESP32/Arduino (ต้องเปลี่ยนตามเครื่อง)
BAUDRATE = 9600     # ความเร็วในการสื่อสาร Serial
EMA_ALPHA = 0.2     # ค่าถ่วงน้ำหนักของ EMA smoothing (ลดความสั่นของค่า)
SHOW_LINES = True   # กำหนดว่าจะโชว์เส้นระหว่างข้อศอก-ข้อมือไหม

# ==== SERIAL =====
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)  # รอให้ Serial พร้อมก่อนเริ่มส่งข้อมูล

# ==== MEDIAPOPE ช่วยวาดรูปออกมา====
mp_drawing = mp.solutions.drawing_utils   # เอาไว้ช่วยวาด skeleton ลงบนภาพ
mp_pose = mp.solutions.pose               # ใช้โมดูล pose detection ของ mediapipe

def to_vec(a, b):
  # สร้าง vector จากจุด a ไปยังจุด b (ใช้หาทิศทางของแขน)
  return (b[0]-a[0], b[1]-a[1])

def angle_between(u, v):
  # คำนวณมุมระหว่าง vector u และ v
  ux, uy = u; vx, vy = v
  du = math.hypot(ux, uy)  # ความยาวเวกเตอร์ u
  dv = math.hypot(vx, vy)  # ความยาวเวกเตอร์ v
  if du == 0 or dv == 0:   # ถ้าเวกเตอร์ยาวเป็นศูนย์ ให้ข้าม
    return None
  cos_t = (ux*vx + uy*vy)/(du*dv)  # หาค่า cosine ของมุม
  cos_t = max(-1.0, min(1.0, cos_t))  # ป้องกันค่าเกิน [-1,1]
  return math.degrees(math.acos(cos_t))  # คืนค่าเป็นมุม (degree)



cap = cv2.VideoCapture(0)  # เปิดกล้อง webcam
ema_angle = None           # เก็บค่า EMA smoothing ครั้งแรก

with mp_pose.Pose(model_complexity=1, #ใช้โมเดลความละเอียดกลาง
                  enable_segmentation=False,
                  min_detection_confidence=0.6,#ขั้นต่ำในการตรวจจับ
                  min_tracking_confidence=0.6) as pose: #ขั้นต่ำในการติดตาม

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)  # พลิกภาพซ้าย-ขวา (เหมือนกระจก)
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # แปลง BGR → RGB
        res = pose.process(rgb) # ประมวลผล pose detection

        angle_0_90 = None

        angle_0_90 = None
        if res.pose_landmarks:
            lm = res.pose_landmarks.landmark
            # Get 2D points (pixel coords)
            def p(id):
                return (int(lm[id].x*w), int(lm[id].y*h))  # แปลงเป็นพิกัด pixel

            L_ELBOW, L_WRIST = p(mp_pose.PoseLandmark.LEFT_ELBOW), p(mp_pose.PoseLandmark.LEFT_WRIST)
            R_ELBOW, R_WRIST = p(mp_pose.PoseLandmark.RIGHT_ELBOW), p(mp_pose.PoseLandmark.RIGHT_WRIST)

            # Forearm vectors: elbow -> wrist (camera plane)
            # สร้าง vector ท่อนแขนซ้ายและขวา (ข้อศอก → ข้อมือ)
            vL = to_vec(L_ELBOW, L_WRIST)
            vR = to_vec(R_ELBOW, R_WRIST)

        # หามุมระหว่างแขนซ้าย-ขวา
        theta = angle_between(vL, vR)  # degrees 0..180 (in-plane)
        if theta is not None:
          # จำกัดมุมให้อยู่ระหว่าง 0–90 องศา
          theta = min(theta, 180 - theta)
          angle_0_90 = max(0, min(90, theta))

          # EMA smoothing
          if ema_angle is None:
            ema_angle = angle_0_90
          else:
            ema_angle = EMA_ALPHA*angle_0_90 + (1-EMA_ALPHA)*ema_angle #กำหนดน้ำหนัก (0.2 = smoothing เยอะ → ค่าจะนิ่งขึ้นแต่ตอบสนองช้า)

          # Map to servo 0..180
          servo_deg = int(max(0, min(180, round(ema_angle*2))))

          ser.write(f"{servo_deg}\n".encode()) # ส่งค่ามุม servo ไป ESP32

          # Draw UI
          cv2.putText(frame, f"Angle: {angle_0_90:.1f} deg  |  Servo: {servo_deg}",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        if SHOW_LINES:
          cv2.circle(frame, L_ELBOW, 6, (0,255,255), -1)
          cv2.circle(frame, R_ELBOW, 6, (0,255,255), -1)
          cv2.circle(frame, L_WRIST, 6, (255,0,255), -1)
          cv2.circle(frame, R_WRIST, 6, (255,0,255), -1)
          cv2.line(frame, L_ELBOW, L_WRIST, (0,255,255), 3)
          cv2.line(frame, R_ELBOW, R_WRIST, (0,255,255), 3)
        mp_drawing.draw_landmarks(frame, res.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        cv2.imshow("Two-Arm Angle → Servo", frame)
        if cv2.waitKey(1) & 0xFF == 27:  #  กด ESC เพื่อออก
          break

