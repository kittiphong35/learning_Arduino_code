import cv2                 # ไลบรารี OpenCV สำหรับประมวลผลภาพและวิดีโอ
import math                # ไลบรารี math สำหรับการคำนวณทางคณิตศาสตร์ (เช่น มุม, cos, acos)
import numpy as np         # ไลบรารี NumPy สำหรับการจัดการ array และการคำนวณเชิงตัวเลข
from ultralytics import YOLO  # ไลบรารี YOLO ของ ultralytics สำหรับโหลดโมเดลตรวจจับ/pose estimation
from PIL import Image, ImageDraw, ImageFont   # ใช้ Pillow สำหรับการวาดข้อความภาษาไทย
import serial # ไลบรารี servo
import time


# ---------------- SERIAL CONFIG ----------------
# ตั้งค่า Serial Port ไปยัง Arduino
arduino = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)

# ---------------- CONFIGURATION ----------------
VIDEO_PATH = "congratulation.mp4"    # พาธของวิดีโอที่จะเปิด
MODEL_PATH = "yolo11n-pose.pt"       # พาธของโมเดล YOLO pose (ตรวจจับท่าทาง)


# กำหนด ROI (Region of Interest) = พื้นที่สนใจในเฟรม (สัดส่วน 0..1)
ROI_LEFT, ROI_RIGHT  = 0.35, 0.55   # แกน X ซ้าย→ขวา
ROI_TOP,  ROI_BOTTOM = 0.20, 0.98   # แกน Y บน→ล่าง

# กำหนดค่ามุมที่ยอมรับ (องศา) ว่าถือว่าท่าทางเหมาะสม
RANGE_ARM  = (160, 185)   # ช่วงมุมแขน
RANGE_BACK = (165, 185)   # ช่วงมุมหลัง
RANGE_KNEE = (160, 185)   # ช่วงมุมเข่า

# สีสำหรับการแสดงผล (BGR)
C_SKEL = (30, 200, 255)   # สีเส้นโครงร่าง skeleton
C_OK   = (0, 220, 0)      # สีเขียว = ผ่าน
C_BAD  = (0, 0, 255)      # สีแดง = ไม่ผ่าน
C_INFO = (255, 255, 255)  # สีขาว = ข้อความ


# รองรับข้อความภาษาไทย (optional)
USE_THAI = True
FONT_PATH = "NotoSansThai-Regular.ttf"  # ไฟล์ฟอนต์ภาษาไทย
FONT_SIZE = 26

try:
  THAI_FONT = ImageFont.truetype(FONT_PATH, FONT_SIZE) if USE_THAI else None
except Exception:
  THAI_FONT = None   # ถ้าโหลดฟอนต์ไม่ได้ จะไม่ใช้ภาษาไทย


# ---------------- ฟังก์ชันวาดข้อความ ----------------
def put_text(img, text_th, text_en, org, color=C_INFO, size=FONT_SIZE):
  """วาดข้อความภาษาไทยถ้ามีฟอนต์, ถ้าไม่มีใช้ภาษาอังกฤษ"""
  text = text_th if THAI_FONT else text_en
  if THAI_FONT:   # ถ้าโหลดฟอนต์ไทยได้
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # แปลงจาก BGR → RGB
    pil = Image.fromarray(rgb)                   # แปลงเป็นภาพแบบ Pillow
    draw = ImageDraw.Draw(pil)                   # สร้างอ็อบเจ็กต์วาด
    b,g,r = color                               # เปลี่ยนสีจาก BGR → RGB
    draw.text(org, text, font=THAI_FONT.font_variant(size=size), fill=(r,g,b))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)  # แปลงกลับเป็น OpenCV (BGR)
  else:   # ถ้าไม่มีฟอนต์ไทย → ใช้ cv2.putText ภาษาอังกฤษ
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return img


# ---------------- ฟังก์ชันช่วยคำนวณเรขาคณิต ----------------
def angle(a, b, c):
  """คำนวณมุม ABC (องศา)"""
  if a is None or b is None or c is None: return None
  ax,ay=a; bx,by=b; cx,cy=c
  ba=(ax-bx, ay-by); bc=(cx-bx, cy-by)   # เวกเตอร์ BA และ BC
  nba, nbc = math.hypot(*ba), math.hypot(*bc)   # ขนาดเวกเตอร์
  if nba==0 or nbc==0: return None
  cosv = max(-1.0, min(1.0, (ba[0]*bc[0]+ba[1]*bc[1])/(nba*nbc)))  # cosine similarity
  return math.degrees(math.acos(cosv))   # แปลงเป็นองศา


def kp_list(kp, conf_th=0.25):
  """แปลง keypoints จาก (17,3) → list[(x,y) หรือ None] ตาม threshold"""
  return [(int(x),int(y)) if c>conf_th else None for x,y,c in kp]


# โครงสร้าง skeleton (COCO17 connections)
CONNS = [(5,7),(7,9),(6,8),(8,10),(5,6),(5,11),(6,12),
        (11,12),(11,13),(13,15),(12,14),(14,16)]


# ---------------- MAIN FUNCTION ----------------
def main():
  model = YOLO(MODEL_PATH)              # โหลดโมเดล YOLO pose
  cap = cv2.VideoCapture(VIDEO_PATH)    # เปิดวิดีโอ
  if not cap.isOpened():                # ถ้าเปิดไม่ได้
    print("Cannot open video:", VIDEO_PATH); return

  face_count = 0


  while True:    # วนลูปอ่านเฟรมทีละภาพ
    ok, frame = cap.read()
    if not ok: break   # ถ้าอ่านไม่ได้ = จบวิดีโอ
    H,W,_ = frame.shape

    # คำนวณ ROI เป็นพิกเซล
    x1, x2 = int(W*ROI_LEFT),  int(W*ROI_RIGHT)
    y1, y2 = int(H*ROI_TOP),   int(H*ROI_BOTTOM)
    roi = frame[y1:y2, x1:x2]   # ตัด ROI ออกมา

    # ทำให้บริเวณนอก ROI มืดลง
    overlay = frame.copy()
    cv2.rectangle(overlay, (0,0), (W,H), (0,0,0), -1)   # ทาสีดำทั้งภาพ
    overlay[y1:y2, x1:x2] = frame[y1:y2, x1:x2]         # แทนที่ ROI ด้วยภาพจริง
    frame = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0)  # ผสม overlay+frame

    # ใช้โมเดลทำนายเฉพาะ ROI
    r = model(roi, verbose=False, conf=0.25, iou=0.5)[0]

    # ถ้าไม่มีผลลัพธ์ (ไม่เจอคน/โครงร่าง)
    if r.keypoints is None or len(r.keypoints)==0 or r.boxes is None or len(r.boxes)==0:
      arduino.write(b'c\n')
      frame = put_text(frame, "ไม่มีคนในโซน", "No person detected", (x1+10, y1+30), C_BAD)
      cv2.imshow("Focus Pose @ Receiving", frame)
      if cv2.waitKey(1) & 0xFF == 27: break
      continue

    # ดึง keypoints และ boxes
    kps   = r.keypoints.data.cpu().numpy()   # [N,17,3]
    boxes = r.boxes.xyxy.cpu().numpy()       # [N,4]
    roi_h = roi.shape[0]

    cands=[]
    for i,(xA,yA,xB,yB) in enumerate(boxes):
      h_box = yB - yA
      cx    = (xA + xB) / 2.0
      cands.append((i, cx, h_box))

      # เลือก box ที่สูงพอ
      valid = [t for t in cands if t[2] >= 0.35*roi_h]
      pool  = valid if valid else cands
      if not pool:
        frame = put_text(frame, "ไม่พบโครงร่าง", "No keypoints", (x1+10, y1+30), C_INFO)
        cv2.imshow("Focus Pose @ Receiving", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
        continue

      # เลือก target (box ที่อยู่ซ้ายสุด)
      target_idx = min(pool, key=lambda t: t[1])[0]
      kp = kps[target_idx]

      # แปลง keypoints ให้อยู่ในพิกัดเต็มเฟรม
      pts = kp_list(kp, conf_th=0.20)
      pts = [(p[0]+x1, p[1]+y1) if p else None for p in pts]

      # วาด skeleton
      for a,b in CONNS:
        if pts[a] and pts[b]:
          cv2.line(frame, pts[a], pts[b], C_SKEL, 1)  # ← ความหนาเส้น = 1px
      for p in pts:
        if p: cv2.circle(frame, p, 2, C_SKEL, -1) # ← จุดข้อต่อ รัศมี = 2px

      # index ข้อต่อต่าง ๆ (ฝั่งขวา)
      R_sh, R_el, R_wr = 6,8,10   # ไหล่, ศอก, ข้อมือ
      R_hp, R_kn, R_an = 12,14,16 # สะโพก, เข่า, ข้อเท้า

      # จุดเขียวค้างที่ข้อเท้า
      R_an = 16
      if pts[R_an]:
        cv2.circle(frame, pts[R_an], 9, (0,255,0), -1)

      # คำนวณมุม
      arm  = angle(pts[R_sh], pts[R_el], pts[R_wr])     # มุมแขน
      back = angle(pts[R_sh], pts[R_hp], pts[R_kn])     # มุมหลัง
      knee = angle(pts[R_hp], pts[R_kn], pts[R_an])     # มุมเข่า

      # คาดคะเนปลายเท้า
      toe = None
      if pts[R_an]:
        ax,ay = pts[R_an]
        if pts[R_kn]:
          kx,ky = pts[R_kn]
          alpha=0.30
          toe = (int(ax + alpha*(ax-kx)), int(ay + alpha*(ay-ky)))
        else:
          toe = (ax,ay)
          cv2.circle(frame, toe, 6, (0,255,0), -1)

      # ฟังก์ชันตรวจว่ามุมอยู่ในช่วงหรือไม่
      def ok_rng(val, rng):
        return val is not None and (rng[0] <= val <= rng[1])

      # เริ่มพิกัดจากขอบขวาล่างของเฟรม
      xtxt = W - 150   # ขยับเข้ามาจากขอบขวา (250 px)
      ytxt = H - 120   # ขยับขึ้นจากขอบล่าง (120 px)

    # Replace with your name
      my_name_th = "นาย กิตติพงษ์ ใจมน"
      my_name_en = "Kittipong Chaimon"
      x_txt = 10   # ขยับเข้ามาจากขอบซ้าย (10 px)
      y_txt = H - 50   # ขยับลงมาจากขอบล่าง (100 px)

      frame = put_text(frame, my_name_th, my_name_en, (x_txt, y_txt), (0, 0, 0))



      # แสดงผลค่ามุมต่าง ๆ พร้อมสี
      if arm is not None:
        frame = put_text(frame, f"แขน: {int(arm)}°", f"Arm: {int(arm)}°",
                        (xtxt, ytxt), C_OK if ok_rng(arm,RANGE_ARM) else C_BAD)
        ytxt += 28
        if back is not None:
          frame = put_text(frame, f"หลัง: {int(back)}°", f"Back: {int(back)}°",
                          (xtxt, ytxt), C_OK if ok_rng(back,RANGE_BACK) else C_BAD)
          ytxt += 28
        if knee is not None:
          frame = put_text(frame, f"เข่า: {int(knee)}°", f"Knee: {int(knee)}°",
                          (xtxt, ytxt), C_OK if ok_rng(knee,RANGE_KNEE) else C_BAD)

        # ---------------- เงื่อนไขสั่ง Servo ----------------
        if ok_rng(arm, RANGE_ARM) and ok_rng(back, RANGE_BACK) and ok_rng(knee, RANGE_KNEE):
          # วาดวงกลมสีเขียวที่มุมขวาบน
          cv2.circle(frame, (W - 30, 30), 10, (0, 255, 0), -1)  # (x,y), radius=15, สีเขียว
          arduino.write(b'o\n')  # ทุกมุมผ่าน → Servo เปิด
        else:
          arduino.write(b'c\n')  # ไม่ผ่าน → Servo ปิด


      cv2.imshow("Focus Pose @ Receiving", frame)   # แสดงผลเฟรม
      if cv2.waitKey(1) & 0xFF == 27: break   # กด Esc เพื่อออก
  cap.release()
  cv2.destroyAllWindows()


# ---------------- RUN MAIN ----------------
if __name__ == "__main__":
  main()   # เริ่มรันโปรแกรม
