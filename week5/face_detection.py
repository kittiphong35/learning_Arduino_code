import face_recognition
import cv2
import serial
import time
import numpy as np


# ตั้งค่า Serial Port ไปยัง Arduino (เช็ค COM port ของคุณ)
arduino = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)  # รอให้ Arduino พร้อม

# -------------------------------
# โหลดภาพตัวอย่างผู้ที่ได้รับอนุญาต
# -------------------------------
authorized_images = [
    ("KITTIPHONG CHAIMON","face-001.jpg"),
    #("pannapat_dungbupha", "faces/pannapat/pannapat.png"),
    #("Chealsea","faces/chanwit/36ad4e86-b336-4168-805a-cbfe3756bf8a.jpg")
]

known_face_encodings = []
known_face_names = []

for name, filename in authorized_images:
    img = face_recognition.load_image_file(filename)
    encodings = face_recognition.face_encodings(img)
    if len(encodings) > 0:
        known_face_encodings.append(encodings[0])
        known_face_names.append(name)

print("โหลดผู้มีสิทธิ์แล้ว:", known_face_names)

# -------------------------------
# เปิดกล้อง
# -------------------------------
video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()
    if not ret:
        continue

    # แปลง BGR → RGB และบังคับ dtype เป็น uint8
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb_frame = np.array(rgb_frame, dtype=np.uint8)

    # หาตำแหน่งใบหน้า
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # เปรียบเทียบกับทุกคนที่อนุญาต
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.5)
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)

        if  not matches[best_match_index] or face_distances[best_match_index] > 0.5:
            print("❌ Unknown face! Door Locked")
            arduino.write(b'c')   # ส่ง C ไป Arduino (ล็อก)
            name = "Unknown"
        elif matches[best_match_index]:
            name = known_face_names[best_match_index]
            print(f"✅ Face recognized: {name} | Door Open")
            arduino.write(b'o')   # ส่ง O ไป Arduino (เปิด)
            color = (0, 255, 0)   # เขียว = ผ่าน
        else:
            print("❌ Unknown face! Door Locked")
            arduino.write(b'c')   # ส่ง C ไป Arduino (ล็อก)
            name = "Unknown"
            color = (0, 0, 255)   # แดง = ไม่ผ่าน

        # วาดกรอบใบหน้า + ชื่อ
        cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
        cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    # แสดงภาพจากกล้อง
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

video_capture.release()
cv2.destroyAllWindows()
