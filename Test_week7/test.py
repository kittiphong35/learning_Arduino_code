import cv2
import face_recognition
import numpy as np
import serial
import mediapipe as mp
import math
import time
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap


class ArduinoControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arduino Controller")
        self.setGeometry(100, 100, 500, 500)

        # Label แสดงข้อความ
        self.label_info = QLabel("เลือกฟังก์ชัน...")
        self.label_info.setStyleSheet("font-size:20px; color:blue;")
        
        # Label แสดงวิดีโอ
        self.video_label = QLabel()
        self.video_label.setFixedSize(500, 500)

        # ปุ่มซ้าย-ขวา
        self.btn_left = QPushButton("Face Scan")
        self.btn_left.clicked.connect(lambda: self.select_function("face"))
        self.btn_right = QPushButton("Hand Axis")
        self.btn_right.clicked.connect(lambda: self.select_function("hand"))

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.btn_left)
        button_layout.addWidget(self.btn_right)

        # Layout หลัก
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.label_info)
        main_layout.addWidget(self.video_label)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

        # serial Arduino
        self.arduino = serial.Serial('COM5', 9600, timeout=1)
        time.sleep(2)

        # ตัวแปรควบคุมฟังก์ชัน
        self.current_function = None
        self.cap = cv2.VideoCapture(0)

        # QTimer สำหรับอัปเดตวิดีโอ
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # ประมาณ 30 fps

        # เตรียม face recognition
        self.authorized_images = [("KITTIPHONG CHAIMON", "face-001.jpg")]
        self.known_face_encodings = []
        self.known_face_names = []
        for name, file in self.authorized_images:
            img = face_recognition.load_image_file(file)
            encodings = face_recognition.face_encodings(img)
            if len(encodings) > 0:
                self.known_face_encodings.append(encodings[0])
                self.known_face_names.append(name)

        # Mediapipe hand
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.ema_angle = None

    def select_function(self, func):
        self.current_function = func
        self.label_info.setText(f"ฟังก์ชันที่เลือก: {func}")

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if self.current_function == "face":
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, tolerance=0.5)
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if not matches[best_match_index] or face_distances[best_match_index] > 0.5:
                    name = "Unknown"
                    color = (0,0,255)
                    self.arduino.write(b'c')
                else:
                    name = self.known_face_names[best_match_index]
                    color = (0,255,0)
                    self.arduino.write(b'o')
                cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                cv2.putText(frame, name, (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        elif self.current_function == "hand":
            results = self.hands.process(rgb_frame)
            h, w, c = frame.shape
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    thumb_tip = hand_landmarks.landmark[2]
                    index_tip = hand_landmarks.landmark[8]
                    wrist = hand_landmarks.landmark[0]
                    x1, y1 = int(thumb_tip.x * w), int(thumb_tip.y * h)
                    x2, y2 = int(index_tip.x * w), int(index_tip.y * h)
                    x0, y0 = int(wrist.x * w), int(wrist.y * h)
                    v1 = [x1-x0, y1-y0]
                    v2 = [x2-x0, y2-y0]
                    dot = v1[0]*v2[0] + v1[1]*v2[1]
                    mag1 = math.sqrt(v1[0]**2+v1[1]**2)
                    mag2 = math.sqrt(v2[0]**2+v2[1]**2)
                    angle = math.degrees(math.acos(dot/(mag1*mag2))) if mag1*mag2 != 0 else 0
                    self.ema_angle = angle if self.ema_angle is None else 0.2*angle + 0.8*self.ema_angle
                    servo_deg = int(max(0,min(180,round(self.ema_angle*2))))
                    self.arduino.write(f"{servo_deg}\n".encode())
                    cv2.putText(frame,f"Angle:{int(angle)} deg",(50,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
                    cv2.circle(frame,(x1,y1),8,(255,0,0),-1)
                    cv2.circle(frame,(x2,y2),8,(0,0,255),-1)

        # แปลง frame เป็น QImage
        img = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888).rgbSwapped()
        self.video_label.setPixmap(QPixmap.fromImage(img))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ArduinoControl()
    window.show()
    sys.exit(app.exec_())
