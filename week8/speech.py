import re
import speech_recognition as sr
import sys
import os

r = sr.Recognizer()
with sr.Microphone() as mic:
    r.adjust_for_ambient_noise(mic, duration=0.6)  # กรองเสียง 0.6
    print("พูดคำสั่ง เช่น 'สวัสดี หมุนมอเตอร์ ไปที่ 90 องศา' หรือพูด 'หยุด' เพื่อออกจากโปรแกรม")

    while True:
        try:
            audio = r.listen(mic, timeout=5, phrase_time_limit=10)
            text = r.recognize_google(audio, language='th-TH').strip()
            print(f"ได้ยิน : {text}")

            #บันทึกลงไฟล์ rec.txt
            with open("rec.txt", "a", encoding="utf-8") as f:
                f.write(text + "\n")

            #ตรวจสอบว่าผู้ใช้พูด "หยุด"
            if "หยุด" in text:
                print("หยุดการทำงาน และบันทึกข้อความเรียบร้อย")
                sys.exit(0)

            #กรณีที่ขึ้นต้นด้วย "สวัสดี"
            if text.startswith("สวัสดี"):
                prompt = text.replace("สวัสดี", " ", 1).strip()
                m = re.search(r"ไปที่ (\d+) องศา", prompt)
                if m:
                    angle = int(m.group(1))
                    print("สวัสดี : ไปที่", prompt, "| มุม :", angle)
                else:
                    print("สวัสดี :", prompt)

        except sr.WaitTimeoutError:
            print("ไม่มีเสียงพูดมาเลยครับ กรุณาลองใหม่อีกครั้ง")
        except sr.UnknownValueError:
            print("ไม่สามารถรู้จำเสียงได้ครับ")
