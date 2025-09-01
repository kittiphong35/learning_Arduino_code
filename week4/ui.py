import sys
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox


try :
    arduino = serial.Serial('COM5', 9600, timeout=1)
except Exception as e:
    arduino = None
    print("เชื่อมต่อไม่ได้ :", e)



class ArduinoControl(QWidget):
  def __init__(self):
    super().__init__()

    self.setWindowTitle("Arduino Controller ") #ชื่อโปรแกรมที่ปรากฏบน title
    self.setGeometry(200, 200, 300, 150)  #ขนาด

    layout = QVBoxLayout()
    # Close
    self.btn_on = QPushButton("Close/Reset")
    self.btn_on.clicked.connect(lambda:self.send_command("C"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("10")
    self.btn_on.clicked.connect(lambda:self.send_command("10"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("20")
    self.btn_on.clicked.connect(lambda:self.send_command("20"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("30")
    self.btn_on.clicked.connect(lambda:self.send_command("30"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("40")
    self.btn_on.clicked.connect(lambda:self.send_command("40"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("50")
    self.btn_on.clicked.connect(lambda:self.send_command("50"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("60")
    self.btn_on.clicked.connect(lambda:self.send_command("60"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("70")
    self.btn_on.clicked.connect(lambda:self.send_command("70"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("80")
    self.btn_on.clicked.connect(lambda:self.send_command("80"))
    layout.addWidget(self.btn_on)

    self.btn_on = QPushButton("90")
    self.btn_on.clicked.connect(lambda:self.send_command("90"))
    layout.addWidget(self.btn_on)


    self.setLayout(layout)


  def send_command(self, command):
    if(arduino):
      arduino.write((command + '\n').encode())
    else:
      QMessageBox.critical(self, "Error", "ไม่พบการเชื่อมต่อ Arduino")












#การสร้าง GUI output
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ArduinoControl()
    window.show()
    sys.exit(app.exec_())

