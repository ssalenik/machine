#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PySide.QtCore import *
from PySide.QtGui import *

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Command Centre - McGill - Eng Games 2013")

        centre = CentralWidget() 
        self.setCentralWidget(centre)

class CentralWidget(QWidget):

    def __init__(self, parent=None):
        super(CentralWidget, self).__init__(parent)

        # status
        self.connected = False
        
        # widgets which will be contained in the central widget
        self.settings = Settings()
        self.chassy = ChassyFrame()

        # layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(0)
        self.layout.addWidget(self.settings)
        self.layout.addWidget(self.chassy)

        self.setLayout(self.layout)
        

class Settings(QFrame):

    def __init__(self, parent=None):
        super(Settings, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)

        # connection status
        self.connected = False
        
        # widgets
        self.label = QLabel("Settings")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.portLabel = QLabel("Port: ")
        self.portSelect = QComboBox()
        self.portSelect.addItems(["port a", "port b", "port c"])
        self.connect = QPushButton("connect")
        
        # flashing connection status button
        self.statusLabel = QLabel(self)
        self.redFill = QPixmap(20,20)
        self.redFill.fill(Qt.red)
        self.greenFill = QPixmap(20,20)
        self.greenFill.fill(Qt.green)
        self.statusLabel.setPixmap(self.redFill)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0)
        self.layout.addWidget(self.hLine, 1, 0, 1, 5)
        self.layout.addWidget(self.portLabel, 2, 0)
        self.layout.addWidget(self.portSelect, 2, 1)
        self.layout.addWidget(self.connect, 2, 3)
        self.layout.addWidget(self.statusLabel, 2, 4)

        # make middle column stretch
        self.layout.setColumnStretch(2, 1)

        self.setLayout(self.layout)

class ChassyFrame(QFrame):

    def __init__(self, parent=None):
        super(ChassyFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)

        # widgets
        self.label = QLabel("Chassy")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.forward = QPushButton("forward")
        self.forward.setMinimumWidth(110)
        self.back = QPushButton("backward")
        self.back.setMinimumWidth(110)
        self.ccw = QPushButton("CCW")
        self.ccw.setMinimumWidth(110)
        self.cw = QPushButton("CW")
        self.cw.setMinimumWidth(110)
        self.PIDLabel = QLabel("PID:")
        self.PIDLabel.setAlignment(Qt.AlignRight)
        self.setPID = QPushButton("off")
        self.setPID.setMaximumWidth(50)
        self.lMotorLabel = QLabel("left power:")
        self.lMotorLabel.setAlignment(Qt.AlignRight)
        self.lMotorInput = QLineEdit()
        self.rMotorLabel = QLabel("right power:")
        self.rMotorLabel.setAlignment(Qt.AlignRight)
        self.rMotorInput = QLineEdit()
        self.setMotorInput = QPushButton("set")
        self.setMotorInput.setMaximumWidth(50)
        self.lEncoderLabel = QLabel("left encoder:")
        self.lEncoderLabel.setAlignment(Qt.AlignRight)
        self.lEncoderValue = QLineEdit()
        self.rEncoderLabel = QLabel("right encoder:")
        self.rEncoderLabel.setAlignment(Qt.AlignRight)
        self.rEncoderValue = QLineEdit()

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine, 1, 0, 1, 2)
        self.layout.addWidget(self.forward, 2, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.ccw, 3, 0, 1, 1, Qt.AlignLeft)
        self.layout.addWidget(self.cw, 3, 1, 1, 1, Qt.AlignRight)
        self.layout.addWidget(self.back, 4, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.PIDLabel, 6, 0)
        self.layout.addWidget(self.setPID, 6, 1)
        self.layout.addWidget(self.lMotorLabel, 7, 0)
        self.layout.addWidget(self.lMotorInput, 7, 1)
        self.layout.addWidget(self.rMotorLabel, 8, 0)
        self.layout.addWidget(self.rMotorInput, 8, 1)
        self.layout.addWidget(self.setMotorInput, 9, 1)
        self.layout.addWidget(self.lEncoderLabel, 11, 0)
        self.layout.addWidget(self.lEncoderValue, 11, 1)
        self.layout.addWidget(self.rEncoderLabel, 12, 0)
        self.layout.addWidget(self.rEncoderValue, 12, 1)

        # make row after label and between sections stretch
        self.layout.setRowStretch(13, 1)

        self.setLayout(self.layout)

class ArmFrame(QFrame):

    def __init__(self, parent=None):
        super(ArmFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)

        # widgets
        self.label = QLabel("Arm")


if __name__ == '__main__':
    # create the app
    app = QApplication(sys.argv)
    
    # show the form
    main = MainWindow()
    main.show()
    main.raise_()
    # run the main loop
    sys.exit(app.exec_())
