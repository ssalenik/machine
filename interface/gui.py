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
        self.controls = ControlsFrame()

        # layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(1)
        self.layout.addWidget(self.settings)
        self.layout.addWidget(self.controls)

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
        self.connectButton = QPushButton("connect")
        
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
        self.layout.addWidget(self.connectButton, 2, 3)
        self.layout.addWidget(self.statusLabel, 2, 4)

        # make middle column stretch
        self.layout.setColumnStretch(2, 1)

        self.setLayout(self.layout)

class ControlsFrame(QFrame):
    
    def __init__(self, parent=None):
        super(ControlsFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)

        # widgets
        self.vLine1 = QFrame()
        self.vLine1.setFrameStyle(QFrame.VLine | QFrame.Raised)
        self.vLine1.setLineWidth(2)
        self.vLine2 = QFrame()
        self.vLine2.setFrameStyle(QFrame.VLine | QFrame.Raised)
        self.vLine2.setLineWidth(2)
        self.chassy = ChassyFrame()
        self.arm = ArmFrame()
        self.claw = ClawFrame()

        # layout
        self.layout = QHBoxLayout()
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(0,10,0,10)
        self.layout.addWidget(self.chassy)
        self.layout.addWidget(self.vLine1)
        self.layout.addWidget(self.arm)
        self.layout.addWidget(self.vLine2)
        self.layout.addWidget(self.claw)

        self.setLayout(self.layout)


class ChassyFrame(QFrame):

    def __init__(self, parent=None):
        super(ChassyFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label = QLabel("Chassy")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.forwardButton = QPushButton("forward")
        self.forwardButton.setMinimumWidth(110)
        self.backwardButton = QPushButton("backward")
        self.backwardButton.setMinimumWidth(110)
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setMinimumWidth(110)
        self.cwButton = QPushButton("CW")
        self.cwButton.setMinimumWidth(110)
        self.PIDLabel = QLabel("PID toggle:")
        self.PIDLabel.setAlignment(Qt.AlignRight)
        self.PIDSet = QPushButton("on")
        self.PIDSet.setMaximumWidth(50)
        self.lMotorLabel = QLabel("left power:")
        self.lMotorLabel.setAlignment(Qt.AlignRight)
        self.lMotorInput = QLineEdit()
        self.lMotorInput.setMaximumWidth(100)
        self.rMotorLabel = QLabel("right power:")
        self.rMotorLabel.setAlignment(Qt.AlignRight)
        self.rMotorInput = QLineEdit()
        self.rMotorInput.setMaximumWidth(100)
        self.motorInputSet = QPushButton("set")
        self.motorInputSet.setMaximumWidth(50)
        self.lEncoderLabel = QLabel("left encoder:")
        self.lEncoderLabel.setAlignment(Qt.AlignRight)
        self.lEncoderValue = QLineEdit()
        self.lEncoderValue.setMaximumWidth(100)
        self.rEncoderLabel = QLabel("right encoder:")
        self.rEncoderLabel.setAlignment(Qt.AlignRight)
        self.rEncoderValue = QLineEdit()
        self.rEncoderValue.setMaximumWidth(100)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine, 1, 0, 1, 2)
        self.layout.addWidget(self.forwardButton, 2, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.ccwButton, 3, 0, 1, 1, Qt.AlignLeft)
        self.layout.addWidget(self.cwButton, 3, 1, 1, 1, Qt.AlignRight)
        self.layout.addWidget(self.backwardButton, 4, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.PIDLabel, 6, 0)
        self.layout.addWidget(self.PIDSet, 6, 1)
        self.layout.addWidget(self.lMotorLabel, 7, 0)
        self.layout.addWidget(self.lMotorInput, 7, 1)
        self.layout.addWidget(self.rMotorLabel, 8, 0)
        self.layout.addWidget(self.rMotorInput, 8, 1)
        self.layout.addWidget(self.motorInputSet, 9, 1)
        self.layout.addWidget(self.lEncoderLabel, 11, 0)
        self.layout.addWidget(self.lEncoderValue, 11, 1)
        self.layout.addWidget(self.rEncoderLabel, 12, 0)
        self.layout.addWidget(self.rEncoderValue, 12, 1)

        # make row at the end stretch
        self.layout.setRowStretch(13, 1)

        self.setLayout(self.layout)

class ArmFrame(QFrame):

    def __init__(self, parent=None):
        super(ArmFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label = QLabel("Arm")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.baseLabel = QLabel("base")
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setMinimumWidth(110)
        self.cwButton = QPushButton("CW")
        self.cwButton.setMinimumWidth(110)
        self.basePowerLabel = QLabel("power:")
        self.basePowerInput = QLineEdit()
        self.basePowerInput.setMaximumWidth(100)
        self.basePowerSet = QPushButton("set")
        self.basePowerSet.setMinimumWidth(50)
        self.linActuatorLabel = QLabel("linear actuator")
        self.upButton = QPushButton("up")
        self.upButton.setMinimumWidth(110)
        self.downButton = QPushButton("down")
        self.downButton.setMinimumWidth(110)
        self.linActPowerLabel = QLabel("power:")
        self.linActPowerInput = QLineEdit()
        self.linActPowerInput.setMaximumWidth(100)
        self.linActPowerSet = QPushButton("set")
        self.basePowerSet.setMinimumWidth(50)
        self.eMagnetLabel = QLabel("electro magnet")
        self.eMagnetSwitchLabel = QLabel("toggle:")
        self.eMagnetSwitch = QPushButton("on")
        self.eMagnetSwitch.setMinimumWidth(50)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine, 1, 0, 1, 4)
        self.layout.addWidget(self.baseLabel, 2, 0, 1, 4)
        self.layout.addWidget(self.ccwButton, 3, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.cwButton, 3, 2, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.basePowerLabel, 4, 0)
        self.layout.addWidget(self.basePowerInput, 4, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.basePowerSet, 4, 3)
        self.layout.addWidget(self.linActuatorLabel, 5, 0, 1, 4)
        self.layout.addWidget(self.upButton, 6, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.downButton, 7, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.linActPowerLabel, 8, 0)
        self.layout.addWidget(self.linActPowerInput, 8, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.linActPowerSet, 8, 3)
        self.layout.addWidget(self.eMagnetLabel, 9, 0, 1, 4)
        self.layout.addWidget(self.eMagnetSwitchLabel, 10, 0)
        self.layout.addWidget(self.eMagnetSwitch, 10, 1, 1, 2, Qt.AlignHCenter)

        # make row at the end stretch
        self.layout.setRowStretch(11, 1)

        self.setLayout(self.layout)

class ClawFrame(QFrame):

    def __init__(self, parent=None):
        super(ClawFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label = QLabel("Claw")
        self.hLine1= QFrame()
        self.hLine1.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.otherLabel = QLabel("Other")
        self.hLine2= QFrame()
        self.hLine2.setFrameStyle(QFrame.HLine | QFrame.Sunken)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine1, 1, 0, 1, 2)
        self.layout.addWidget(self.otherLabel, 3, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine2, 4, 0, 1, 2)

        # make row at the end stretch
        self.layout.setRowStretch(2, 1)
        self.layout.setRowStretch(5, 1)

        self.setLayout(self.layout)


if __name__ == '__main__':
    # create the app
    app = QApplication(sys.argv)
    
    # show the form
    main = MainWindow()
    main.show()
    main.raise_()
    # run the main loop
    sys.exit(app.exec_())
