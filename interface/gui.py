#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import serial
import serial.tools
import serial.tools.list_ports
import time
from threading import Thread
from PySide.QtCore import *
from PySide.QtGui import *

POLL_RATE = 20        # default serial poll rate
MAX_POLL_RATE = 100   # max serial poll rate
MAX_REFRESH_RATE = 30 # max gui refresh rate

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Command Centre - McGill - Eng Games 2013")

        self.centre = CentralWidget() 
        self.setCentralWidget(self.centre)

    def closeEvent(self, event):
        self.centre.closeAll()

        event.accept() # let the window close

class OutputWindow(QMainWindow):
    def __init__(self, parent=None):
        super(OutputWindow, self).__init__(parent)
        self.setWindowTitle("output")

        self.outputText = QTextEdit("<b>output should go here</b>")
        self.outputText.setReadOnly(True)
        self.setCentralWidget(self.outputText)
        self.resize(350, 550)

        self.closing = False

    def closeEvent(self, event):
        # don't close it
        if self.closing == True:
            event.accept() # let the window close
        else:
            event.ignore()

    def hideEvent(self, event):
        # don't hide it
        #event.accept()
        event.ignore()

class CentralWidget(QWidget):

    def __init__(self, parent=None):
        super(CentralWidget, self).__init__(parent)

        # status
        self.connected = False
        
        # widgets which will be contained in the central widget
        self.settings = Settings()
        self.controls = ControlsFrame()
        self.command = Commands()

        #serial
        self.serial = serial.Serial()

        # layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(1)
        self.layout.addWidget(self.settings)
        self.layout.addWidget(self.controls)
        self.layout.addWidget(self.command)

        self.setLayout(self.layout)

        # second window
        self.outputWindow = OutputWindow(parent=self)

        # signals
        self.settings.connectButton.clicked.connect(self.connect)


    def connect(self):
        if self.connected == True :
            self.connected = False
            while self.serialThread.isAlive() :
                None
            self.serial.close()
            self.outputWindow.outputText.append("<font color=green>closed serial port</font>")
            self.settings.connectButton.setText("connect")

        else :
            self.port = self.settings.portSelect.currentText()
            self.outputWindow.outputText.append("<font color=green>trying to connect to port <b>%s</b></font>" % self.port)
            self.serial.port =self.port
            self.serial.timeout = 1
            # try to connect a few times
            i = 0
            while not self.serial.isOpen() and (i < 20) :
                i += 1
                try:
                    self.serial.open()
                except serial.SerialException:
                    None
                    
                if not self.serial.isOpen():
                    self.outputWindow.outputText.append("<font color=red>could not open connection, trying again</font>")

            if self.serial.isOpen():
                self.connected = True
                self.serialThread = Thread(target=self.pollSerial)
                self.serialThread.daemon = True
                self.serialThread.start()
                self.settings.connectButton.setText("disconnect")
            else:
                self.outputWindow.outputText.append("<font color=red>could not open connection, check if the port is correct</font>")

    def pollSerial(self):
        self.outputWindow.outputText.append("<font color=green>listening to serial port </font>")
        while(self.connected):
            if self.serial.isOpen():
                data = self.serial.read()
                if data :
                    self.outputWindow.outputText.append("<font color=black>%s</font>" % data.encode('us-ascii','xmlcharrefreplace') )#.encode("hex"))
                    time.sleep(0.1)
            else:
                self.outputWindow.outputText.append("<font color=red>connect terminated unexpectedly</font>")
                self.connected = False
                self.settings.connectButton.setText("connect")

        self.outputWindow.outputText.append("<font color=green>stopping listening to serial port</font>")

    def disconnected(self):
            self.settings.statusLabel.setPixmap(self.settings.redFill)

    def closeAll(self):
        self.outputWindow.closing = True
 

# class Controller(QObjcet):

#     # define slots
#     disconnected = Signal()

#     def __init__(self, logger, parent=None):
#         super(Controller, self).__init__(parent)

#         self.log = logger

#     def connect(self, port):

#     def disconnect(self):



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
        self.portSelect.addItem("select or enter port")
        #get all serial ports
        self.serialPorts = serial.tools.list_ports.comports()
        for port in self.serialPorts:
            self.portSelect.addItem(port[0])
        self.portSelect.setMinimumWidth(200)
        self.portSelect.setEditable(True)
        self.portSelect.setToolTip("serial port should be in the form of \"COM#\" on windows and \"/dev/tty.*\" on linux/osx")
        self.rateLabel = QLabel("poll rate:")
        self.rateInput = QSpinBox()
        self.rateInput.setValue(POLL_RATE)
        self.rateInput.setMinimum(1)
        self.rateInput.setMaximum(MAX_POLL_RATE)
        #self.rateInput.setMaximumWidth(50)
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
        self.layout.addWidget(self.hLine, 1, 0, 1, 7)
        self.layout.addWidget(self.portLabel, 2, 0)
        self.layout.addWidget(self.portSelect, 2, 1)
        self.layout.addWidget(self.rateLabel, 2, 2)
        self.layout.addWidget(self.rateInput, 2, 3)
        self.layout.addWidget(self.connectButton, 2, 5)
        self.layout.addWidget(self.statusLabel, 2, 6)

        # make middle column stretch
        self.layout.setColumnStretch(4, 1)

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
        self.openButton = QPushButton("open")
        self.openButton.setMinimumWidth(110)
        self.closeButton = QPushButton("close")
        self.closeButton.setMinimumWidth(110)
        self.clawPowerLabel = QLabel("power:")
        self.clawPowerInput = QLineEdit()
        self.clawPowerInput.setMaximumWidth(100)
        self.clawPowerSet = QPushButton("set")
        self.clawPowerSet.setMinimumWidth(50)
        self.raiseButton = QPushButton("raise")
        self.raiseButton.setMinimumWidth(110)
        self.lowerButton = QPushButton("lower")
        self.lowerButton.setMinimumWidth(110)
        self.heightPowerLabel = QLabel("power:")
        self.heightPowerInput = QLineEdit()
        self.heightPowerInput.setMaximumWidth(100)
        self.heightPowerSet = QPushButton("set")
        self.heightPowerSet.setMinimumWidth(50)

        self.hLine2= QFrame()
        self.hLine2.setFrameStyle(QFrame.HLine | QFrame.Sunken)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine1, 1, 0, 1, 4)
        self.layout.addWidget(self.openButton, 2, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.closeButton, 2, 2, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.clawPowerLabel, 3, 0)
        self.layout.addWidget(self.clawPowerInput, 3, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.clawPowerSet, 3, 3)
        self.layout.addWidget(self.raiseButton, 4, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.lowerButton, 5, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.heightPowerLabel, 6, 0)
        self.layout.addWidget(self.heightPowerInput, 6, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.heightPowerSet, 6, 3)
        self.layout.addWidget(self.otherLabel, 7, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine2, 8, 0, 1, 4)

        # make row at the end stretch
        self.layout.setRowStretch(9, 1)

        self.setLayout(self.layout)

class Commands(QFrame):

    def __init__(self, parent=None):
        super(Commands, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)
        
        # widgets
        self.label = QLabel("Custom Command:")
        #self.hLine = QFrame()
        #self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.commandInput = QLineEdit()
        self.commandInput.setMinimumWidth(200)
        self.sendButton = QPushButton("send")

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0)
        self.layout.addWidget(self.commandInput, 0, 1)
        self.layout.addWidget(self.sendButton, 0, 2)

        # make middle column stretch
        self.layout.setColumnStretch(3, 1)

        # make last row stretch
        self.layout.setRowStretch(1, 1)

        self.setLayout(self.layout)

if __name__ == '__main__':
    # create the app
    app = QApplication(sys.argv)
    
    # show the form
    main = MainWindow()
    
    main.centre.outputWindow.show()
    main.centre.outputWindow.move(main.centre.outputWindow.x() + 600, main.centre.outputWindow.y())
    main.centre.outputWindow.raise_()
    main.show()
    main.move(main.x() - 200, main.y() + 50)
    main.raise_()
    # run the main loop
    sys.exit(app.exec_())
