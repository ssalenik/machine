#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import serial
import serial.tools
import serial.tools.list_ports
from struct import *
#import io
import time
import copy
from threading import Thread
from threading import Lock
from PySide.QtCore import *
from PySide.QtGui import *
import platform
import csv
from time import strftime

from serialComm import *

POLL_RATE = 20        # default serial poll rate
MAX_POLL_RATE = 100    # max serial poll rate
GUI_RATE = 15         # max gui refresh rate
MAX_LINES = 1000      # max lines in output window

# slot defines

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Command Centre - McGill - Eng Games 2013")        

        # main layout
        self.centre = CentralWidget(parent=self) 
        self.setCentralWidget(self.centre)

        self.setWindowIcon(QIcon('mcgilllogo.png'))

    def closeEvent(self, event):
        # make sure we're disconnected from the port; kill all threads"
        print "quitting"
        self.centre.closing()
        event.accept()

class CentralWidget(QWidget):

    def __init__(self, parent=None):
        super(CentralWidget, self).__init__(parent)

        # status
        self.connected = False
        self.refresh = True
        
        # widgets which will be contained in the central widget
        self.settings = Settings()
        self.controls = ControlsFrame()
        self.command = Commands()
        self.logSelect = LogSelectFrame()

        # output
        self.output = Output()
        self.out = self.output.output

        # logger
        self.logger = Logger(output=self.out)

        # controller
        self.controller = Controller(output=self.out, logger=self.logger)

        # layout
        self.layout = QGridLayout()
        self.layout.setVerticalSpacing(1)
        self.layout.setHorizontalSpacing(12)
        self.layout.addWidget(self.settings, 0, 0)
        self.layout.addWidget(self.controls, 1, 0)
        self.layout.addWidget(self.command, 2, 0)
        self.layout.addWidget(self.output, 0, 1, 4, 1)
        self.layout.addWidget(self.logSelect, 3, 0)

        self.layout.setColumnStretch(1, 1)
        self.setLayout(self.layout)

        # disable stuff untill connected
        self.disableButtons()

        # settings signals
        self.settings.connectButton.clicked.connect(self.connect)
        self.settings.rateInput.valueChanged.connect(self.controller.changePollRate)
        self.controller.connectionLost.connect(self.disconnected)
        self.settings.mcuSelect.currentIndexChanged[unicode].connect(self.mcuSelect)
        self.settings.debugSelect.stateChanged.connect(self.debugRequest)
        self.settings.printSelect.stateChanged.connect(self.printAll)
        self.settings.refreshButton.clicked.connect(self.refreshRequest)

        self.controls.stopAllButton.clicked.connect(self.stopAll)

        # chassy signals
        self.controls.chassy.PIDSwitch.stateChanged.connect(self.PIDToggle)
        self.controls.chassy.forwardButton.pressed.connect(self.forward)
        self.controls.chassy.forwardButton.released.connect(self.stopChassy)
        self.controls.chassy.backwardButton.pressed.connect(self.backward)
        self.controls.chassy.backwardButton.released.connect(self.stopChassy)
        self.controls.chassy.cwButton.pressed.connect(self.chassyCW)
        self.controls.chassy.cwButton.released.connect(self.stopChassy)
        self.controls.chassy.ccwButton.pressed.connect(self.chassyCCW)
        self.controls.chassy.ccwButton.released.connect(self.stopChassy)

        # arm signals
        self.controls.arm.ccwButton.pressed.connect(self.baseCCW)
        self.controls.arm.ccwButton.released.connect(self.stopBase)
        self.controls.arm.cwButton.pressed.connect(self.baseCW)
        self.controls.arm.cwButton.released.connect(self.stopBase)
        self.controls.arm.upButton.pressed.connect(self.armUp)
        self.controls.arm.upButton.released.connect(self.stopArm)
        self.controls.arm.downButton.pressed.connect(self.armDown)
        self.controls.arm.downButton.released.connect(self.stopArm)
        self.controls.arm.setBaseRef.clicked.connect(self.setBaseRef)
        self.controls.arm.setLinActRef.clicked.connect(self.setArmRef)

        # claw signals
        self.controls.claw.openButton.pressed.connect(self.clawOpen)
        self.controls.claw.openButton.released.connect(self.stopClaw)
        self.controls.claw.closeButton.pressed.connect(self.clawClose)
        self.controls.claw.closeButton.released.connect(self.stopClaw)
        self.controls.claw.raiseButton.pressed.connect(self.clawRaise)
        self.controls.claw.raiseButton.released.connect(self.stopHeight)
        self.controls.claw.lowerButton.pressed.connect(self.clawLower)
        self.controls.claw.lowerButton.released.connect(self.stopHeight)

        # command signals
        self.command.sendButton.clicked.connect(self.sendCustom)

        # logger signals
        self.logSelect.logButton.clicked.connect(self.addLoggers)

        # start gui update thread
        self.refreshThread = Thread(target=self.refreshGUI)
        self.refreshThread.start()

    def addLoggers(self):
        codes = self.logSelect.logInput.text().split()
        for i in range(len(codes)):
            codes[i] = codes[i].replace(",", "")

        for code in codes:
            self.logger.openLogFile(code)

        self.logSelect.logInput.setText("")

    def sendCustom(self):
        self.controller.sendCustomMessage(self.command.commandInput.text())

    def stopAll(self):
        self.controller.sendCustomMessage(STOP_ALL)

    def clawOpen(self):
        #TODO
        None

    def clawClose(self):
        #TODO
        None

    def stopClaw(self):
        #TODO
        None

    def clawRaise(self):
        #TODO
        None

    def clawLower(self):
        #TODO
        None

    def stopHeight(self):
        #TODO
        None

    def setArmRef(self):
        self.controller.sendMessage(code=mainCPU['arm']|mainCPU['encoder'], sendToDriver=False, data=self.controls.arm.linActRefInput.value(), encoding='s16')

    def setBaseRef(self):
        self.controller.sendMessage(code=mainCPU['base']|mainCPU['encoder'], sendToDriver=False, data=self.controls.arm.baseRefInput.value(), encoding='s16')

    def baseCCW(self):
        self.controller.sendMessage(code=mainCPU['base']|mainCPU['power_ccw_up'], sendToDriver=False, data=self.controls.arm.basePowerInput.value(), encoding='u8')

    def baseCW(self):
        self.controller.sendMessage(code=mainCPU['base']|mainCPU['power_cw_down'], sendToDriver=False, data=self.controls.arm.basePowerInput.value(), encoding='u8')

    def stopBase(self):
        self.controller.sendMessage(code=mainCPU['base']|mainCPU['power_ccw_up'], sendToDriver=False, data=0, encoding='u8')
        self.controller.sendMessage(code=mainCPU['base']|mainCPU['power_cw_down'], sendToDriver=False, data=0, encoding='u8')

    def armUp(self):
        self.controller.sendMessage(code=mainCPU['arm']|mainCPU['power_ccw_up'], sendToDriver=False, data=self.controls.arm.linActPowerInput.value(), encoding='u8')

    def armDown(self):
        self.controller.sendMessage(code=mainCPU['arm']|mainCPU['power_cw_down'], sendToDriver=False, data=self.controls.arm.linActPowerInput.value(), encoding='u8')

    def stopArm(self):
        self.controller.sendMessage(code=mainCPU['arm']|mainCPU['power_ccw_up'], sendToDriver=False, data=0, encoding='u8')
        self.controller.sendMessage(code=mainCPU['arm']|mainCPU['power_cw_down'], sendToDriver=False, data=0, encoding='u8')
        

    def stopChassy(self):
        if self.controls.chassy.PIDSwitch.isChecked() :
            self.controller.sendMessage(code=driver['speed_both'], data=0, encoding='u8')
        else:
            self.controller.sendMessage(code=driver['power_both'], data=0, encoding='u8')

    def chassyCCW(self):
        # set direction
        self.controller.sendMessage(code=driver['dir_left'], data=BACKWARD, encoding='u8')
        self.controller.sendMessage(code=driver['dir_right'], data=FORWARD, encoding='u8')

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def chassyCW(self):
        # set direction
        self.controller.sendMessage(code=driver['dir_left'], data=FORWARD, encoding='u8')
        self.controller.sendMessage(code=driver['dir_right'], data=BACKWARD, encoding='u8')

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def backward(self):
        # set direction
        self.controller.sendMessage(code=driver['dir_left'], data=BACKWARD, encoding='u8')
        self.controller.sendMessage(code=driver['dir_right'], data=BACKWARD, encoding='u8')

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def forward(self):
        # set direction
        self.controller.sendMessage(code=driver['dir_left'], data=FORWARD, encoding='u8')
        self.controller.sendMessage(code=driver['dir_right'], data=FORWARD, encoding='u8')

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')


    def PIDToggle(self, state):
        if state == Qt.CheckState.Checked:
            # set speed instead of power
            self.controls.chassy.inputLabel.setText("speed:")
            self.controls.chassy.lMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.rMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.lMotorInput.setValue(0)
            self.controls.chassy.rMotorInput.setValue(0)
        else:
            # set power instead of speed
            self.controls.chassy.inputLabel.setText("power:")
            self.controls.chassy.lMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.rMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.lMotorInput.setValue(0)
            self.controls.chassy.rMotorInput.setValue(0)

    def refreshRequest(self):
        self.controller.requestFeedback()

    def debugRequest(self, state):
        if state == Qt.CheckState.Checked:
            # turn on all teh debugs
            self.controller.sendMessage(code=driver['debug_1'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_2'], data=1, encoding='u8')
            # self.controller.sendMessage(code=driver['debug_3'], data=1)
            # self.controller.sendMessage(code=driver['debug_4'], data=1)
            # self.controller.sendMessage(code=driver['debug_5'], data=1)
            # self.controller.sendMessage(code=driver['debug_6'], data=1)
            # self.controller.sendMessage(code=driver['debug_7'], data=1)
        else:
            # turn off debug
            # self.controller.sendMessage(code=driver['debug_off'])
            self.controller.sendMessage(code=driver['debug_1'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_2'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_3'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_4'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_5'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_6'], data=0, encoding='u8')
            self.controller.sendMessage(code=driver['debug_7'], data=0, encoding='u8')

    def printAll(self, state):
        if state == Qt.CheckState.Checked:
            self.controller.printAll = True
        else:
            self.controller.printAll = False

    def mcuSelect(self, checked):
        if checked == "main CPU" :
            self.controller.connectedToMainCPU = True
        else :
            self.controller.connectedToMainCPU = False

    def closing(self):
        # disconnect if connected

        self.logSelect.close
        if self.connected :
            self.connect()

        # end refresh thread
        self.refresh = False
        self.refreshThread.join()

    def connect(self):
        if self.connected == True :
            self.disableButtons()
            self.connected = False
            #self.refreshThread.join()
            #self.controller.disconnect()
            self.controller.disconnect()
            self.settings.connectButton.setText("connect")
            self.settings.statusLabel.setPixmap(self.settings.redFill)
            self.settings.portSelect.setEnabled(True)
        else :
            if self.controller.connectToPort(port=self.settings.portSelect.currentText()):
                self.connected = True
                self.settings.connectButton.setText("disconnect")
                self.settings.statusLabel.setPixmap(self.settings.greenFill)
                self.settings.portSelect.setEnabled(False)
                #self.refreshThread = Thread(target=refreshGUI)
                #self.refreshThread.start()
                self.enableButtons()
    
    def disconnected(self):
        self.settings.statusLabel.setPixmap(self.settings.redFill)
        self.connected = False
        self.settings.connectButton.setText("connect")
        self.settings.portSelect.setEnabled(True)
        self.disableButtons()

    def enableButtons(self):
        self.settings.enableButtons()
        self.controls.enableButtons()
        self.command.enableButtons()

    def disableButtons(self):
        self.settings.disableButtons()
        self.controls.disableButtons()
        self.command.disableButtons()

    def refreshGUI(self):
        while(self.refresh):
            if self.connected :
                c = self.controller
                # chassy
                chassy = self.controls.chassy
                chassy.lSpeedValue.setText("%i" % c.speed_act_left)
                chassy.rSpeedValue.setText("%i" % c.speed_act_right)
                chassy.lEncoderValue.setText("%i" % c.encoder_left)
                chassy.rEncoderValue.setText("%i" % c.encoder_right)
                chassy.lSensorValue.setText("%i" % c.sensor_left)
                chassy.rSensorValue.setText("%i" % c.sensor_right)
                chassy.lPositionValue.setText("%i" % c.position_left)
                chassy.rPositionValue.setText("%i" % c.position_right)
                # arm
                arm = self.controls.arm
                arm.baseEncoderValue.setText("%i" % c.encoder_base)
                arm.linActEncoderValue.setText("%i" % c.encoder_arm)
                arm.basePValue.setText("%i" % c.p_base)
                arm.baseIValue.setText("%i" % c.i_base)
                arm.baseDValue.setText("%i" % c.d_base)
                arm.linActPValue.setText("%i" % c.p_arm)
                arm.linActIValue.setText("%i" % c.i_arm)
                arm.linActDValue.setText("%i" % c.d_arm)
                # claw
                # claw = self.controls.claw
                # claw.clawEncoderValue.setText("%i" % c.encoder_claw)
                # claw.heightEncoderValue.setText("%i" % c.encoder_claw_height)

            # output
            self.output.refresh()

            time.sleep(1.0/float(GUI_RATE))
        
class Output(QTextEdit):
    def __init__(self, parent=None):
        QTextEdit.__init__(self, "<b>output should go here</b>")
        
        self.setReadOnly(True)
        self.setMinimumWidth(350)

        # output lock to make outputting thread safe
        self.outputLock = Lock()

        self.newOutput = []

        self.printedLastRefresh = False # for scrolling

        # set the maximum paragraph count
        # will delete paragraphs from the beginging once the limit is reached
        self.document().setMaximumBlockCount(MAX_LINES)

    def output(self, text):
        self.outputLock.acquire()
        self.newOutput.append(copy.copy(text))
        self.outputLock.release()

    def refresh(self):
        if self.printedLastRefresh:
            sb = self.verticalScrollBar()
            sb.setValue(sb.maximum())

        self.outputLock.acquire()
        temp = self.newOutput
        self.newOutput = [] # should create a new list
        self.outputLock.release()

        if temp :
            self.printedLastRefresh = True
            for text in temp :
                if text :
                    self.append(text)
        else :
            self.printedLastRefresh = False         

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
        self.portLabel = QLabel("port: ")
        self.portSelect = QComboBox()
        self.portSelect.addItem("select or enter port")
        #get all serial ports
        if platform.system() == 'Linux' :
            self.portSelect.addItem('/dev/rfcomm0')
        else:
            self.serialPorts = serial.tools.list_ports.comports()
            for port in self.serialPorts:
                self.portSelect.addItem(port[0])
        self.portSelect.setMinimumWidth(300)
        self.portSelect.setEditable(True)
        self.portSelect.setToolTip("serial port should be in the form of \"COM#\" on windows and \"/dev/tty.*\" on linux/osx")
        self.refreshButton = QPushButton("refresh all")
        self.refreshButton.setFixedWidth(110)
        self.rateSwitch = QCheckBox("auto refresh at")
        self.rateSwitch.setCheckState(Qt.Checked)
        self.rateInput = QSpinBox()
        self.rateInput.setValue(POLL_RATE)
        self.rateInput.setMinimum(1)
        self.rateInput.setMaximum(MAX_POLL_RATE)
        #self.rateInput.setFixedWidth(50)
        self.rateUnits = QLabel("Hz")
        self.mcuLabel = QLabel("connected to:")
        self.mcuSelect = QComboBox()
        self.mcuSelect.setFixedWidth(100)
        self.mcuSelect.addItem("main CPU")
        self.mcuSelect.addItem("driver")
        self.connectButton = QPushButton("connect")
        self.connectButton.setFixedWidth(110)
        self.debugSelect = QCheckBox("debug")
        self.printSelect = QCheckBox("print all")
        
        # flashing connection status button
        self.statusLabel = QLabel(self)
        self.redFill = QPixmap(20,20)
        self.redFill.fill(Qt.red)
        self.greenFill = QPixmap(20,20)
        self.greenFill.fill(Qt.green)
        self.statusLabel.setPixmap(self.redFill)

        # layout
        layout = QGridLayout()
        layout.setHorizontalSpacing(5)
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.hLine, 1, 0, 1, 14)
        layout.addWidget(self.portLabel, 2, 0)
        layout.addWidget(self.portSelect, 2, 1, 1, 8, Qt.AlignLeft)
        layout.addWidget(self.connectButton, 2, 11, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.statusLabel, 2, 13, Qt.AlignHCenter)
        layout.addWidget(self.refreshButton, 3, 0, 1, 2, Qt.AlignLeft)
        layout.addWidget(self.rateSwitch, 3, 3, Qt.AlignRight)
        layout.addWidget(self.rateInput, 3, 4, Qt.AlignRight)
        layout.addWidget(self.rateUnits, 3, 5, Qt.AlignLeft)
        layout.addWidget(self.mcuLabel, 3, 7, Qt.AlignRight)
        layout.addWidget(self.mcuSelect, 3, 8, Qt.AlignLeft)
        layout.addWidget(self.debugSelect, 3, 10, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.printSelect, 3, 12, 1, 2, Qt.AlignHCenter)

        self.setLayout(layout)

        # signals
        self.rateSwitch.stateChanged.connect(self.enableAutoRefresh)

    def enableAutoRefresh(self, state):
        if state == Qt.CheckState.Checked:
            self.rateInput.setEnabled(True)
        else:
            self.rateInput.setEnabled(False)

    def enableButtons(self):
        self.refreshButton.setEnabled(True)
        self.debugSelect.setEnabled(True)

    def disableButtons(self):
        self.refreshButton.setEnabled(False)
        self.debugSelect.setEnabled(False)


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
        self.stopAllButton = QPushButton("STOP ALL MOTORS")
        font = self.stopAllButton.font()
        font.setPointSize(15)
        self.stopAllButton.setFont(font)
        self.stopAllButton.setStyleSheet(
            "QPushButton"
                "{color: red;}"
            "QPushButton:disabled"
                "{color: gray;}"
            "QPushButton:pressed"
                "{color: yellow}"
                )

        # layout
        layout = QGridLayout()
        layout.setHorizontalSpacing(0)
        #self.layout.setContentsMargins(0,10,0,10)
        layout.addWidget(self.stopAllButton, 0, 0, 1, 5)
        layout.addWidget(self.chassy, 1, 0)
        layout.addWidget(self.vLine1, 1, 1)
        layout.addWidget(self.arm, 1, 2)
        layout.addWidget(self.vLine2, 1, 3)
        layout.addWidget(self.claw, 1, 4)

        self.setLayout(layout)

    def enableButtons(self):
        self.setEnabled(True)

    def disableButtons(self):
        self.setEnabled(False)

class ChassyFrame(QFrame):

    def __init__(self, parent=None):
        super(ChassyFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label = QLabel("Chassy")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.forwardButton = QPushButton("forward")
        self.forwardButton.setFixedWidth(110)
        self.backwardButton = QPushButton("backward")
        self.backwardButton.setFixedWidth(110)
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setFixedWidth(110)
        self.cwButton = QPushButton("CW")
        self.cwButton.setFixedWidth(110)
        self.PIDSwitch = QCheckBox("PID on")

        # left/right
        self.lMotorLabel = QLabel("left:")
        self.lMotorLabel.setAlignment(Qt.AlignHCenter)
        self.rMotorLabel = QLabel("right:")
        self.rMotorLabel.setAlignment(Qt.AlignHCenter)
        # input power/speed
        self.inputLabel = QLabel("power:")
        self.lMotorInput = QSpinBox()
        self.lMotorInput.setMinimum(0)
        self.lMotorInput.setMaximum(100) #100 for power
        self.lMotorInput.setFixedWidth(75)
        self.rMotorInput = QSpinBox()
        self.rMotorInput.setMinimum(0)
        self.rMotorInput.setMaximum(100) #100 for power
        self.rMotorInput.setFixedWidth(75)
        # actual speed
        self.speedLabel = QLabel("speed:")
        #self.speedLabel.setAlignment(Qt.AlignHCenter)
        self.lSpeedValue = QLineEdit()
        self.lSpeedValue.setReadOnly(True)
        self.lSpeedValue.setFixedWidth(75)
        self.rSpeedValue = QLineEdit()
        self.rSpeedValue.setReadOnly(True)
        self.rSpeedValue.setFixedWidth(75)
        # encoder
        self.encoderLabel = QLabel("encoder:")
        #self.encoderLabel.setAlignment(Qt.AlignHCenter)
        self.lEncoderValue = QLineEdit()
        self.lEncoderValue.setReadOnly(True)
        self.lEncoderValue.setFixedWidth(75)
        self.rEncoderValue = QLineEdit()
        self.rEncoderValue.setReadOnly(True)
        self.rEncoderValue.setFixedWidth(75)
        # sensor
        self.sensorLabel = QLabel("sensor:")
        #self.sensorLabel.setAlignment(Qt.AlignHCenter)
        self.lSensorValue = QLineEdit()
        self.lSensorValue.setReadOnly(True)
        self.lSensorValue.setFixedWidth(75)
        self.rSensorValue = QLineEdit()
        self.rSensorValue.setReadOnly(True)
        self.rSensorValue.setFixedWidth(75)
        # position
        self.positionLabel = QLabel("position:")
        #self.positionLabel.setAlignment(Qt.AlignHCenter)
        self.lPositionValue = QLineEdit()
        self.lPositionValue.setReadOnly(True)
        self.lPositionValue.setFixedWidth(75)
        self.rPositionValue = QLineEdit()
        self.rPositionValue.setReadOnly(True)
        self.rPositionValue.setFixedWidth(75)

        # main layout
        layout = QGridLayout()

        layout.addWidget(self.label, 0, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.hLine, 1, 0, 1, 2)
        layout.addWidget(self.forwardButton, 2, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.ccwButton, 3, 0, 1, 1, Qt.AlignLeft)
        layout.addWidget(self.cwButton, 3, 1, 1, 1, Qt.AlignRight)
        layout.addWidget(self.backwardButton, 4, 0, 1, 2, Qt.AlignHCenter)

        # bottom layout
        bottomLayout = QGridLayout()
        bottomLayout.setContentsMargins(0, 0, 0, 0)
        bottomLayout.setHorizontalSpacing(5)
        chassyValues = QWidget()
        bottomLayout.addWidget(self.lMotorLabel, 0, 1)
        bottomLayout.addWidget(self.rMotorLabel, 0, 2)
        bottomLayout.addWidget(self.inputLabel, 1, 0)
        bottomLayout.addWidget(self.lMotorInput, 1, 1)
        bottomLayout.addWidget(self.rMotorInput, 1, 2)
        bottomLayout.addWidget(self.PIDSwitch, 2, 1, 1, 2, Qt.AlignHCenter)
        bottomLayout.addWidget(self.speedLabel, 3, 0)
        bottomLayout.addWidget(self.lSpeedValue, 3, 1)
        bottomLayout.addWidget(self.rSpeedValue, 3, 2)
        bottomLayout.addWidget(self.positionLabel, 4, 0)
        bottomLayout.addWidget(self.lPositionValue, 4, 1)
        bottomLayout.addWidget(self.rPositionValue, 4, 2)
        bottomLayout.addWidget(self.sensorLabel, 5, 0)
        bottomLayout.addWidget(self.lSensorValue, 5, 1)
        bottomLayout.addWidget(self.rSensorValue, 5, 2)
        bottomLayout.addWidget(self.encoderLabel, 6, 0)
        bottomLayout.addWidget(self.lEncoderValue, 6, 1)
        bottomLayout.addWidget(self.rEncoderValue, 6, 2)

        chassyValues.setLayout(bottomLayout)
        layout.addWidget(chassyValues, 5, 0, 1, 2)

        # make row at the end stretch
        layout.setRowStretch(6, 1)

        self.setLayout(layout)

    def enableButtons(self):
        None

    def disableButtons(self):
        None

class ArmFrame(QFrame):

    def __init__(self, parent=None):
        super(ArmFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label = QLabel("Arm")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.baseLabel = QLabel("base")
        self.linActLabel = QLabel("actuator")
        # ccw, cw
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setFixedWidth(90)
        self.cwButton = QPushButton("CW")
        self.cwButton.setFixedWidth(90)
        # up, down
        self.upButton = QPushButton("up")
        self.upButton.setFixedWidth(90)
        self.downButton = QPushButton("down")
        self.downButton.setFixedWidth(90)
        # power
        self.powerLabel = QLabel("power:")
        self.basePowerInput = QSpinBox()
        self.basePowerInput.setMinimum(0)
        self.basePowerInput.setMaximum(100) #100 for power
        self.basePowerInput.setFixedWidth(75)
        self.linActPowerInput = QSpinBox()
        self.linActPowerInput.setMinimum(0)
        self.linActPowerInput.setMaximum(100) #100 for power
        self.linActPowerInput.setFixedWidth(75)
        # encoder
        self.encoderLabel = QLabel("encoder:")
        self.baseEncoderValue = QLineEdit()
        self.baseEncoderValue.setReadOnly(True)
        self.baseEncoderValue.setFixedWidth(75)
        self.linActEncoderValue = QLineEdit()
        self.linActEncoderValue.setReadOnly(True)
        self.linActEncoderValue.setFixedWidth(75)
        # pid
        self.pLabel = QLabel("P :")
        self.basePValue = QLineEdit()
        self.basePValue.setReadOnly(True)
        self.basePValue.setFixedWidth(75)
        self.linActPValue = QLineEdit()
        self.linActPValue.setReadOnly(True)
        self.linActPValue.setFixedWidth(75)
        self.iLabel = QLabel("I  :")
        self.baseIValue = QLineEdit()
        self.baseIValue.setReadOnly(True)
        self.baseIValue.setFixedWidth(75)
        self.linActIValue = QLineEdit()
        self.linActIValue.setReadOnly(True)
        self.linActIValue.setFixedWidth(75)
        self.dLabel = QLabel("D :")
        self.baseDValue = QLineEdit()
        self.baseDValue.setReadOnly(True)
        self.baseDValue.setFixedWidth(75)
        self.linActDValue = QLineEdit()
        self.linActDValue.setReadOnly(True)
        self.linActDValue.setFixedWidth(75)
        # ref
        self.refLabel = QLabel("ref value:")
        self.baseRefInput = QSpinBox()
        self.baseRefInput.setMinimum(-4256)
        self.baseRefInput.setMaximum(4256)
        self.baseRefInput.setFixedWidth(75)
        self.setBaseRef = QPushButton("set")
        self.setBaseRef.setFixedWidth(90)
        self.linActRefInput = QSpinBox()
        self.linActRefInput.setMinimum(160)
        self.linActRefInput.setMaximum(750)
        self.linActRefInput.setFixedWidth(75)
        self.setLinActRef = QPushButton("set")
        self.setLinActRef.setFixedWidth(90)

        # sensor
        # self.baseSensorLabel = QLabel("sensor:")
        # self.baseSensorValue = QLineEdit()
        # self.baseSensorValue.setReadOnly(True)
        # self.baseSensorValue.setFixedWidth(75)
        # magnet
        # self.magnetSwitchSwitch = QCheckBox("electro magnet on")

        # layout
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0, 1, 3, Qt.AlignHCenter)
        layout.addWidget(self.hLine, 1, 0, 1, 3)
        # labels
        layout.addWidget(self.baseLabel, 2, 1, Qt.AlignHCenter)
        layout.addWidget(self.linActLabel, 2, 2, Qt.AlignHCenter)
        # base buttons
        layout.addWidget(self.ccwButton, 3, 1)
        layout.addWidget(self.cwButton, 4, 1)
        # lin act buttons
        layout.addWidget(self.upButton, 3, 2)
        layout.addWidget(self.downButton, 4, 2)
        # power input
        layout.addWidget(self.powerLabel, 5, 0, Qt.AlignRight)
        layout.addWidget(self.basePowerInput, 5, 1)
        layout.addWidget(self.linActPowerInput, 5, 2)
        # encoder
        layout.addWidget(self.encoderLabel, 6, 0, Qt.AlignRight)
        layout.addWidget(self.baseEncoderValue, 6, 1)
        layout.addWidget(self.linActEncoderValue, 6, 2)
        # P val
        layout.addWidget(self.pLabel, 7, 0, Qt.AlignRight)
        layout.addWidget(self.basePValue, 7, 1)
        layout.addWidget(self.linActPValue, 7, 2)
        # I val
        layout.addWidget(self.iLabel, 8, 0, Qt.AlignRight)
        layout.addWidget(self.baseIValue, 8, 1)
        layout.addWidget(self.linActIValue, 8, 2)
        # D val
        layout.addWidget(self.dLabel, 9, 0, Qt.AlignRight)
        layout.addWidget(self.baseDValue, 9, 1)
        layout.addWidget(self.linActDValue, 9, 2)
        # ref
        layout.addWidget(self.refLabel, 10, 0, Qt.AlignRight)
        layout.addWidget(self.baseRefInput, 10, 1)
        layout.addWidget(self.linActRefInput, 10, 2)
        layout.addWidget(self.setBaseRef, 11, 1)
        layout.addWidget(self.setLinActRef, 11, 2)

        # make row at the end stretch
        layout.setRowStretch(12, 1)

        self.setLayout(layout)

    def enableButtons(self):
        None

    def disableButtons(self):
        None

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
        self.openButton.setFixedWidth(110)
        self.closeButton = QPushButton("close")
        self.closeButton.setFixedWidth(110)
        self.clawPowerLabel = QLabel("power:")
        self.clawPowerInput = QSpinBox()
        self.clawPowerInput.setMinimum(0)
        self.clawPowerInput.setMaximum(100) #100 for power
        self.clawPowerInput.setFixedWidth(75)
        self.clawEncoderLabel = QLabel("encoder:")
        self.clawEncoderValue = QLineEdit()
        self.clawEncoderValue.setReadOnly(True)
        self.clawEncoderValue.setFixedWidth(75)
        self.raiseButton = QPushButton("raise")
        self.raiseButton.setFixedWidth(110)
        self.lowerButton = QPushButton("lower")
        self.lowerButton.setFixedWidth(110)
        self.heightPowerLabel = QLabel("power:")
        self.heightPowerInput = QSpinBox()
        self.heightPowerInput.setMinimum(0)
        self.heightPowerInput.setMaximum(100) #100 for power
        self.heightPowerInput.setFixedWidth(75)
        self.heightEncoderLabel = QLabel("encoder:")
        self.heightEncoderValue = QLineEdit()
        self.heightEncoderValue.setReadOnly(True)
        self.heightEncoderValue.setFixedWidth(75)

        self.hLine2= QFrame()
        self.hLine2.setFrameStyle(QFrame.HLine | QFrame.Sunken)

        # layout
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine1, 1, 0, 1, 4)
        layout.addWidget(self.openButton, 2, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.closeButton, 2, 2, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.clawPowerLabel, 3, 0)
        layout.addWidget(self.clawPowerInput, 3, 1, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.clawEncoderLabel, 4, 0)
        layout.addWidget(self.clawEncoderValue, 4, 1, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.raiseButton, 5, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.lowerButton, 6, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.heightPowerLabel, 7, 0)
        layout.addWidget(self.heightPowerInput, 7, 1, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.heightEncoderLabel, 8, 0)
        layout.addWidget(self.heightEncoderValue, 8, 1, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.otherLabel, 9, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine2, 10, 0, 1, 4)

        # make row at the end stretch
        layout.setRowStretch(11, 1)

        self.setLayout(layout)

    def enableButtons(self):
        None

    def disableButtons(self):
        None

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
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.commandInput, 0, 1)
        layout.addWidget(self.sendButton, 0, 2)

        # make middle column stretch
        layout.setColumnStretch(3, 1)

        # make last row stretch
        # layout.setRowStretch(1, 1)

        self.setLayout(layout)

    def enableButtons(self):
        self.setEnabled(True)

    def disableButtons(self):
        self.setEnabled(False)

class LogSelectFrame(QFrame):

    def __init__(self, parent=None):
        super(LogSelectFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)
        
        # widgets
        self.label = QLabel("Enter commands to log:")
        self.logInput = QLineEdit()
        self.logInput.setMinimumWidth(300)
        self.logButton = QPushButton("log")

        # layout
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.logInput, 0, 1)
        layout.addWidget(self.logButton, 0, 2)

        # make middle column stretch
        layout.setColumnStretch(3, 1)

        # make last row stretch
        layout.setRowStretch(1, 1)

        self.setLayout(layout)

    def enableButtons(self):
        self.setEnabled(True)

    def disableButtons(self):
        self.setEnabled(False)

class Logger():

    def __init__(self, output, parent=None):
        self.writers = {}
        self.files = []
        self.out = output

    def openLogFile(self, code):
        """
        code should include the feedback delimeter
        """
        if code in self.writers :
            self.out("<font color=red>code already set for logging</font>")

        filename = "log_" + strftime("%H-%M-%S") + ("%s" % code) + ".csv"

        csvfile = open(filename, 'wb')

        csvwriter = csv.writer(csvfile, dialect='excel')

        self.writers[code] = csvwriter
        self.files.append(csvwriter)

    def logData(self, code, data):
        try :
            writer = self.writers[code]

            writer.writerow([data])
        except :
            self.out("<font color=red>log: no such file, or file is closed</font>")

    def closeFiles(self):
        for csvfile in self.files:
            csvfile.close()


class Controller(QObject):

    # define slots
    connectionLost = Signal()

    def __init__(self, output, logger, parent=None):
        super(Controller, self).__init__(parent)

        self.out = output
        self.serial = serial.Serial()
        self.connected = False

        self.sendLock = Lock()

        self.logger = logger

        # default states
        self.connectedToMainCPU = True
        self.printAll = False

        # init controlled values
        self.power_left = 0
        self.power_right = 0
        self.speed_right = 0
        self.speed_left = 0

        # init feedback values
        self.encoder_left = 0
        self.encoder_right = 0
        self.speed_act_left = 0
        self.speed_act_right = 0
        self.position_left = 0
        self.position_right = 0
        self.sensor_left = 0
        self.sensor_right = 0

        # main cpu
        self.encoder_base = 0
        self.encoder_arm = 0
        self.p_base = 0
        self.i_base = 0
        self.d_base = 0
        self.p_arm = 0
        self.i_arm = 0
        self.d_arm = 0

    def connectToPort(self, port, rate=POLL_RATE):
        if not self.serial.isOpen() :
            self.rate = rate
            self.port = port
            self.out("<font color=green>trying to connect to port <b>%s</b></font>" % self.port)
            # self.serial.port =self.port
            # self.serial.timeout = 0
            # self.serial.baudrate = 9600
            # self.serial.bytesize=serial.EIGHTBITS
            # self.serial.stopbits=serial.STOPBITS_ONE
            # try to connect a few times
            try:
                #self.serial.open()
                self.serial = serial.Serial(self.port, 9600, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE ) #timeout=0.1)
            except serial.SerialException as e:
                self.out("<font color=red>%s</font>" % e)

            if self.serial.isOpen():
                self.connected = True
                self.serialThread = Thread(target=self.pollSerial)
                #self.serialThread.daemon = True
                self.serial.flushInput()
                self.serialThread.start()
                return True
            else:
                self.out("<font color=blue>check if you have the correct port and try again</font>")
                self.connected = False
                return False
        else:
            self.out("<font color=red>already connected</font>")
            return True


    def disconnect(self):
        if self.serial.isOpen() :
            self.connected = False
            self.serialThread.join()    #wait for threads to end
            self.serial.flushInput()
            self.serial.close()
            self.out("<font color=green>closed serial port</font>")

    def changePollRate(self, rate):
        self.out("<font color=blue>changing poll rate to %i</font>" % rate)
        self.rate = rate

    def requestFeedback(self):
        """
        requests for feedback for all the gui items
        """
        #list of all the feedback requests to send
        requests = ['encoder_left',
                    'encoder_right',
                    'speed_act_left',
                    'speed_act_right',
                    'position_left', 
                    'position_right',
                    'sensor_left',
                    'sensor_right'
                    ]
        
        for r in requests :
            self.sendMessage(feedback[r], sendToDriver=True)

    def pollFeedback(self):
        """
        loop which polls for feedback updates at the set rate
        """
        self.out("<font color=green>starting to poll for feedback </font>")
        while(self.connected):
                self.requestFeedback()
                time.sleep(1.0/float(self.rate))

        self.out("<font color=green>stoped polling for feedback</font>")

    def sendMessage(self, code, data=0, sendToDriver=True, encoding='none'):
        """
        sends message
        valid encodings:
            *none - when no data
            *u8
            *s8
            *u16
            *s16
            *u32
            *s32
        """
        if not self.serial.isOpen:
            # make sure connection is open
            self.out("<font color=red>send error : no connection</font>")
            return

        message = ""
        if self.connectedToMainCPU and sendToDriver :
            # if command to driver and we're not direclty connected to it
            message += driver['cmd']

        message += "%02X" % code

        # check which encoding to use
        if not encoding == 'none' :
            if encoding == 'u8' :
                message += "%02X" % (int(data)&0xFF)
            if encoding == 's8' :
                message += "%02X" % (int(data)&0xFF)
            if encoding == 's16' :
                message += "%04X" % (int(data)&0xFFFF)
            if encoding == 'u16' :
                message += "%04X" % (int(data)&0xFFFF)
            if encoding == 's32' :
                message += "%08X" % (int(data)&0xFFFFFFFF)
            if encoding == 'u32' :
                message += "%08X" % (int(data)&0xFFFFFFFF)

        #before we append the EOL
        if self.printAll :
            self.out("<font color=green>sending: </font>\"<font color=blue>%s</font>\"" % message)

        # add EOL
        message += EOL

        self.sendLock.acquire()
        self.serial.write(message)
        self.sendLock.release()

    def sendCustomMessage(self, message):
        """
        sends message as given to it
        appends EOL char to the end
        """
        if not self.serial.isOpen:
            # make sure connection is open
            self.out("<font color=red>send error : no connection</font>")
            return

        #before we append the EOL
        if self.printAll :
            self.out("<font color=green>sending: </font>\"<font color=blue>%s</font>\"" % message)

        message += EOL

        self.sendLock.acquire()
        self.serial.write(message)
        self.sendLock.release()

    def pollSerial(self):
        """
        loop which continously gets the message at the serial in buffer
        delimited by newlines
        runs at full speed currently, no sleep
        """

        self.out("<font color=green>listening to serial port </font>")
        while(self.connected):
            if self.serial.isOpen():
                #keep getting messages while there is something to read
                #if performance gets bad, slow this guy down with a sleep
                #self.receiveMessage
                #self.parseMessage(self.receiveMessage())

                self.parseMessage(self.serial.readline())

                #time.sleep(0.1)
            else:
                self.out("<font color=red>connect terminated unexpectedly</font>")
                self.connected = False
                self.connectionLost.emit()

        self.out("<font color=green>stopping listening to serial port</font>")

    def receiveMessage(self):
        """
        returns the message if there was one
        message will be empty if nothing was received, or only newline was received
        """
        message = []
        byte = self.serial.read(1)
        if byte :
            if byte != '\n' : #ignore these too
                while byte != EOL :
                    message.append(byte)

        print message
        return message    


    # not the most clever implementation, but should be more readable/maintainable
    def parseMessage(self, line):
        """returns True if a message was receive, False if it was empty"""

        # copy for parsing
        message = copy.copy(line)
        message = message.rstrip() #strip of all spaces

        # check for empty message
        if not message :
            return False
        
        parseError = False # if there was an error during parsing

        # message must start with a feedback indicator and be long enough
        # at least 5 because, 1 for start, 2 for code, and 2 for hex byte
        if message[0] == driver['feedback'] or message[0] == mainCPU['feedback'] :
            #feedback message, check if its one the gui 
            code = ord((message[1:3].decode('hex'))) #converts to int
            if message[0] == driver['feedback'] and code in driver :
                #try unpacking the data in different ways:
                data = message[3:]

                try :
                    # expecting hex values
                    hexValue = data.decode('hex')

                    if len(hexValue) == 1 :
                        # one byte
                        data_int = unpack('!b', hexValue)[0]
                        data_uint = unpack('!B', hexValue)[0]
                    elif len(hexValue) == 2 :
                        # two bytes
                        data_int = unpack('!h', hexValue)[0]
                        data_uint = unpack('H', hexValue)[0]
                    elif len(hexValue) == 4 :
                        # four bytes
                        data_int = unpack('!i', hexValue)[0]
                        data_uint = unpack('!I', hexValue)[0]

                        # maybe 2 (16 bit) ints?
                        data_2_int = unpack('!hh', hexValue)
                        data_2_uint = unpack('!HH', hexValue)
                    elif len(hexValue) == 8 :
                        # 8 bytes
                        # should be 2 (32 bit) ints
                        data_2_int = unpack('!ii', hexValue)
                        data_2_uint = unpack('!II', hexValue)
                    else :
                        # does not match expected data format
                        parseError = True
                except error:
                    # some error trying to unpack
                    parseError = True

                # now match the data to the value
                if not parseError :

                    if code == driver['encoder_left'] :
                        self.encoder_left = data_int

                    elif code == driver['encoder_right'] :
                        self.encoder_right = data_int

                    # elif code == driver['encoder_both'] :
                    #     self.encoder_left = data_2_int[0]
                    #     self.encoder_right = data_2_int[1]

                    elif code == driver['speed_act_left'] :
                        self.speed_left = data_int

                    elif code == driver['speed_act_right'] :
                        self.speed_right = data_int

                    # elif code == driver['speed_both'] :
                    #     self.speed_left = data_2_int[0]
                    #     self.speed_right = data_2_int[1]

                    elif code == driver['position_left'] :
                        self.position_left = data_int

                    elif code == driver['position_right'] :
                        self.position_right = data_int

                    # elif code == driver['position_both'] :
                    #     self.position_left = data_2_int[0]
                    #     self.position_right = data_2_int[1]

                    elif code == driver['sensor_left'] :
                        self.sensor_left = data_uint

                    elif code == driver['sensor_right'] :
                        self.sensor_right = data_uint

                    # elif code == driver['sensor_both'] :
                    #     self.sensor_left = data_2_int[0]
                    #     self.sensor_right = data_2_int[1]

                    # elif code == driver['encoder_base'] :
                    #     self.encoder_base = data_int

                    # elif code == driver['encoder_arm'] :
                    #     self.encoder_arm = data_int

                    # elif code == driver['encoder_claw'] :
                    #     self.encoder_claw = data_int

                    # elif code == driver['encoder_claw_height'] :
                    #     self.encoder_claw_height = data_int

                    else :
                        # should not happed, but just in case
                        parseError = True

                # log
                logCode = message[0:3]
                if logCode in self.logger.writers :
                    self.logger.logData(code=("".join(logCode)), data=data_int)

            elif message[0] == mainCPU['feedback'] and code&0xF0 in mainCPU and code&0x0F in mainCPU:
                #try unpacking the data in different ways:
                data = message[3:]

                try :
                    # expecting hex values
                    hexValue = data.decode('hex')

                    if len(hexValue) == 1 :
                        # one byte
                        data_int = unpack('!b', hexValue)[0]
                        data_uint = unpack('!B', hexValue)[0]
                    elif len(hexValue) == 2 :
                        # two bytes
                        data_int = unpack('!h', hexValue)[0]
                        data_uint = unpack('H', hexValue)[0]
                    elif len(hexValue) == 4 :
                        # four bytes
                        data_int = unpack('!i', hexValue)[0]
                        data_uint = unpack('!I', hexValue)[0]

                        # maybe 2 (16 bit) ints?
                        data_2_int = unpack('!hh', hexValue)
                        data_2_uint = unpack('!HH', hexValue)
                    elif len(hexValue) == 8 :
                        # 8 bytes
                        # should be 2 (32 bit) ints
                        data_2_int = unpack('!ii', hexValue)
                        data_2_uint = unpack('!II', hexValue)
                    else :
                        # does not match expected data format
                        parseError = True
                except error:
                    # some error trying to unpack
                    parseError = True
                    print "parse error in decoding data"

                # now match the data to the value
                if not parseError :
                    motor = code&0xF0
                    feedback = code&0x0F

                    if motor == mainCPU['base'] :

                        if feedback == mainCPU['encoder']:
                            self.encoder_base = data_int
                        elif feedback == mainCPU['pid_p']:
                            self.p_base = data_int
                        elif feedback == mainCPU['pid_i']:
                            self.i_base = data_int
                        elif feedback == mainCPU['pid_d']:
                            self.d_base = data_int
                        else:
                            parseError = True

                    elif motor == mainCPU['arm'] :

                        if feedback == mainCPU['encoder']:
                            self.encoder_arm = data_int
                        elif feedback == mainCPU['pid_p']:
                            self.p_arm = data_int
                        elif feedback == mainCPU['pid_i']:
                            self.i_arm = data_int
                        elif feedback == mainCPU['pid_d']:
                            self.d_arm = data_int
                        else:
                            parseError = True

                    else :
                        # should not happed, but just in case
                        parseError = True

                # log
                logCode = message[0:3]
                if logCode in self.logger.writers :
                    self.logger.logData(code=("".join(logCode)), data=data_int)

            # print the message if print all is enabled
            # or if there was a parsing error
            if self.printAll or parseError :
                # else just print it in hex
                self.out("<font color=black><b>%s</b></font>" % message)

        else:
            #unknown message, push to output as hex string in red
            self.out("<font color=black><b>%s</b></font>" % line)

        return True

if __name__ == '__main__':
    # create the app
    app = QApplication(sys.argv)
    
    # show the form
    main = MainWindow()
    main.show()
    main.raise_()
    # run the main loop
    sys.exit(app.exec_())
