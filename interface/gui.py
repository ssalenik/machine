#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import serial.tools
import serial.tools.list_ports
#import io
import time
import copy
from threading import Thread
from threading import Lock
from PySide.QtCore import *
from PySide.QtGui import *
import platform

# local source files
from controller import *
from logger import *

MAX_POLL_RATE = 100   # max serial poll rate
GUI_RATE = 25         # max gui refresh rate
MAX_LINES = 1000      # max lines in output window

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
        self.baseRefInputSetPermission = True
        self.armRefInputSetPermission = True
        
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

        # melanie
        self.melanie = Melanie()
        
        # kate
        self.kate = Kate()

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
        self.controls.chassy.PIDSwitchMotors.stateChanged.connect(self.PIDToggleMotors)
        self.controls.chassy.forwardButton.pressed.connect(self.forward)
        self.controls.chassy.forwardButton.released.connect(self.stopChassy)
        self.controls.chassy.backwardButton.pressed.connect(self.backward)
        self.controls.chassy.backwardButton.released.connect(self.stopChassy)
        self.controls.chassy.cwButton.pressed.connect(self.chassyCW)
        self.controls.chassy.cwButton.released.connect(self.stopChassy)
        self.controls.chassy.ccwButton.pressed.connect(self.chassyCCW)
        self.controls.chassy.ccwButton.released.connect(self.stopChassy)

        # arm signals
        self.controls.arm.PIDSwitchArm.stateChanged.connect(self.setArmPID)
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

        # servo signals
        self.controls.claw.openButton.clicked.connect(self.clawOpen)
        self.controls.claw.closeButton.clicked.connect(self.clawClose)
        self.controls.claw.raiseButton.clicked.connect(self.clawRaise)
        self.controls.claw.lowerButton.clicked.connect(self.clawLower)
        self.controls.claw.initLaserButton.clicked.connect(self.laserInit)
        self.controls.claw.midWayButton.clicked.connect(self.laserMid)
        self.controls.claw.fullButton.clicked.connect(self.laserFull)
        self.controls.claw.initPingPongButton.clicked.connect(self.pingPongInit)
        self.controls.claw.shootButton.clicked.connect(self.pingPongShoot)
        self.controls.claw.magnetSwitch.stateChanged.connect(self.MagnetToggle)

        # command signals
        self.command.sendButton.clicked.connect(self.sendCustom)
        self.command.commandInput.returnPressed.connect(self.sendCustom)

        # logger signals
        self.logSelect.logButton.clicked.connect(self.addLoggers)
        self.logSelect.logInput.returnPressed.connect(self.addLoggers)
        self.logSelect.closeButton.clicked.connect(self.stopLogging)

        # start gui update thread
        self.refreshThread = Thread(target=self.refreshGUI)
        self.refreshThread.start()

    def addLoggers(self):
        codes = self.logSelect.logInput.text().split()
        for i in range(len(codes)):
            codes[i] = codes[i].replace(",", "")

        for code in codes:
            self.logger.openLogFile(code)

        self.logSelect.logInput.clear()

    def stopLogging(self):
        self.logger.closeFiles()

    def sendCustom(self):
        if self.command.commandInput.text() == 'melanie' :
            self.melanie.showMelanie()
        
        elif self.command.commandInput.text() == 'kate' :
            self.kate.showKate()

        else:
            self.controller.sendCustomMessage(self.command.commandInput.text())

        self.command.commandInput.clear()

    def stopAll(self):
        self.controller.sendCustomMessage(STOP_ALL)
        self.controls.arm.PIDSwitchArm.setChecked(False)

    def clawOpen(self):
        self.controller.sendMessage(code=mainCPU['claw_servo_2'], data=0x00, sendToDriver=False, encoding='u8')

    def clawClose(self):
        self.controller.sendMessage(code=mainCPU['claw_servo_2'], data=0xA0, sendToDriver=False, encoding='u8')

    def clawRaise(self):
        self.controller.sendMessage(code=mainCPU['claw_servo_1'], data=0x60, sendToDriver=False, encoding='u8')

    def clawLower(self):
        self.controller.sendMessage(code=mainCPU['claw_servo_1'], data=0xA0, sendToDriver=False, encoding='u8')
        
    def laserInit(self):
        self.controller.sendMessage(code=mainCPU['laser_servo'], data=0x04, sendToDriver=False, encoding='u8')

    def laserMid(self):
        self.controller.sendMessage(code=mainCPU['laser_servo'], data=0x3A, sendToDriver=False, encoding='u8')

    def laserFull(self):
        self.controller.sendMessage(code=mainCPU['laser_servo'], data=0x60, sendToDriver=False, encoding='u8')

    def pingPongInit(self):
        self.controller.sendMessage(code=mainCPU['ping_pong_servo'], data=0x00, sendToDriver=False, encoding='u8')

    def pingPongShoot(self):
        self.controller.sendMessage(code=mainCPU['ping_pong_servo'], data=0x32, sendToDriver=False, encoding='u8')

    def setArmPID(self, state):
        if state == Qt.CheckState.Checked:
            self.controller.sendMessage(code=mainCPU['arm_pid_on'], sendToDriver=False)
            self.controls.arm.ccwButton.setEnabled(False) 
            self.controls.arm.cwButton.setEnabled(False) 
            self.controls.arm.upButton.setEnabled(False) 
            self.controls.arm.downButton.setEnabled(False)
            self.controls.arm.linActPowerInput.setEnabled(False)
            self.controls.arm.basePowerInput.setEnabled(False)
            self.controls.arm.baseSValue.setEnabled(True)
            self.controls.arm.linActSValue.setEnabled(True)
            self.controls.arm.baseRefInput.setEnabled(True)
            self.controls.arm.linActRefInput.setEnabled(True)
            self.controls.arm.setBaseRef.setEnabled(True)
            self.controls.arm.setLinActRef.setEnabled(True)
            
        else:
            self.controller.sendMessage(code=mainCPU['arm_pid_off'], sendToDriver=False)
            self.controls.arm.ccwButton.setEnabled(True) 
            self.controls.arm.cwButton.setEnabled(True) 
            self.controls.arm.upButton.setEnabled(True) 
            self.controls.arm.downButton.setEnabled(True)
            self.controls.arm.linActPowerInput.setEnabled(True)
            self.controls.arm.basePowerInput.setEnabled(True)
            self.controls.arm.basePowerInput.setValue(40)
            self.controls.arm.linActPowerInput.setValue(40)
            self.controls.arm.baseSValue.setEnabled(False)
            self.controls.arm.linActSValue.setEnabled(False)
            self.controls.arm.baseRefInput.setEnabled(False)
            self.controls.arm.linActRefInput.setEnabled(False)
            self.controls.arm.setBaseRef.setEnabled(False)
            self.controls.arm.setLinActRef.setEnabled(False)
            
    def setArmRef(self):
        # used before to tweak PID manually for the actuator
        # self.controller.sendMessage(code=mainCPU['arm_pid_p'], sendToDriver=False, data=self.controls.arm.linActPValue.value(), encoding='s16')
        # self.controller.sendMessage(code=mainCPU['arm_pid_i'], sendToDriver=False, data=self.controls.arm.linActIValue.value(), encoding='s16')
        # self.controller.sendMessage(code=mainCPU['arm_pid_d'], sendToDriver=False, data=self.controls.arm.linActDValue.value(), encoding='s16')
        self.controller.sendMessage(code=mainCPU['arm_pid_s'], sendToDriver=False, data=self.controls.arm.linActSValue.value(), encoding='s16')
        self.controller.sendMessage(code=mainCPU['arm_encoder'], sendToDriver=False, data=self.controls.arm.linActRefInput.value(), encoding='s16')

    def setBaseRef(self):
        # used before to tweak PID manually for the base
        # self.controller.sendMessage(code=mainCPU['base_pid_p'], sendToDriver=False, data=self.controls.arm.basePValue.value(), encoding='s16')
        # self.controller.sendMessage(code=mainCPU['base_pid_i'], sendToDriver=False, data=self.controls.arm.baseIValue.value(), encoding='s16')
        # self.controller.sendMessage(code=mainCPU['base_pid_d'], sendToDriver=False, data=self.controls.arm.baseDValue.value(), encoding='s16')
        self.controller.sendMessage(code=mainCPU['base_pid_s'], sendToDriver=False, data=self.controls.arm.baseSValue.value(), encoding='s16')
        self.controller.sendMessage(code=mainCPU['base_encoder'], sendToDriver=False, data=self.controls.arm.baseRefInput.value(), encoding='s16')

    def baseCCW(self):
        if not self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['base_power_up'], sendToDriver=False, data=self.controls.arm.basePowerInput.value(), encoding='u8')

    def baseCW(self):
        if not self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['base_power_down'], sendToDriver=False, data=self.controls.arm.basePowerInput.value(), encoding='u8')

    def stopBase(self):
        if self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['base_pid_s'], sendToDriver=False, data=0, encoding='s16')
            self.controller.sendMessage(code=mainCPU['base_encoder'], sendToDriver=False, data=0, encoding='s16')
        else :
            self.controller.sendMessage(code=mainCPU['base_power_up'], sendToDriver=False, data=0, encoding='u8')
            self.controller.sendMessage(code=mainCPU['base_power_down'], sendToDriver=False, data=0, encoding='u8')

    def armUp(self):
        if not self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['arm_power_up'], sendToDriver=False, data=self.controls.arm.linActPowerInput.value(), encoding='u8')

    def armDown(self):
        if not self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['arm_power_down'], sendToDriver=False, data=self.controls.arm.linActPowerInput.value(), encoding='u8')

    def stopArm(self):
        if self.controls.arm.PIDSwitchArm.isChecked() :
            self.controller.sendMessage(code=mainCPU['arm_pid_s'], sendToDriver=False, data=0, encoding='s16')
            self.controller.sendMessage(code=mainCPU['arm_encoder'], sendToDriver=False, data=self.controls.arm.linActRefInput.value(), encoding='s16')
        else:
            self.controller.sendMessage(code=mainCPU['arm_power_up'], sendToDriver=False, data=0, encoding='u8')
            self.controller.sendMessage(code=mainCPU['arm_power_down'], sendToDriver=False, data=0, encoding='u8')
        
    def stopChassy(self):
        if self.controls.chassy.PIDSwitchMotors.isChecked() :
            self.controller.sendMessage(code=driver['set_speed_both'], data=0, encoding='u8')
        else:
            self.controller.sendMessage(code=driver['set_power_both'], data=0, encoding='u8')

    def chassyCCW(self):
        # set direction
        self.controller.sendMessage(code=driver['set_dir_left'], data=BACKWARD, encoding='u8')
        self.controller.sendMessage(code=driver['set_dir_right'], data=FORWARD, encoding='u8')

        if self.controls.chassy.PIDSwitchMotors.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['set_speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['set_power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def chassyCW(self):
        # set direction
        self.controller.sendMessage(code=driver['set_dir_left'], data=FORWARD, encoding='u8')
        self.controller.sendMessage(code=driver['set_dir_right'], data=BACKWARD, encoding='u8')

        if self.controls.chassy.PIDSwitchMotors.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['set_speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['set_power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def backward(self):
        # set direction
        self.controller.sendMessage(code=driver['set_dir_left'], data=BACKWARD, encoding='u8')
        self.controller.sendMessage(code=driver['set_dir_right'], data=BACKWARD, encoding='u8')

        if self.controls.chassy.PIDSwitchMotors.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['set_speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['set_power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def forward(self):
        # set direction
        self.controller.sendMessage(code=driver['set_dir_left'], data=FORWARD, encoding='u8')
        self.controller.sendMessage(code=driver['set_dir_right'], data=FORWARD, encoding='u8')

        if self.controls.chassy.PIDSwitchMotors.isChecked() :
            # set speed
            self.controller.sendMessage(code=driver['set_speed_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_speed_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')
        else :
            # set power
            self.controller.sendMessage(code=driver['set_power_left'], data=self.controls.chassy.lMotorInput.value(), encoding='u8')
            self.controller.sendMessage(code=driver['set_power_right'], data=self.controls.chassy.rMotorInput.value(), encoding='u8')

    def PIDToggleMotors(self, state):
        if state == Qt.CheckState.Checked:
            # set speed instead of power
            self.controller.sendMessage(code=driver['pid_toggle'], data = 0x01, encoding = 'u8')
            self.controls.chassy.inputLabel.setText("speed:")
            self.controls.chassy.lMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.rMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.lMotorInput.setValue(160)
            self.controls.chassy.rMotorInput.setValue(160)
            self.controls.claw.speedInput.setEnabled(True)
            self.controls.claw.TransitionInput.setEnabled(True)
            self.controls.claw.offsetInput.setEnabled(True)
            self.controls.claw.goToButton.setEnabled(True)
        else:
            # set power instead of speed
            self.controller.sendMessage(code=driver['pid_toggle'], data = 0x00, encoding = 'u8')
            self.controls.chassy.inputLabel.setText("power:")
            self.controls.chassy.lMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.rMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.lMotorInput.setValue(50)
            self.controls.chassy.rMotorInput.setValue(50)
            self.controls.claw.speedInput.setEnabled(False)
            self.controls.claw.TransitionInput.setEnabled(False)
            self.controls.claw.offsetInput.setEnabled(False)
            self.controls.claw.goToButton.setEnabled(False)

    def MagnetToggle(self, state):
        if state == Qt.CheckState.Checked:
            self.controller.sendMessage(code=mainCPU['magnets_on'], sendToDriver=False)
        else:
            self.controller.sendMessage(code=mainCPU['magnets_off'], sendToDriver=False)
          
    def refreshRequest(self):
        self.controller.requestFeedback()

    def debugRequest(self, state):
        if state == Qt.CheckState.Checked:
            # turn on all the debugs
            self.controller.sendMessage(code=driver['debug_1'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_2'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_3'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_4'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_5'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_6'], data=1, encoding='u8')
            self.controller.sendMessage(code=driver['debug_7'], data=1, encoding='u8')
        else:
            # turn off debug
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
                chassy.lSpeedValue.setText("%i" % c.speed_left)
                chassy.rSpeedValue.setText("%i" % c.speed_right)
                chassy.lEncoderValue.setText("%i" % c.encoder_left)
                chassy.rEncoderValue.setText("%i" % c.encoder_right)
                chassy.lSensorValue.setText("%i" % c.sensor_left)
                chassy.rSensorValue.setText("%i" % c.sensor_right)
                chassy.lPositionValue.setText("%i" % c.abs_position_left)
                chassy.rPositionValue.setText("%i" % c.abs_position_right)
                chassy.lRelPositionTransition.setText("%i" % c.rel_position_left_transition)
                chassy.rRelPositionTransition.setText("%i" % c.rel_position_right_transition)
                chassy.lRelPositionOffset.setText("%i" % c.rel_position_left_offset)
                chassy.rRelPositionOffset.setText("%i" % c.rel_position_right_offset)
                
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
                
                # init ref values only once
                if self.armRefInputSetPermission and c.encoderArmRead:
                    arm.linActRefInput.setValue(int(c.encoder_arm))
                    self.armRefInputSetPermission = False
                
                if self.baseRefInputSetPermission and c.encoderBaseRead:
                    arm.baseRefInput.setValue(int(c.encoder_base))
                    self.baseRefInputSetPermission = False
                
            # output
            self.output.refresh()

            time.sleep(1.0/float(GUI_RATE))

class Melanie(QMainWindow):
    def __init__(self, parent=None):
        super(Melanie, self).__init__(parent)
        self.setWindowTitle("Melanie Iglesias")        

        # img
        self.melanieLabel = QLabel(self)
        self.melanie_gif_01 = "m01.gif"
        self.melanieMovie = QMovie(self.melanie_gif_01)
        self.melanieLabel.setMovie(self.melanieMovie)

        self.setCentralWidget(self.melanieLabel)

    def showMelanie(self):
        if self.isVisible() == False :
            self.melanieMovie.start()
            self.show()
            self.raise_()

class Kate(QMainWindow):
    def __init__(self, parent=None):
        super(Kate, self).__init__(parent)
        self.setWindowTitle("Kate Upton")        

        # img
        self.kateLabel = QLabel(self)
        self.kate_gif_01 = "kate.gif"
        self.kateMovie = QMovie(self.kate_gif_01)
        self.kateLabel.setMovie(self.kateMovie)

        self.setCentralWidget(self.kateLabel)

    def showKate(self):
        if self.isVisible() == False :
            self.kateMovie.start()
            self.show()
            self.raise_()
        
class Output(QTextEdit):
    def __init__(self, parent=None):
        QTextEdit.__init__(self, "<b>output should go here</b>")
        
        self.setReadOnly(True)
        self.setMinimumWidth(250)

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
        # get all serial ports
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
        self.refreshButton.setFixedWidth(60)
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
        self.mcuSelect.setFixedWidth(75)
        self.mcuSelect.addItem("main CPU")
        self.mcuSelect.addItem("driver")
        self.connectButton = QPushButton("connect")
        self.connectButton.setFixedWidth(60)
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
        self.claw = ServoFrame()
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
        self.forwardButton.setFixedWidth(75)
        self.backwardButton = QPushButton("backward")
        self.backwardButton.setFixedWidth(75)
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setFixedWidth(75)
        self.cwButton = QPushButton("CW")
        self.cwButton.setFixedWidth(75)
        self.PIDSwitchMotors = QCheckBox("PID motors on")
        self.PIDSwitchMotors.setChecked(True)

        # left/right
        self.lMotorLabel = QLabel("left:")
        self.lMotorLabel.setAlignment(Qt.AlignHCenter)
        self.rMotorLabel = QLabel("right:")
        self.rMotorLabel.setAlignment(Qt.AlignHCenter)
        
        # input power/speed
        self.inputLabel = QLabel("speed:")
        self.lMotorInput = QSpinBox()
        self.lMotorInput.setMinimum(0)
        self.lMotorInput.setMaximum(0xFF) # max u8
        self.lMotorInput.setFixedWidth(60)
        self.lMotorInput.setValue(160)
        self.rMotorInput = QSpinBox()
        self.rMotorInput.setMinimum(0)
        self.rMotorInput.setMaximum(0xFF) # max u8
        self.rMotorInput.setFixedWidth(60)
        self.rMotorInput.setValue(160)
        
        # actual speed
        self.speedLabel = QLabel("speed:")
        #self.speedLabel.setAlignment(Qt.AlignHCenter)
        self.lSpeedValue = QLineEdit()
        self.lSpeedValue.setReadOnly(True)
        self.lSpeedValue.setFixedWidth(60)
        self.rSpeedValue = QLineEdit()
        self.rSpeedValue.setReadOnly(True)
        self.rSpeedValue.setFixedWidth(60)
        
        # encoder
        self.encoderLabel = QLabel("encoder:")
        #self.encoderLabel.setAlignment(Qt.AlignHCenter)
        self.lEncoderValue = QLineEdit()
        self.lEncoderValue.setReadOnly(True)
        self.lEncoderValue.setFixedWidth(60)
        self.rEncoderValue = QLineEdit()
        self.rEncoderValue.setReadOnly(True)
        self.rEncoderValue.setFixedWidth(60)
        
        # sensor
        self.sensorLabel = QLabel("sensor:")
        #self.sensorLabel.setAlignment(Qt.AlignHCenter)
        self.lSensorValue = QLineEdit()
        self.lSensorValue.setReadOnly(True)
        self.lSensorValue.setFixedWidth(60)
        self.rSensorValue = QLineEdit()
        self.rSensorValue.setReadOnly(True)
        self.rSensorValue.setFixedWidth(60)
        
        # absolute position
        self.positionLabel = QLabel("abs. pos.:")
        #self.positionLabel.setAlignment(Qt.AlignHCenter)
        self.lPositionValue = QLineEdit()
        self.lPositionValue.setReadOnly(True)
        self.lPositionValue.setFixedWidth(60)
        self.rPositionValue = QLineEdit()
        self.rPositionValue.setReadOnly(True)
        self.rPositionValue.setFixedWidth(60)
        
        # relative position transition
        self.relPositionLabelTransition = QLabel("rel. pos. trans.:")
        #self.positionLabel.setAlignment(Qt.AlignHCenter)
        self.lRelPositionTransition = QLineEdit()
        self.lRelPositionTransition.setReadOnly(True)
        self.lRelPositionTransition.setFixedWidth(60)
        self.rRelPositionTransition = QLineEdit()
        self.rRelPositionTransition.setReadOnly(True)
        self.rRelPositionTransition.setFixedWidth(60)
        
        # relative position offset
        self.relPositionLabelOffset = QLabel("rel. pos. offset.:")
        self.lRelPositionOffset = QLineEdit()
        self.lRelPositionOffset.setReadOnly(True)
        self.lRelPositionOffset.setFixedWidth(60)
        self.rRelPositionOffset = QLineEdit()
        self.rRelPositionOffset.setReadOnly(True)
        self.rRelPositionOffset.setFixedWidth(60)

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
        bottomLayout.addWidget(self.PIDSwitchMotors, 2, 1, 1, 2, Qt.AlignHCenter)
        
        bottomLayout.addWidget(self.encoderLabel, 3, 0)
        bottomLayout.addWidget(self.lEncoderValue, 3, 1)
        bottomLayout.addWidget(self.rEncoderValue, 3, 2)
        
        bottomLayout.addWidget(self.positionLabel, 4, 0)
        bottomLayout.addWidget(self.lPositionValue, 4, 1)
        bottomLayout.addWidget(self.rPositionValue, 4, 2)
        
        bottomLayout.addWidget(self.relPositionLabelTransition, 5, 0)
        bottomLayout.addWidget(self.lRelPositionTransition, 5, 1)
        bottomLayout.addWidget(self.rRelPositionTransition, 5, 2)
        
        bottomLayout.addWidget(self.relPositionLabelOffset, 6, 0)
        bottomLayout.addWidget(self.lRelPositionOffset, 6, 1)
        bottomLayout.addWidget(self.rRelPositionOffset, 6, 2)
        
        bottomLayout.addWidget(self.sensorLabel, 7, 0)
        bottomLayout.addWidget(self.lSensorValue, 7, 1)
        bottomLayout.addWidget(self.rSensorValue, 7, 2)
        
        bottomLayout.addWidget(self.speedLabel, 8, 0)
        bottomLayout.addWidget(self.lSpeedValue, 8, 1)
        bottomLayout.addWidget(self.rSpeedValue, 8, 2)

        chassyValues.setLayout(bottomLayout)
        layout.addWidget(chassyValues, 9, 0, 1, 2)

        # make row at the end stretch
        layout.setRowStretch(10, 1)

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
        self.label = QLabel("Arm control")
        self.hLine = QFrame()
        self.hLine.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.baseLabel = QLabel("base")
        self.linActLabel = QLabel("actuator")
        
        # ccw, cw
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setFixedWidth(60)
        self.ccwButton.setEnabled(False)
        self.cwButton = QPushButton("CW")
        self.cwButton.setFixedWidth(60)
        self.cwButton.setEnabled(False)
                
        # up, down
        self.upButton = QPushButton("up")
        self.upButton.setFixedWidth(60)
        self.upButton.setEnabled(False)
        self.downButton = QPushButton("down")
        self.downButton.setFixedWidth(60)
        self.downButton.setEnabled(False)
        
        # pid switch
        self.PIDSwitchArm = QCheckBox("PID arm on")
        self.PIDSwitchArm.setChecked(True)
        
        # power
        self.powerLabel = QLabel("power:")
        self.basePowerInput = QSpinBox()
        self.basePowerInput.setMinimum(0)
        self.basePowerInput.setMaximum(100) #100 for power
        self.basePowerInput.setFixedWidth(60)
        self.basePowerInput.setValue(40)
        self.basePowerInput.setEnabled(False)
        self.linActPowerInput = QSpinBox()
        self.linActPowerInput.setMinimum(0)
        self.linActPowerInput.setMaximum(100) #100 for power
        self.linActPowerInput.setFixedWidth(60)
        self.linActPowerInput.setValue(40)
        self.linActPowerInput.setEnabled(False)
        
        # encoder
        self.encoderLabel = QLabel("encoder:")
        self.baseEncoderValue = QLineEdit()
        self.baseEncoderValue.setReadOnly(True)
        self.baseEncoderValue.setFixedWidth(60)
        self.linActEncoderValue = QLineEdit()
        self.linActEncoderValue.setReadOnly(True)
        self.linActEncoderValue.setFixedWidth(60)
        
        # pid
        self.pLabel = QLabel("P :")
        self.basePValue = QLineEdit()
        self.basePValue.setFixedWidth(60)
        self.basePValue.setReadOnly(True)
        self.linActPValue = QLineEdit()
        self.linActPValue.setFixedWidth(60)
        self.linActPValue.setReadOnly(True)
        
        self.iLabel = QLabel("I  :")
        self.baseIValue = QLineEdit()
        self.baseIValue.setFixedWidth(60)
        self.baseIValue.setReadOnly(True)
        self.linActIValue = QLineEdit()       
        self.linActIValue.setFixedWidth(60)
        self.linActIValue.setReadOnly(True)
        
        self.dLabel = QLabel("D :")
        self.baseDValue = QLineEdit()      
        self.baseDValue.setFixedWidth(60)
        self.baseDValue.setReadOnly(True)
        self.linActDValue = QLineEdit()       
        self.linActDValue.setFixedWidth(60)
        self.linActDValue.setReadOnly(True)
		
        self.sLabel = QLabel("speed :")
        self.baseSValue = QSpinBox()
        self.baseSValue.setMinimum(0)
        self.baseSValue.setMaximum(100)
        self.baseSValue.setFixedWidth(60)
        self.linActSValue = QSpinBox()
        self.linActSValue.setMinimum(0)
        self.linActSValue.setMaximum(100)
        self.linActSValue.setFixedWidth(60)
		
        self.refLabel = QLabel("ref value:")
        self.baseRefInput = QSpinBox()
        self.baseRefInput.setMinimum(-4256)
        self.baseRefInput.setMaximum(4256)
        self.baseRefInput.setFixedWidth(60)
        self.linActRefInput = QSpinBox()
        self.linActRefInput.setMinimum(20)
        self.linActRefInput.setMaximum(750)
        self.linActRefInput.setFixedWidth(60)
		
        self.setBaseRef = QPushButton("set")
        self.setBaseRef.setFixedWidth(60)
		
        self.setLinActRef = QPushButton("set")
        self.setLinActRef.setFixedWidth(60)

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
        # pid switch
        layout.addWidget(self.PIDSwitchArm, 6, 1, 1, 2, Qt.AlignHCenter)        
        # encoder
        layout.addWidget(self.encoderLabel, 7, 0, Qt.AlignRight)
        layout.addWidget(self.baseEncoderValue, 7, 1)
        layout.addWidget(self.linActEncoderValue, 7, 2)
        # P val
        layout.addWidget(self.pLabel, 8, 0, Qt.AlignRight)
        layout.addWidget(self.basePValue, 8, 1)
        layout.addWidget(self.linActPValue, 8, 2)
        # I val
        layout.addWidget(self.iLabel, 9, 0, Qt.AlignRight)
        layout.addWidget(self.baseIValue, 9, 1)
        layout.addWidget(self.linActIValue, 9, 2)
        # D val
        layout.addWidget(self.dLabel, 10, 0, Qt.AlignRight)
        layout.addWidget(self.baseDValue, 10, 1)
        layout.addWidget(self.linActDValue, 10, 2)
        # speed
        layout.addWidget(self.sLabel, 11, 0, Qt.AlignRight)
        layout.addWidget(self.baseSValue, 11, 1)
        layout.addWidget(self.linActSValue, 11, 2)
        # ref
        layout.addWidget(self.refLabel, 12, 0, Qt.AlignRight)
        layout.addWidget(self.baseRefInput, 12, 1)
        layout.addWidget(self.linActRefInput, 12, 2)
        layout.addWidget(self.setBaseRef, 13, 1)
        layout.addWidget(self.setLinActRef, 13, 2)

        # make row at the end stretch
        layout.setRowStretch(14, 1)

        self.setLayout(layout)

    def enableButtons(self):
        None

    def disableButtons(self):
        None

class ServoFrame(QFrame):

    def __init__(self, parent=None):
        super(ServoFrame, self).__init__(parent)

        self.setFrameStyle(QFrame.NoFrame)

        # widgets
        self.label1 = QLabel("Claw servos")
        self.hLine1= QFrame()
        self.hLine1.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        
        # open/close
        self.openButton = QPushButton("open")
        self.openButton.setFixedWidth(60)
        self.closeButton = QPushButton("close")
        self.closeButton.setFixedWidth(60)
        
        # raise/lower
        self.raiseButton = QPushButton("raise")
        self.raiseButton.setFixedWidth(60)
        self.lowerButton = QPushButton("lower")
        self.lowerButton.setFixedWidth(60)
        
        # widgets
        self.label2 = QLabel("Laser servo")
        self.hLine2= QFrame()
        self.hLine2.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        
        # init/midWay/full
        self.initLaserButton = QPushButton("init")
        self.initLaserButton.setFixedWidth(45)
        self.midWayButton = QPushButton("mid")
        self.midWayButton.setFixedWidth(45)
        self.fullButton = QPushButton("full")
        self.fullButton.setFixedWidth(45)
        
        # widgets
        self.label3 = QLabel("Ping pong servo")
        self.hLine3= QFrame()
        self.hLine3.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        
        # init/midWay/full
        self.initPingPongButton = QPushButton("reload")
        self.initPingPongButton.setFixedWidth(60)
        self.shootButton = QPushButton("shoot")
        self.shootButton.setFixedWidth(60)
        
        # widgets
        self.label4 = QLabel("Electro Magnet")
        self.hLine4= QFrame()
        self.hLine4.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        
        # magnet
        self.magnetSwitch = QCheckBox("Electro magnet on")
        
        # go to command labels
        self.label5 = QLabel("Go to command")
        self.hLine5 = QFrame()
        self.hLine5.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        self.label6 = QLabel("speed")
        self.label7 = QLabel("transition")
        self.label8 = QLabel("offset")
        
        # go to command boxes
        self.speedInput = QSpinBox()
        self.speedInput.setMinimum(0)
        self.speedInput.setMaximum(0xFF)
        self.speedInput.setFixedWidth(45)
        self.TransitionInput = QSpinBox()
        self.TransitionInput.setMinimum(0)
        self.TransitionInput.setMaximum(47)
        self.speedInput.setFixedWidth(45)
        self.offsetInput = QSpinBox()
        self.offsetInput.setMinimum(0)
        self.offsetInput.setMaximum(255)
        self.offsetInput.setFixedWidth(45)
        
        # go to button
        self.goToButton = QPushButton("GOO!!!")
        self.goToButton.setFixedWidth(45)
        
        # layout
        layout = QGridLayout()
        
        layout.addWidget(self.label1, 0, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine1, 1, 0, 1, 4)
        layout.addWidget(self.openButton, 2, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.closeButton, 2, 2, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.raiseButton, 5, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.lowerButton, 5, 2, 1, 2, Qt.AlignHCenter)
        
        layout.addWidget(self.label2, 6, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine2, 7, 0, 1, 4)
        layout.addWidget(self.initLaserButton, 8, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.midWayButton, 8, 1, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.fullButton, 8, 2, 1, 2, Qt.AlignHCenter)
        
        layout.addWidget(self.label3, 9, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine3, 10, 0, 1, 4)
        layout.addWidget(self.initPingPongButton, 11, 0, 1, 2, Qt.AlignHCenter)
        layout.addWidget(self.shootButton, 11, 2, 1, 2, Qt.AlignHCenter)
        
        layout.addWidget(self.label4, 12, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine4, 13, 0, 1, 4)
        layout.addWidget(self.magnetSwitch, 14, 1, 1, 2)
        
        layout.addWidget(self.label5, 15, 0, 1, 4, Qt.AlignHCenter)
        layout.addWidget(self.hLine5, 16, 0, 1, 4)
        layout.addWidget(self.label6, 17, 0, Qt.AlignHCenter)
        layout.addWidget(self.label7, 17, 1, Qt.AlignHCenter)
        layout.addWidget(self.label8, 17, 2, Qt.AlignHCenter)
        
        layout.addWidget(self.speedInput, 18, 0, Qt.AlignHCenter)
        layout.addWidget(self.TransitionInput, 18, 1, Qt.AlignHCenter)
        layout.addWidget(self.offsetInput, 18, 2, Qt.AlignHCenter)
        
        layout.addWidget(self.goToButton, 19, 2, Qt.AlignHCenter)

        # make row at the end stretch
        layout.setRowStretch(20, 1)

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
        self.logInput.setMinimumWidth(200)
        self.logButton = QPushButton("log")
        self.explanation = QLabel("eg: >21, >22, <43")
        self.closeButton = QPushButton("close log files")

        # layout
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.logInput, 0, 1)
        layout.addWidget(self.logButton, 0, 2)
        layout.addWidget(self.explanation, 0, 3)
        layout.addWidget(self.closeButton, 0, 5)    

        # make middle column stretch
        layout.setColumnStretch(4, 1)

        # make last row stretch
        layout.setRowStretch(1, 1)

        self.setLayout(layout)

    def enableButtons(self):
        self.setEnabled(True)

    def disableButtons(self):
        self.setEnabled(False)

if __name__ == '__main__':
    # create the app
    app = QApplication(sys.argv)
    
    # show the form
    main = MainWindow()
    main.show()
    main.raise_()
    main.move(0, 0)
    # run the main loop
    sys.exit(app.exec_())
