#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import serial
import serial.tools
import serial.tools.list_ports
from struct import *
#import io
import time
from threading import Thread
from threading import Lock
from PySide.QtCore import *
from PySide.QtGui import *

from serialComm import *

POLL_RATE = 20        # default serial poll rate
MAX_POLL_RATE = 100    # max serial poll rate
GUI_RATE = 25         # max gui refresh rate
MAX_LINES = 1000      # max lines in output window

# slot defines

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Command Centre - McGill - Eng Games 2013")        

        # main layout
        self.centre = CentralWidget(parent=self) 
        self.setCentralWidget(self.centre)

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

        # output
        self.output = Output()
        self.out = self.output.output

        # serial
        self.serial = serial.Serial()

        # controller
        self.controller = Controller(serial=self.serial, output=self.out)

        # layout
        self.layout = QGridLayout()
        self.layout.setVerticalSpacing(1)
        self.layout.setHorizontalSpacing(12)
        self.layout.addWidget(self.settings, 0, 0)
        self.layout.addWidget(self.controls, 1, 0)
        self.layout.addWidget(self.command, 2, 0)
        self.layout.addWidget(self.output, 0, 1, 3, 1)

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

        # start gui update thread
        self.refreshThread = Thread(target=self.refreshGUI)
        self.refreshThread.start()

    def sendCustom(self):
        self.controller.sendCustomMessage(self.command.commandInput.text())

    def stopAll(self):
        self.controller.sendCustomMessage(commands['stop_all'])

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

    def baseCCW(self):
        #TODO
        None

    def baseCW(self):
        #TODO
        None

    def stopBase(self):
        #TODO
        None

    def armUp(self):
        #TODO
        None

    def armDown(self):
        #TODO
        None

    def stopArm(self):
        #TODO
        None

    def stopChassy(self):
        if self.controls.chassy.PIDSwitch.isChecked() :
            self.controller.sendMessage(code=commands['speed_both'], data=0)
        else:
            self.controller.sendMessage(code=commands['power_both'], data=0)

    def chassyCCW(self):
        # set direction
        self.controller.sendMessage(code=commands['dir_left'], data=BACKWARD)
        self.controller.sendMessage(code=commands['dir_right'], data=FORWARD)

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=commands['speed_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['speed_right'], data=self.controls.chassy.rMotorInput.value())
        else :
            # set power
            self.controller.sendMessage(code=commands['power_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['power_right'], data=self.controls.chassy.rMotorInput.value())

    def chassyCW(self):
        # set direction
        self.controller.sendMessage(code=commands['dir_left'], data=FORWARD)
        self.controller.sendMessage(code=commands['dir_right'], data=BACKWARD)

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=commands['speed_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['speed_right'], data=self.controls.chassy.rMotorInput.value())
        else :
            # set power
            self.controller.sendMessage(code=commands['power_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['power_right'], data=self.controls.chassy.rMotorInput.value())

    def backward(self):
        # set direction
        self.controller.sendMessage(code=commands['dir_left'], data=BACKWARD)
        self.controller.sendMessage(code=commands['dir_right'], data=BACKWARD)

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=commands['speed_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['speed_right'], data=self.controls.chassy.rMotorInput.value())
        else :
            # set power
            self.controller.sendMessage(code=commands['power_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['power_right'], data=self.controls.chassy.rMotorInput.value())

    def forward(self):
        # set direction
        self.controller.sendMessage(code=commands['dir_left'], data=FORWARD)
        self.controller.sendMessage(code=commands['dir_right'], data=FORWARD)

        if self.controls.chassy.PIDSwitch.isChecked() :
            # set speed
            self.controller.sendMessage(code=commands['speed_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['speed_right'], data=self.controls.chassy.rMotorInput.value())
        else :
            # set power
            self.controller.sendMessage(code=commands['power_left'], data=self.controls.chassy.lMotorInput.value())
            self.controller.sendMessage(code=commands['power_right'], data=self.controls.chassy.rMotorInput.value())


    def PIDToggle(self, state):
        if state == Qt.CheckState.Checked:
            # set speed instead of power
            self.controls.chassy.lMotorLabel.setText("left speed:")
            self.controls.chassy.rMotorLabel.setText("right speed")
            self.controls.chassy.lMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.rMotorInput.setMaximum(0xFF) # max u8
            self.controls.chassy.lMotorInput.setValue(0)
            self.controls.chassy.rMotorInput.setValue(0)
        else:
            # set power instead of speed
            self.controls.chassy.lMotorLabel.setText("left power:")
            self.controls.chassy.rMotorLabel.setText("right power")
            self.controls.chassy.lMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.rMotorInput.setMaximum(100) # power is 100
            self.controls.chassy.lMotorInput.setValue(0)
            self.controls.chassy.rMotorInput.setValue(0)

    def refreshRequest(self):
        self.controller.requestFeedback()

    def debugRequest(self, state):
        if state == Qt.CheckState.Checked:
            # turn on all teh debugs
            self.controller.sendMessage(code=commands['debug_1'], data=1)
            self.controller.sendMessage(code=commands['debug_2'], data=1)
            self.controller.sendMessage(code=commands['debug_3'], data=1)
            self.controller.sendMessage(code=commands['debug_4'], data=1)
            self.controller.sendMessage(code=commands['debug_5'], data=1)
            self.controller.sendMessage(code=commands['debug_6'], data=1)
            self.controller.sendMessage(code=commands['debug_7'], data=1)
        else:
            # turn off debug
            self.controller.sendMessage(code=commands['debug_off'])

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
                # arm
                arm = self.controls.arm
                arm.baseEncoderValue.setText("%i" % c.encoder_base)
                arm.baseSensorValue.setText("%i" % c.sensor_base)
                arm.linActEncoderValue.setText("%i" % c.encoder_arm)
                # claw
                claw = self.controls.claw
                claw.clawEncoderValue.setText("%i" % c.encoder_claw)
                claw.heightEncoderValue.setText("%i" % c.encoder_claw_height)

            # output
            self.output.refresh()

            time.sleep(1.0/float(GUI_RATE))

class Controller(QObject):

    # define slots
    connectionLost = Signal()

    def __init__(self, output, serial, parent=None):
        super(Controller, self).__init__(parent)

        self.out = output
        self.serial = serial
        self.connected = False

        self.sendLock = Lock()

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
        self.encoder_base = 0
        self.sensor_base = 0
        self.encoder_arm = 0
        self.encoder_claw = 0
        self.encoder_claw_height = 0

    def connectToPort(self, port, rate=POLL_RATE):
        if not self.serial.isOpen() :
            self.rate = rate
            self.port = port
            self.out("<font color=green>trying to connect to port <b>%s</b></font>" % self.port)
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
                    self.out("<font color=red>could not open connection, trying again</font>")

            if self.serial.isOpen():
                self.connected = True
                self.serialThread = Thread(target=self.pollSerial)
                #self.serialThread.daemon = True

                # create text wrapper
                # when line_bufferin=True a call to write implies a flush() if it contains a newline char
                #self.sio = io.TextIOWrapper(io.BufferedRWPair(self.serial, self.serial), encoding='ascii', line_buffering=True)

                self.serialThread.start()
                return True
            else:
                self.out("<font color=red>could not open connection, check if the port is correct</font>")
                self.connected = False
                return False
        else:
            self.out("<font color=red>already connected</font>")
            return True


    def disconnect(self):
        if self.serial.isOpen() :
            self.connected = False
            self.serialThread.join()    #wait for threads to end
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
        requests = ['encoder_both', 'speed_both', 'position_both', 'sensor_both', 'encoder_base',
                    'sensor_base', 'encoder_arm', 'encoder_claw', 'encoder_claw_height']
        
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

    def sendMessage(self, code, sendToDriver=True, data=-1):
        """
        sends message
        if there is data to send, assumes its only u8 bits for now
        """
        if not self.serial.isOpen:
            # make sure connection is open
            self.out("<font color=red>send error : no connection</font>")
            return

        message = ""
        if self.connectedToMainCPU and sendToDriver :
            # if command to driver and we're not direclty connected to it
            message += commands['driver']

        message += "%02X" % code

        # right now I assume we're just sending an 8 bit signed int
        # insert if/else statements here if otherwise
        if data > -1 :
            message += "%02X" % ord(pack('!B', data&0xFF))

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
        sends message; assumes its already in hex
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
                self.parseMessage(self.receiveMessage())
                
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
            while byte != serialComm.EOL :
                message.append(byte)

        return message    


    # not the most clever implementation, but should be more readable/maintainable
    def parseMessage(self, message):
        """returns True if a message was receive, False if it was empty"""
        # check for empty message
        if not message :
            return

        parseError = False # if there was an error during parsing

        # message must start with a feedback indicator and be long enough
        # at least 5 because, 1 for start, 2 for code, and 2 for hex byte
        if message[0] == feedback['delim'] and len(message) > 4 :
            #feedback message, check if its one the gui 
            code = ord(("".join(message[1:3]).decode('hex'))) #converts to int
            if code in feedback :
                #try unpacking the data in different ways:
                data = message[2:]

                try :
                    # expecting hex values
                    hexValue = ("".join(data)).decode('hex')

                    if len(hexValue == 1) :
                        # one byte
                        data_int = unpack('!b', char)[0]
                        data_uint = unpack('!B', char)[0]
                    elif len(hexValue == 2) :
                        # two bytes
                        data_int = unpack('!h', ''.join(data))[0]
                        data_uint = unpack('H', ''.join(data))[0]
                    elif len(hexValue == 4) :
                        # four bytes
                        data_int = unpack('!i', ''.join(data))[0]
                        data_uint = unpack('!I', ''.join(data))[0]

                        # maybe 2 (16 bit) ints?
                        data_2_int = unpack('!hh', ''.join(data))
                        data_2_uint = unpack('!HH', ''.join(data))
                    elif len(hexValue == 8) :
                        # 8 bytes
                        # should be 2 (32 bit) ints
                        data_2_int = unpack('!ii', ''.join(data))
                        data_2_uint = unpack('!II', ''.join(data))
                    else :
                        # does not match expected data format
                        parseError = True
                except error:
                    # some error trying to unpack
                    parseError = True

                # now match the data to the value
                if not parseError :

                    if code == feedback['encoder_left'] :
                        self.encoder_left = data_int

                    elif code == feedback['encoder_right'] :
                        self.encoder_right = data_int

                    elif code == feedback['encoder_both'] :
                        self.encoder_left = data_2_int[0]
                        self.encoder_right = data_2_int[1]

                    elif code == feedback['speed_left'] :
                        self.speed_left = data_int

                    elif code == feedback['speed_right'] :
                        self.speed_right = data_int

                    elif code == feedback['speed_both'] :
                        self.speed_left = data_2_int[0]
                        self.speed_right = data_2_int[1]

                    elif code == feedback['position_left'] :
                        self.position_left = data_int

                    elif code == feedback['position_right'] :
                        self.position_right = data_int

                    elif code == feedback['position_both'] :
                        self.position_left = data_2_int[0]
                        self.position_right = data_2_int[1]

                    elif code == feedback['sensor_act_left'] :
                        self.sensor_left = data_int

                    elif code == feedback['sensor_act_right'] :
                        self.sensor_right = data_int

                    elif code == feedback['sensor_both'] :
                        self.sensor_left = data_2_int[0]
                        self.sensor_right = data_2_int[1]

                    elif code == feedback['encoder_base'] :
                        self.encoder_base = data_int

                    elif code == feedback['encoder_arm'] :
                        self.encoder_arm = data_int

                    elif code == feedback['encoder_claw'] :
                        self.encoder_claw = data_int

                    elif code == feedback['encoder_claw_height'] :
                        self.encoder_claw_height = data_int

                    else :
                        # should not happed, but just in case
                        parseError = True

            # print the message if print all is enabled
            # or if there was a parsing error
            if self.printAll or parseError :
                # else just print it in hex
                hexString = "< " + "".join(message[1:])
                self.out("<font color=black><b>%s</b></font>" % hexString.upper())

        else:
            #unknown message, push to output as hex string in red
            hexString = "".join(message)
            self.out("<font color=red><b>%s</b></font>" % hexString.upper())
        
class Output(QTextEdit):
    def __init__(self, parent=None):
        QTextEdit.__init__(self, "<b>output should go here</b>")
        
        self.setReadOnly(True)
        self.setMinimumWidth(350)

        # output lock to make outputting thread safe
        self.outputLock = Lock()
        self.newOutput = []

        # set the maximum paragraph count
        # will delete paragraphs from the beginging once the limit is reached
        self.document().setMaximumBlockCount(MAX_LINES)

    def output(self, text):
        self.outputLock.acquire()
        self.newOutput.append(text)
        self.outputLock.release()

    def refresh(self):
        self.outputLock.acquire()
        if self.newOutput :
            for text in self.newOutput :
                if text :
                    self.append(text)
            self.newOutput[:] = []
        self.outputLock.release()


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
        self.layout = QGridLayout()
        self.layout.setHorizontalSpacing(5)
        self.layout.addWidget(self.label, 0, 0)
        self.layout.addWidget(self.hLine, 1, 0, 1, 14)
        self.layout.addWidget(self.portLabel, 2, 0)
        self.layout.addWidget(self.portSelect, 2, 1, 1, 8, Qt.AlignLeft)
        self.layout.addWidget(self.connectButton, 2, 11, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.statusLabel, 2, 13, Qt.AlignHCenter)
        self.layout.addWidget(self.refreshButton, 3, 0, 1, 2, Qt.AlignLeft)
        self.layout.addWidget(self.rateSwitch, 3, 3, Qt.AlignRight)
        self.layout.addWidget(self.rateInput, 3, 4, Qt.AlignRight)
        self.layout.addWidget(self.rateUnits, 3, 5, Qt.AlignLeft)
        self.layout.addWidget(self.mcuLabel, 3, 7, Qt.AlignRight)
        self.layout.addWidget(self.mcuSelect, 3, 8, Qt.AlignLeft)
        self.layout.addWidget(self.debugSelect, 3, 10, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.printSelect, 3, 12, 1, 2, Qt.AlignHCenter)

        self.setLayout(self.layout)

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
        self.layout = QGridLayout()
        self.layout.setHorizontalSpacing(0)
        #self.layout.setContentsMargins(0,10,0,10)
        self.layout.addWidget(self.stopAllButton, 0, 0, 1, 5)
        self.layout.addWidget(self.chassy, 1, 0)
        self.layout.addWidget(self.vLine1, 1, 1)
        self.layout.addWidget(self.arm, 1, 2)
        self.layout.addWidget(self.vLine2, 1, 3)
        self.layout.addWidget(self.claw, 1, 4)

        self.setLayout(self.layout)

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
        self.lMotorLabel = QLabel("left power:")
        self.lMotorLabel.setAlignment(Qt.AlignHCenter)
        self.lMotorInput = QSpinBox()
        self.lMotorInput.setMinimum(0)
        self.lMotorInput.setMaximum(100) #100 for power
        self.lMotorInput.setFixedWidth(75)
        self.rMotorLabel = QLabel("right power:")
        self.rMotorLabel.setAlignment(Qt.AlignHCenter)
        self.rMotorInput = QSpinBox()
        self.rMotorInput.setMinimum(0)
        self.rMotorInput.setMaximum(100) #100 for power
        self.rMotorInput.setFixedWidth(75)
        self.lSpeedLabel = QLabel("left speed:")
        self.lSpeedLabel.setAlignment(Qt.AlignHCenter)
        self.lSpeedValue = QLineEdit()
        self.lSpeedValue.setReadOnly(True)
        self.lSpeedValue.setFixedWidth(75)
        self.rSpeedLabel = QLabel("right speed:")
        self.rSpeedLabel.setAlignment(Qt.AlignHCenter)
        self.rSpeedValue = QLineEdit()
        self.rSpeedValue.setReadOnly(True)
        self.rSpeedValue.setFixedWidth(75)
        self.lEncoderLabel = QLabel("left encoder:")
        self.lEncoderLabel.setAlignment(Qt.AlignHCenter)
        self.lEncoderValue = QLineEdit()
        self.lEncoderValue.setReadOnly(True)
        self.lEncoderValue.setFixedWidth(75)
        self.rEncoderLabel = QLabel("right encoder:")
        self.rEncoderLabel.setAlignment(Qt.AlignHCenter)
        self.rEncoderValue = QLineEdit()
        self.rEncoderValue.setReadOnly(True)
        self.rEncoderValue.setFixedWidth(75)
        self.lSensorLabel = QLabel("left sensor:")
        self.lSensorLabel.setAlignment(Qt.AlignHCenter)
        self.lSensorValue = QLineEdit()
        self.lSensorValue.setReadOnly(True)
        self.lSensorValue.setFixedWidth(75)
        self.rSensorLabel = QLabel("right sensor:")
        self.rSensorLabel.setAlignment(Qt.AlignHCenter)
        self.rSensorValue = QLineEdit()
        self.rSensorValue.setReadOnly(True)
        self.rSensorValue.setFixedWidth(75)

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine, 1, 0, 1, 2)
        self.layout.addWidget(self.forwardButton, 2, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.ccwButton, 3, 0, 1, 1, Qt.AlignLeft)
        self.layout.addWidget(self.cwButton, 3, 1, 1, 1, Qt.AlignRight)
        self.layout.addWidget(self.backwardButton, 4, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.lMotorLabel, 6, 0)
        self.layout.addWidget(self.rMotorLabel, 6, 1)
        self.layout.addWidget(self.lMotorInput, 7, 0, Qt.AlignHCenter)
        self.layout.addWidget(self.rMotorInput, 7, 1, Qt.AlignHCenter)
        self.layout.addWidget(self.PIDSwitch, 8, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.lSpeedLabel, 9, 0)
        self.layout.addWidget(self.lSpeedValue, 10, 0, Qt.AlignHCenter)
        self.layout.addWidget(self.rSpeedLabel, 9, 1)
        self.layout.addWidget(self.rSpeedValue, 10, 1, Qt.AlignHCenter)
        self.layout.addWidget(self.lEncoderLabel, 11, 0)
        self.layout.addWidget(self.lEncoderValue, 12, 0, Qt.AlignHCenter)
        self.layout.addWidget(self.rEncoderLabel, 11, 1)
        self.layout.addWidget(self.rEncoderValue, 12, 1, Qt.AlignHCenter)
        self.layout.addWidget(self.lSensorLabel, 13, 0)
        self.layout.addWidget(self.lSensorValue, 14, 0, Qt.AlignHCenter)
        self.layout.addWidget(self.rSensorLabel, 13, 1)
        self.layout.addWidget(self.rSensorValue, 14, 1, Qt.AlignHCenter)

        # make row at the end stretch
        self.layout.setRowStretch(15, 1)

        self.setLayout(self.layout)

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
        self.ccwButton = QPushButton("CCW")
        self.ccwButton.setFixedWidth(110)
        self.cwButton = QPushButton("CW")
        self.cwButton.setFixedWidth(110)
        self.basePowerLabel = QLabel("power:")
        self.basePowerInput = QSpinBox()
        self.basePowerInput.setMinimum(0)
        self.basePowerInput.setMaximum(100) #100 for power
        self.basePowerInput.setFixedWidth(75)
        self.baseEncoderLabel = QLabel("encoder:")
        self.baseEncoderValue = QLineEdit()
        self.baseEncoderValue.setReadOnly(True)
        self.baseEncoderValue.setFixedWidth(75)
        self.baseSensorLabel = QLabel("sensor:")
        self.baseSensorValue = QLineEdit()
        self.baseSensorValue.setReadOnly(True)
        self.baseSensorValue.setFixedWidth(75)
        self.linActuatorLabel = QLabel("linear actuator")
        self.upButton = QPushButton("up")
        self.upButton.setFixedWidth(110)
        self.downButton = QPushButton("down")
        self.downButton.setFixedWidth(110)
        self.linActPowerLabel = QLabel("power:")
        self.linActPowerInput = QSpinBox()
        self.linActPowerInput.setMinimum(0)
        self.linActPowerInput.setMaximum(100) #100 for power
        self.linActPowerInput.setFixedWidth(75)
        self.linActEncoderLabel = QLabel("encoder:")
        self.linActEncoderValue = QLineEdit()
        self.linActEncoderValue.setReadOnly(True    )
        self.linActEncoderValue.setFixedWidth(75)
        self.magnetSwitchSwitch = QCheckBox("electro magnet on")

        # layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine, 1, 0, 1, 4)
        self.layout.addWidget(self.baseLabel, 2, 0, 1, 4)
        self.layout.addWidget(self.ccwButton, 3, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.cwButton, 3, 2, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.basePowerLabel, 4, 0)
        self.layout.addWidget(self.basePowerInput, 4, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.baseEncoderLabel, 5, 0)
        self.layout.addWidget(self.baseEncoderValue, 5, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.baseSensorLabel, 6, 0)
        self.layout.addWidget(self.baseSensorValue, 6, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.linActuatorLabel, 7, 0, 1, 4)
        self.layout.addWidget(self.upButton, 8, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.downButton, 9, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.linActPowerLabel, 10, 0)
        self.layout.addWidget(self.linActPowerInput, 10, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.linActEncoderLabel, 11, 0)
        self.layout.addWidget(self.linActEncoderValue, 11, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.magnetSwitchSwitch, 12, 0, 1, 4, Qt.AlignHCenter)

        # make row at the end stretch
        self.layout.setRowStretch(13, 1)

        self.setLayout(self.layout)

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
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine1, 1, 0, 1, 4)
        self.layout.addWidget(self.openButton, 2, 0, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.closeButton, 2, 2, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.clawPowerLabel, 3, 0)
        self.layout.addWidget(self.clawPowerInput, 3, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.clawEncoderLabel, 4, 0)
        self.layout.addWidget(self.clawEncoderValue, 4, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.raiseButton, 5, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.lowerButton, 6, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.heightPowerLabel, 7, 0)
        self.layout.addWidget(self.heightPowerInput, 7, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.heightEncoderLabel, 8, 0)
        self.layout.addWidget(self.heightEncoderValue, 8, 1, 1, 2, Qt.AlignHCenter)
        self.layout.addWidget(self.otherLabel, 9, 0, 1, 4, Qt.AlignHCenter)
        self.layout.addWidget(self.hLine2, 10, 0, 1, 4)

        # make row at the end stretch
        self.layout.setRowStretch(11, 1)

        self.setLayout(self.layout)

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
        self.layout = QGridLayout()
        self.layout.addWidget(self.label, 0, 0)
        self.layout.addWidget(self.commandInput, 0, 1)
        self.layout.addWidget(self.sendButton, 0, 2)

        # make middle column stretch
        self.layout.setColumnStretch(3, 1)

        # make last row stretch
        self.layout.setRowStretch(1, 1)

        self.setLayout(self.layout)

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
    # run the main loop
    sys.exit(app.exec_())
