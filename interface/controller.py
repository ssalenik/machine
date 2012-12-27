# python imports
from threading import Thread
from threading import Lock
import copy
import time
from struct import *

# pyside imports
from PySide.QtCore import *
from PySide.QtGui import *

# pyserial import
import serial

# local source files
from serialcomm import *
from logger import *

POLL_RATE = 20        # default serial poll rate

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
        self.encoderArmRead = False
        self.encoderBaseRead = False

        # init feedback values
        self.encoder_left = 0
        self.encoder_right = 0
        self.speed_left = 0
        self.speed_right = 0
        self.abs_position_left = 0
        self.abs_position_right = 0
        self.rel_position_left_transition = 0
        self.rel_position_right_transition = 0
        self.rel_position_left_offset = 0
        self.rel_position_right_offset = 0
        self.sensor_left = 0
        self.sensor_right = 0
        self.pos_err_left = 0
        self.pos_err_right = 0

        # main cpu
        self.encoder_base = 0
        self.encoder_arm = 0
        self.p_base = 30
        self.i_base = 50
        self.d_base = 250
        self.s_base = 0
        self.p_arm = 60
        self.i_arm = 150
        self.d_arm = 350
        self.s_arm = 0
        self.battery_status = 0

    def connectToPort(self, port, rate=POLL_RATE):
        if not self.serial.isOpen() :
            self.rate = rate
            self.port = port
            self.out("<font color=green>trying to connect to port <b>%s</b></font>" % self.port)

            try:
                #self.serial.open()
                self.serial = serial.Serial(self.port, 9600, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, timeout=0.01)
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
                    'abs_position_left', 
                    'abs_position_right',
                    'rel_position_left_transition',
                    'rel_position_right_transition',
                    'rel_position_left_offset',
                    'rel_position_right_offset',
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

    def composeMessage(self, code, data=0, sendToDriver=True, encoding='none'):
        """
        composes message without EOL
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
            return None

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

        return message

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

        message = self.composeMessage(code, data, sendToDriver, encoding)

        if not message :
            return

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
        line = ""

        self.out("<font color=green>listening to serial port </font>")
        while(self.connected):
            if self.serial.isOpen():
                #keep getting messages while there is something to read
                #if performance gets bad, slow this guy down with a sleep
                #self.receiveMessage
                #self.parseMessage(self.receiveMessage())
                line += self.serial.readline()
                if '\n' in line or EOL in line:
                    self.parseMessage(line)
                    line = ""

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

    def parseMessage(self, line):
        """returns True if a message was received, False if it was empty"""

        # copy for parsing
        message = copy.copy(line)
        message = message.rstrip() #strip of all spaces

        # check for empty message
        if not message :
            return False

        #print message
        
        parseError = False # if there was an error during parsing

        if len(message) < 4 :
            parseError = True

        else :
            prepend = message[0]
            code = message[1:3]
            data = message[3:]

        if not parseError and isValidMessage(prepend, code, data) :
            #feedback message, check if its one the gui 
            hex_code = ord((message[1:3].decode('hex'))) #converts to int

            # decodes the data
            dataDecoded = getData(prepend, code, data)

            if prepend == driver['feedback'] :

                if hex_code == driver['dir_power_both'] :
                    #TODO
                    None

                elif hex_code == driver['encoder_both'] :
                    self.encoder_left = dataDecoded[0]
                    self.encoder_right = dataDecoded[1]

                elif hex_code == driver['speed_both'] :
                    self.speed_left = dataDecoded[0]
                    self.speed_right = dataDecoded[1]

                elif hex_code == driver['acc_both'] :
                    #TODO
                    None

                elif hex_code == driver['abs_pos_both'] :
                    self.abs_position_left = dataDecoded[0]
                    self.abs_position_right = dataDecoded[1]

                elif hex_code == driver['rel_pos_both'] :
                    self.rel_position_left_transition = dataDecoded[0]
                    self.rel_position_left_offset = dataDecoded[1]
                    self.rel_position_right_transition = dataDecoded[2]
                    self.rel_position_right_offset = dataDecoded[3]

                elif hex_code == driver['sensor_both'] :
                    self.sensor_left = dataDecoded[0]
                    self.sensor_right = dataDecoded[1]

                elif hex_code == driver['pos_err_both'] :
                    self.pos_err_left = dataDecoded[0]
                    self.pos_err_right = dataDecoded[1]

            elif prepend == mainCPU['feedback'] :

                if hex_code == mainCPU['base_encoder']:
                    self.encoder_base = dataDecoded[0]
                    if not self.encoderBaseRead:
                        self.encoderBaseRead = True

                #elif hex_code == mainCPU['base_pid_p']:
                #    self.p_base = dataDecoded[0]

                #elif hex_code == mainCPU['base_pid_i']:
                #    self.i_base = dataDecoded[0]

                #elif hex_code == mainCPU['base_pid_d']:
                #    self.d_base = dataDecoded[0]

                elif hex_code == mainCPU['arm_encoder']:
                    self.encoder_arm = dataDecoded[0]
                    if not self.encoderArmRead:
                        self.encoderArmRead = True

                #elif hex_code == mainCPU['arm_pid_p']:
                #    self.p_arm = dataDecoded[0]

                #elif hex_code == mainCPU['arm_pid_i']:
                #    self.i_arm = dataDecoded[0]

                #elif hex_code == mainCPU['arm_pid_d']:
                #    self.d_arm = dataDecoded[0]
                
                if hex_code == mainCPU['battery_status']:
                    self.battery_status = dataDecoded[0]

            else :
                # should not happed, but just in case
                parseError = True

        else :
            parseError = True


        if self.printAll or parseError:
            #unknown message, push to output as hex string in red
            self.out("<font color=black><b>%s</b></font>" % line)

        self.logger.logMessage(message)

        return True
