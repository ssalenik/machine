# python imports
from threading import Thread
from threading import Lock
import copy
import time

# pyside imports
from PySide.QtCore import *
from PySide.QtGui import *

# pyserial import
import serial

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
                logCode = message[0:3]

                try :
                    # expecting hex values
                    hexValue = data.decode('hex')

                    if len(hexValue) == 1 :
                        # one byte
                        data_int = unpack('!b', hexValue)
                        data_uint = unpack('!B', hexValue)

                    elif len(hexValue) == 2 :
                        # two bytes
                        data_int = unpack('!h', hexValue)
                        data_uint = unpack('H', hexValue)

                    elif len(hexValue) == 3 :
                        # 3 bytes
                        data_3_int = unpack('!b', hexValue)
                        data_3_uint = unpack('B', hexValue)

                    elif len(hexValue) == 4 :
                        # four bytes
                        data_int = unpack('!i', hexValue)
                        data_uint = unpack('!I', hexValue)

                        # maybe 2 (16 bit) ints?
                        data_2_int = unpack('!hh', hexValue)
                        data_2_uint = unpack('!HH', hexValue)


                    elif len(hexValue) == 6 :
                        # u8, s16, u8, s16
                        data_mix = unpack('!BhBh', hexValue)

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

                    if code == driver['dir_power_both'] :
                        #TODO
                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_3_uint)

                    elif code == driver['encoder_both'] :
                        self.encoder_left = data_2_int[0]
                        self.encoder_right = data_2_int[1]

                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_int)

                    elif code == driver['speed_both'] :
                        self.speed_left = data_2_int[0]
                        self.speed_right = data_2_int[1]

                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_int)

                    elif code == driver['acc_both'] :
                        #TODO
                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_int)

                    elif code == driver['pos_both'] :
                        self.position_left = data_2_int[0]
                        self.position_right = data_2_int[1]

                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_int)

                    elif code == driver['transition'] :
                        #TODO

                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_mix)

                    elif code == driver['sensor_both'] :
                        self.sensor_left = data_2_uint[0]
                        self.sensor_right = data_2_uint[1]

                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_uint)

                    elif code == driver['pos_err_both'] :
                        #TODO
                        
                        if logCode in self.logger.writers :
                            self.logger.logData(code=("".join(logCode)), data=data_2_int)

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
                        data_int = unpack('!b', hexValue)
                        data_uint = unpack('!B', hexValue)
                    elif len(hexValue) == 2 :
                        # two bytes
                        data_int = unpack('!h', hexValue)
                        data_uint = unpack('H', hexValue)
                    elif len(hexValue) == 4 :
                        # four bytes
                        data_int = unpack('!i', hexValue)
                        data_uint = unpack('!I', hexValue)

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
                    logCode = message[0:3]

                    if motor == mainCPU['base'] :

                        if feedback == mainCPU['encoder']:
                            self.encoder_base = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_p']:
                            self.p_base = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_i']:
                            self.i_base = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_d']:
                            self.d_base = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)
                        else:
                            parseError = True

                    elif motor == mainCPU['arm'] :

                        if feedback == mainCPU['encoder']:
                            self.encoder_arm = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_p']:
                            self.p_arm = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_i']:
                            self.i_arm = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        elif feedback == mainCPU['pid_d']:
                            self.d_arm = data_int[0]

                            if logCode in self.logger.writers :
                                self.logger.logData(code=("".join(logCode)), data=data_int)

                        else:
                            parseError = True

                    else :
                        # should not happed, but just in case
                        parseError = True

                

            # print the message if print all is enabled
            # or if there was a parsing error
            if self.printAll or parseError :
                # else just print it in hex
                self.out("<font color=black><b>%s</b></font>" % message)

        else:
            #unknown message, push to output as hex string in red
            self.out("<font color=black><b>%s</b></font>" % line)

        if parseError :
            # didn't understand

            # assume all unknown stuff is logged as a signed int
            if len(message) > 3 :
                # log
                logCode = message[0:3]
                data = message[3:]

                if logCode in self.logger.writers :
                    try :
                        # expecting hex values
                        hexValue = data.decode('hex')

                        if len(hexValue) == 1 :
                            # one byte
                            data_int = unpack('!b', hexValue)
                        elif len(hexValue) == 2 :
                            # two bytes
                            data_int = unpack('!h', hexValue)
                        elif len(hexValue) == 4 :
                            # four bytes
                            data_int = unpack('!i', hexValue)
                        elif len(hexValue) == 8 :
                            # 8 bytes
                            # should be 2 (32 bit) ints
                            data_int = unpack('!ii', hexValue)

                        self.logger.logData(code=("".join(logCode)), data=data_int)
                    except:
                        None
        
        return True