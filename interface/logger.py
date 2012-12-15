# python imports
from time import strftime
import csv

# local source imports
from serialcomm import *

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

        filecode = code.replace('>', 'p')
        filecode = filecode.replace('<', 'm')

        filename = "log_" + strftime("%H-%M-%S") + "_" + ("%s" % filecode) + ".csv"

        csvfile = open(filename, 'wb')

        csvwriter = csv.writer(csvfile, dialect='excel')

        self.writers[code] = csvwriter
        self.files.append(csvfile)

    def logData(message):
        """
        message stripped of all white space chars
        assumes that its either a known encoding, or a signed int (of soe length)
        """
        # assume all unknown stuff is logged as a signed int
        if len(message) > 4 :
            # log
            logCode = message[0:3]
            prepend = message[0]
            code = message[1:3]
            data = message[3:]

            if logCode in self.logger.writers :
                if isValidMessage(prepend, code, data):
                    logData(code=logCode, data = getData(prepend, code, data))
                else :
                    # assume we're getting an int of some sort

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

                        logData(code=logCode, data=data_int)
                    except:
                        None
                
    def closeFiles(self):
        """
        closes all log files
        nothing will be logged again until 'openLogFile' is called is desired code to log
        """
        for csvfile in self.files:
            csvfile.close()

        self.writers.clear()
        self.files = []