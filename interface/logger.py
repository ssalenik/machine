from time import strftime
import csv

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

    def logData(self, code, data):
        """
        code should be a string
        data should be a list or a string
        """
        try :
            writer = self.writers[code]

            writer.writerow(data)
        except :
            self.out("<font color=red>log: no such file, or file is closed</font>")

    def closeFiles(self):
        """
        closes all log files
        nothing will be logged again until 'openLogFile' is called is desired code to log
        """
        for csvfile in self.files:
            csvfile.close()

        self.writers.clear()
        self.files = []