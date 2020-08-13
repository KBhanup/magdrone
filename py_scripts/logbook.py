import datetime

class LogBook():
    '''
        Keeps a timestamped log
    '''

    def __init__(self, logFileName):
        # Get time now
        t = datetime.datetime.now()
        current_time = t.strftime("%Y-%m-%d-%H-%M-%S")

        # Create log file
        pathToLog = "Logs/" + logFileName + "_" + current_time
        self.log_file = open(pathToLog, 'w')

    def justLog(self, msg):
        t = datetime.datetime.now()
        current_time = t.strftime("%H:%M:%S.%f")
        time_stamped_msg = current_time + "\t" + msg
        self.log_file.write(time_stamped_msg)
        self.log_file.write("\n")

    def printAndLog(self, msg):
        t = datetime.datetime.now()
        current_time = t.strftime("%H:%M:%S.%f")
        time_stamped_msg = current_time + " -> " + msg
        print(time_stamped_msg)
        self.log_file.write(time_stamped_msg)
        self.log_file.write("\n")
