import subprocess
import os
import time
from threading import Event, Thread
from threading import Timer
import requests

from logger import getLogger


DIR_PATH = os.path.dirname(os.path.realpath(__file__))
logger = getLogger('monitor')

FACIAL_EXPRESSIONS = ['attention','browFurrow','browRaise','cheekRaise','chinRaise','dimpler','eyeClosure','eyeWiden','innerBrowRaise',
                      'jawDrop','lidTighten','lipCornerDepressor','lipPress','lipPucker','lipSuck','mouthOpen','noseWrinkle','smile','smirk','upperLipRaise']

EMOTIONS = ['joy','sadness','disgust','contempt','anger','fear','surprise','valence','engagement']

WEBSITES = {'socialNetwork':['facebook','instagram','twitter','tumblr'],
           'shopping':['amazon','craigslist','bestbuy'],
           'entertainment':['youtube','reddit','buzzfeed','pinterest'],
           'general':['news','blog','board','game','shopping'],
           'work':['github','overleaf','blackboard','qualtrics','surveymonkey','stack overflow']}


useEmotion = False

PORT = 8899

    
class Monitor():
    
    def __init__(self, saveFile = None):
        # Check if the saveFile exsits, if not, create a new file
        if saveFile is None:
            self.saveFile = os.path.join(DIR_PATH,"data","test.txt")
        else:
            self.saveFile = os.path.join(DIR_PATH,"data",saveFile)
         
        print("Created a new file at: {0}".format(self.saveFile))
        open(self.saveFile, 'w').close()    
            
        # Write the header    
        header = []
        
        header.append("label")
        header.append("wordsPerMinute")
        
        for key in FACIAL_EXPRESSIONS:
            header.append(key)
        for key in WEBSITES:
            header.append("active_{0}".format(key))
        header.append("active_other")
        for key in WEBSITES:
            header.append("open_{0}".format(key))
            
        with open(self.saveFile, 'w') as f:
            out = [str(x) for x in header]
            f.write(",".join(out))
            f.write('\n')  
        
        self.last_facial_expression_stored = {}
        for key in FACIAL_EXPRESSIONS:
            self.last_facial_expression_stored[key] = 0   

                
    def run(self, state = 'working'):
        
        # Assumes that the keylogger, the web server is running
        timer = RepeatedTimer(5, self.logData, state)
        # start timer
        timer.start()
        
        var = input("Please enter 'w' for working state, and 'd' for distracted state. Enter 'x' to stop recording data")
        
        if var == "w":
            timer.stop()
            self.run('working')
        
        elif var == "d":
            timer.stop()            
            self.run('distracted')
        
        else:
            timer.stop()
        
        
    def logData(self, state):
        data = []
        ind = 0
        
        if state=="working":
            data.append(1)
        else: # distracted
            data.append(0)
        
        self.request_facial_expression()
        facial_expression = self.read_facial_expression()
        
        self.write_open_windows()
        active_window, open_windows = self.read_open_windows()
        
        words_typed_per_minute = self.read_keylogger_data()
        data.append(words_typed_per_minute)
            
        allZero = True
        for key in facial_expression:
            if not facial_expression[key] == 0:
                allZero = False

        if allZero:
            print("Facial expression not found")
            for key in FACIAL_EXPRESSIONS:
                facial_expression[key] = self.last_facial_expression_stored[key]                    
        else:
            for key in FACIAL_EXPRESSIONS:
                self.last_facial_expression_stored[key] = facial_expression[key]        
                
        for key in FACIAL_EXPRESSIONS:
            data.append(facial_expression[key])

                
        if active_window:
            categoryFound = False
            for category in WEBSITES:
                
                titleMatchFound = False
                for key in WEBSITES[category]:
                    if key in active_window.lower():
                        titleMatchFound = True
                        categoryFound = True
                        break
                
                if titleMatchFound: # Set 1 if a page is found to be in the current category
                    data.append(1)
                else:
                    data.append(0)
                    
            if categoryFound: # Category "others"
                data.append(0)
            else:
                data.append(1)
                    
        else:
            for category in WEBSITES:
                data.append(0)
            data.append(0)
            
        if open_windows:
            for category in WEBSITES:
                counter = 0
                for window in open_windows:
                    title = window.lower()
                    for key in WEBSITES[category]:
                        if key in title:
                            counter += 1
                data.append(counter)
        else:
            for key in WEBSITES:
                data.append(0)

        with open(self.saveFile,'a') as f:
            out = [str(x) for x in data]
            f.write(",".join(out))
            f.write('\n')
            print("Logged data with {0} features".format(len(data)))
            print(",".join(out))
            
        
            
    def read_keylogger_data(self):
        keyloggerData = os.path.join(DIR_PATH,"data.log")
        
        numWords = []
        elapsed_time = []
        
        try:
            with open(keyloggerData, 'r') as f: 
                counter = 0
                for line in f:
                    if line.isspace():
                        pass
                    elif "--------------------" in line:
                        pass
                    elif "Words: " in line:
                        words = line.split()
                        numWords.append(len(words))
                    elif "Elapsed Time: " in line:
                        split = line.split()
                        time = int(split[2])
                        elapsed_time.append(time)
                    else:
                        pass
                        #print("Unrecognized line: {0}".format(line))
                        
            # Get the last line
            words_per_minute = numWords[-1] / elapsed_time[-1] * 60
            return words_per_minute
            
        except IOError as e:
            print(str(e))
            return 0

    
    def request_facial_expression(self):
        requests.post('http://localhost:{0}/get-facial-expression'.format(PORT), data = {'key':'value'})
        
    def read_facial_expression(self):
        data_file = os.path.join(DIR_PATH,"web","facial_expressions.txt")        

        try:
            waitForFileGeneration(data_file)        
            facial_expressions = {}            
            
            with open(data_file, 'r') as f: 

                for i, line in enumerate(f):
                    if line.isspace():
                        pass
                    else:
                        lineSplit = line.split(":")
                        number = float(lineSplit[1].strip())
                        facial_expressions[lineSplit[0]]=number
            os.remove(data_file)
            return facial_expressions

        except OSError as e:
            print('Facial expression not recognized')
            print(str(e))
            facial_expressions = {}
            for exp in FACIAL_EXPRESSIONS:
                facial_expressions[exp] = 0            
            return facial_expressions            
                        
    def write_open_windows(self):
        script_path = os.path.join(DIR_PATH,"applescript","windows_monitor.scpt")
        subprocess.check_call("osascript {0}".format(script_path),shell=True)    

    def read_open_windows(self):
        data_file = os.path.join(DIR_PATH,"applescript","open_windows.txt")
        
        waitForFileGeneration(data_file)
        
        active_tab_URL = None
        active_tab_title = None
        open_tab_URL = []
        open_tab_title = []
        open_applications = []

        try:
            with open(data_file, 'r') as f: 
                inputType = 0
                for i, line in enumerate(f):
                    if line.isspace():
                        pass
                    elif '-----' in line:
                        inputType += 1
                    else:
                        if inputType==0: # Active chrome tab URL & title
                            if i==0:
                                active_tab_URL = line
                            else:
                                active_tab_title = line
                        elif inputType==1: # All chrome tab URLs
                            open_tab_URL.append(line)
                        elif inputType==2: # All chrome tab titles
                            open_tab_title.append(line)
                        elif inputType==3: # All applications open
                            lineSplit = line.split(",")
                            open_applications = lineSplit
            os.remove(data_file)
            
            return active_tab_title, open_tab_title

        except Error as e:
            print('Exe in reading the window monitor file')
            print(str(e))
            
            return None, None
            
            
def waitForFileGeneration(filePath,trial=3,interval=1):
    max_i = trial
    for i in range(max_i):
        try:
            with open(filePath, 'rb') as _:
                break
        except IOError:
            time.sleep(interval)
    else:
        raise IOError('Could not access {} after {} attempts'.format(filePath, str(max_i)))


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
        
        

def main():
    pass
#    monitor = Monitor(server)
#
#    timer = RepeatedTimer(30, monitor_open_windows)
#    
#    # start timer
#    timer.start()
#
#    # stop timer
#    timer.stop()        


if __name__=='__main__':
    monitor = Monitor()
    monitor.run()
    
