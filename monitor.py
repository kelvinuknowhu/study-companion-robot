import subprocess
import os
import time
from threading import Event, Thread
from threading import Timer
import requests

from logger import getLogger


DIR_PATH = os.path.dirname(os.path.realpath(__file__))
logger = getLogger('monitor')


#FACIAL_EXPRESSION_SCREEN="facial_expression_server_screen"
#KEYLOGGER_SCREEN="key_logger_screen"

#def kill_screens():
#    try:
#        subprocess.check_output("killall SCREEN", shell=True)
#    except subprocess.CalledProcessError as e:
#        print(e.output)    
#    
#def run_command_on_screen(command):
#    subprocess.run("screen -dm {0}".format(command), shell=True, check=True)
#    
#def open_new_screen(name,command=None):
#    try:
#        #subprocess.check_output("killall SCREEN", shell=True)
#        subprocess.run("screen -S {0}".format(name), shell=True, check=True)
#        
#    except subprocess.CalledProcessError as e:
#        print(e.output)            
    
#    subprocess.run("screen -S {0}".format(name), shell=True, check=True)
#    if command is not None:
#        run_command_on_screen(command)

    
class Monitor():
    
    def __init__(self):
        pass
#        kill_screens()
    
    def run(self):
        # Assumes that the keylogger, the web server is running
        timer = RepeatedTimer(20, logData)
        # start timer
        timer.start()
        
    def logData(self):
        
        request_facial_expression()
        read_facial_expression()
        
        write_open_windows()
        read_open_windows()
        
        
#    def run_keylogger_process(self):
#        open_new_screen(KEYLOGGER_SCREEN)
#        run_command_on_screen("keylogger")
#        
#    def run_facial_expression_monitor_server(self):
#        open_new_screen(FACIAL_EXPRESSION_SCREEN)
#        server_file_path = os.path.join(DIR_PATH,"web","server.py")
#        run_command_on_screen("python {0}".format(server_file_path))
    
    def request_facial_expression(self):
        requests.post('http://localhost:8889/get-facial-expression', data = {'key':'value'})
        
    def read_facial_expression(self):
        data_file = os.path.join(DIR_PATH,"web","facial_expressions.txt")        

        try:
            waitForFileGeneration(data_file)        
            facialExpressions = {}            
            
            with open(data_file, 'r') as f: 

                for i, line in enumerate(f):
                    if line.isspace():
                        pass
                    else:
                        lineSplit = line.split(":")
                        facialExpressions[lineSplit[0]]=lineSplit[1]
            os.remove(data_file)

        except OSError as e:
            print('Facial expression not recognized yet')
            print(str(e))
                        
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

        except Error as e:
            print('Exe in reading the window monitor file')
            print(str(e))
            
            
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
    main()
