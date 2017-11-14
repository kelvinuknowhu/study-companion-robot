import subprocess
import os
import time

from threading import Event, Thread
from threading import Timer


DIR_PATH = os.path.dirname(os.path.realpath(__file__))



def monitor_open_windows():
    
    record_open_windows()
    
    read_open_windows()
    
    

def record_open_windows():
    
    script_path = os.path.join(DIR_PATH,"applescript","windows_monitor.scpt")
    
    subprocess.check_call("osascript {0}".format(script_path),shell=True)    
    
    
def read_open_windows():
    
    data_file = os.path.join(DIR_PATH,"applescript","open_windows.txt")
    
    active_tab_URL = None
    active_tab_title = None
    open_tab_URL = []
    open_tab_title = []
    open_applications = []
    
    try:
    
    #filePath = Path(data_file)
    
        with open(data_file, 'r') as f: 

            inputType = 0

            for i, line in enumerate(f):
                
                print(line)

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

        print('Exe in reading the monitor file')
        print(str(e))

    
    

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
    
    
    monitor_open_windows()
    
    timer = RepeatedTimer(15, monitor_open_windows)
    
    # start timer
    timer.start()

    # stop timer
    #timer.stop()    

    
    
    


if __name__=='__main__':
    main()
