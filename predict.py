import os
import socket
import pickle


DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class Predictor():
    
    def __init__(self, modelFile):
        modelFile = os.path.join(DIR_PATH,modelFile)  
        
        self.c = None
        self.addr = None
        self.s = None
        
        with open(modelFile, 'rb') as file:
            try:
                self.model = pickle.load(file)   
            except EOFError:
                print("Error in reading the model file")            
                        
    def predict(self, X):
        return self.model.predict(X)
    
    def open_socket(self):
        # Provides prediction service for the robot manipulation script
        try:
            print("Opening socket connection")
            s = socket.socket()
            ip = '127.0.0.1'
            port = 20000
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((ip, port))
            s.listen(5)
            c, addr = s.accept()
            self.c = c
            self.addr = addr
            self.s = s
            return True
            
        except OSError as e:
            print(str(e))
            self.close_socket()
            return False
        
    def send_message(self, state = "0"):    
        
        socketOpen = False
        
        if self.s is None:
            socketOpen = self.open_socket()
            
        try:
            if socketOpen:
                print('Sending prediction result \'{}\' to 127.0.0.1:20000'.format(state))
                self.c.send(state)
                self.close_socket()
            
        except OSError:
            self.close_socket()
            
    
    def close_socket(self):
        print("Closing socket connection")
        if self.c:
            self.c.close()
        if self.s:
            self.s.close()   