import os
import socket
import pickle


DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class Predictor():
    
    def __init__(self, modelFile):
        modelFile = os.path.join(DIR_PATH,modelFile)  
        
        with open(modelFile, 'rb') as file:
            try:
                self.model = pickle.load(file)   
            except EOFError:
                print("Error in reading the model file")            
                        
    def predict(self, X):
        return self.model.predict(X)
    
