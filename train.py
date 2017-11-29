import hmmlearn
import seqlearn
from seqlearn import hmm

import numpy as np
import os
import pandas as pd
import pickle

np.random.seed(123123)

#from logger import getLogger

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
#logger = getLogger('train')

class Trainer():
    
    def __init__(self, option="seqlearn"):        
        self.option = option
        self.data = None
        self.label = None
        self.header = None

    def getData(self,inputFile, columns_to_skip = []):
        
        inputFile = os.path.join(DIR_PATH,'data',inputFile)        

        header = []
        label = []
        data = []

        try:
            with open(inputFile, 'r') as f: 
                first = True
                for line in f:
                    if line.isspace():
                        pass
                    elif first:
                        lineSplit = line.split(',')
                        header = lineSplit
                        self.header = header
                        first = False
                    else:   
                        lineSplit = line.split(',')
                        label.append(int(lineSplit[0]))
                        # Skip the first column, as it is the class label
                        sample = lineSplit[1:]
                        temp = []
                        # Remove "\n" attached at the end of the line
                        for s in sample:
                            if "\n" in s:
                                s = s[:-1]
                            temp.append(float(s))

                        sample = temp
                        # append to the data
                        data.append(sample)

        except IOError as e:
            print(str(e))
            return 0
        
        if len(columns_to_skip) != 0:
            newData = []
            for row in data:
                newRow = []
                for i,col in enumerate(row):
                    if i in columns_to_skip:
                        pass
                    else:
                        newRow.append(col)
                newData.append(newRow)
            data = newData
        
        if not self.option == "seqlearn":
            # Transpose data to match hmmlearn format
            data = [list(i) for i in zip(*data)]
            
        self.data = data
        self.label = label
        
        return data, label
    
    
    
    def pickleModel(self, model, outputData):
        
        outputData = os.path.join(DIR_PATH,outputData)        
        
        with open(outputData, 'wb') as handle:
            pickle.dump(model, handle, protocol=pickle.HIGHEST_PROTOCOL)
    

    def run(self, inputData, outputData = None, columns_to_skip = []):
        
        data, label = self.getData(inputData, columns_to_skip = columns_to_skip)
        print("Training using option: {}".format(self.option))
        print("Input data shape: {}".format(str(np.shape(data))))  
        
        model = None
        hidden_states = None
        
        if self.option== "seqlearn": 
            # seqlearn supervised algorithm
            model = seqlearn.hmm.MultinomialHMM(decode='viterbi', alpha=0.01)
            lengths = [len(label)]    
            model.fit(data, label, lengths)
            hidden_states = model.predict(data)
            
        else: # hmmlearn's unsupervised learning algorithm
            model = hmmlearn.hmm.GaussianHMM(n_components=1, covariance_type="diag", n_iter=1000).fit(data)  
            hidden_states = model.predict(data)
            
            print("Transition matrix")
            print(model.transmat_)
            print("Means and vars of each hidden state")
            for i in range(model.n_components):
                print("{0}th hidden state".format(i))
                print("mean = ", model.means_[i])
                print("var = ", np.diag(model.covars_[i]))
            
        
        if outputData is None:
            outputData = os.path.join(DIR_PATH,'{}Model.pkl'.format(self.option))
        
        self.pickleModel(model, outputData)
            
        print("Hidden states: {}".format(str(hidden_states))) 
        print("Training done")   

        return hidden_states