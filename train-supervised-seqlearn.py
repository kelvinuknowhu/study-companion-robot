import seqlearn
from seqlearn import hmm

import numpy as np
import os

from logger import getLogger

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
logger = getLogger('train')

def getData(inputFile):
    
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
                    first = False
                else:   
                    lineSplit = line.split(',')
                    label.append(int(lineSplit[0]))
                    sample = lineSplit[1:]
                    temp = []
                    for s in sample:
                        if "\n" in s:
                            s = s[:-1]
                        temp.append(float(s))
                    
                    sample = temp
                    data.append(sample)
                    
    except IOError as e:
        print(str(e))
        return 0
    
    return data, label
    #return [list(i) for i in zip(*data)]

    
def main():
    
    data, label = getData('first_trial_browsing_web_5sec.txt')
    
    print(str(np.shape(data)))
    
    model = hmm.MultinomialHMM(decode='viterbi', alpha=0.01)
    
    lengths = [len(label)]    
    
    model.fit(data, label, lengths)
    
    hidden_states = model.predict(data)
    
    print(str(hidden_states))
    print("Done")
    

if __name__ == '__main__':
    main()