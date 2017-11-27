from hmmlearn import hmm
from pomegranate import *

import numpy as np
import subprocess
import os
import time

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
    model = HiddenMarkovModel.from_samples(NormalDistribution, n_components=2, X=data)
    
    s1 = State(BernoulliDistribution(0.5))
    s2 = State(BernoulliDistribution(0.5))
    
    temp = []
    for l in label:
        if l==0:
            temp.append(s1)
        else:
            temp.append(s2)
    label = temp
    
    sequences = [np.array(row) for row in data]
    model.fit(sequences, labels=label, max_iterations=1000, algorithm='labeled')
    
    print(str((np.shape(data))))

if __name__ == '__main__':
    main()