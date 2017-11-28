from hmmlearn import hmm

import numpy as np
import os
from matplotlib import cm, pyplot as plt

np.random.seed(123123)

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
    
    data = [list(i) for i in zip(*data)]
    return data, label


    

def main():
    
    data, label = getData('20171128-1238.csv')
    print(str((np.shape(data))))    

#    X = np.concatenate(data)
#    lengths = [len(x) for x in data]
    
    model = hmm.GaussianHMM(n_components=1, covariance_type="diag", n_iter=1000).fit(data)  
    
    hidden_states = model.predict(data)
    
    
    print("Done")
    
    print("Transition matrix")
    print(model.transmat_)
    print()

    print("Means and vars of each hidden state")
    for i in range(model.n_components):
        print("{0}th hidden state".format(i))
        print("mean = ", model.means_[i])
        print("var = ", np.diag(model.covars_[i]))
        print()


if __name__ == '__main__':
    main()