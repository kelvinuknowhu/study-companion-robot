import logging
import os

PATH = os.path.dirname(os.path.realpath(__file__))

def getLogger(name):
    
    # create logger
    logger = logging.getLogger(name)

    logger.setLevel(logging.DEBUG)

    # create console handler and set level to debug
    filehandler = logging.FileHandler(os.path.join(PATH,name+".log"))
    filehandler.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # add formatter to ch
    filehandler.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(filehandler)

    return logger
