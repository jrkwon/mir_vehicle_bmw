#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 22 16:50:52 2017

@author: jaerock
"""

import sys
from drive_train import DriveTrain

# to train your model  you will need to run python2 train.py "net model name " directory to dataset
#example python2 train.py lidar_net_model /home/mir-lab/Cam-Lidar\ Data/2019-03-03-17-53-35/camera_only


###############################################################################
# 
def main():
    try:
        if (len(sys.argv) != 3):
            print('Give a folder name of drive data.')
            return
        drive_train = DriveTrain(sys.argv[1])
        drive_train.train(show_summary=False)    

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
       

###############################################################################
#       
if __name__ == '__main__':
    main()
