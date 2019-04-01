#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017

@author: jaerock
"""

import rospy
import numpy as np
from net_model import NetModel
from config import Config


###############################################################################
#
class DriveRun:
    
    ###########################################################################
    # model_path = 'path_to_pretrained_model_name' excluding '.h5' or 'json'
    # data_path = 'path_to_drive_data'  e.g. ../data/2017-09-22-10-12-34-56'
    def __init__(self, model_path,model_id):
        
        self.config = Config()
        self.net_model = NetModel(model_path,model_id)   
        self.net_model.load()

   ###########################################################################
    #
    def run(self, image):
        npimg = np.expand_dims(image, axis=0)
        measurements = self.net_model.model.predict(npimg)
        measurements = measurements / self.config.raw_scale
        return measurements
