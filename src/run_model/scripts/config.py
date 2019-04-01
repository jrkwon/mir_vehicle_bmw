#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 22 21:30:08 2017

@author: jaerock
"""

###############################################################################
#
class Config:
    def __init__(self): # model_name):
        self.version = (0, 4) # version 0.4
        self.valid_rate = 0.3
        self.image_size = (160, 70, 3) # (64, 64, 3) #(320//2, 70, 3)
        self.num_outputs = 1  # steering_angle, throttle
        #self.model_name = model_name # 'torcs_2017-05-31-20-49-09'
        self.fname_ext = '.jpg'
        self.num_epochs = 20
        self.batch_size = 16 #64
        self.raw_scale = 1.0 # Multiply raw input by this scale
        self.jitter_tolerance = 0.009 # joystick jitter
        self.capture_area = (0,402,800,800)
        self.capture_size = (self.capture_area[3]-self.capture_area[1], 
                             self.capture_area[2]-self.capture_area[0], 3)
