#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 22 15:37:04 2017

@author: jaerock
"""

import pandas as pd
from progressbar import ProgressBar

class DriveData:
    
    csv_header = ['image_fname', 'steering_angle', 'throttle']
    
    def __init__(self, csv_fname):
        self.csv_fname = csv_fname
        self.df = None
        self.image_names = []
        self.measurements = []
    
    def read(self):
        self.df = pd.read_csv(self.csv_fname, names=self.csv_header, index_col=False)
        #self.fname = fname
                
        num_data = len(self.df)
        
        bar = ProgressBar()
        
        for i in bar(range(num_data)): # we don't have a title
            self.image_names.append(self.df.loc[i]['image_fname'])
            self.measurements.append((float(self.df.loc[i]['steering_angle']),
                                        float(self.df.loc[i]['throttle'])))
