#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 22 21:29:53 2017

@author: jaerock
"""

import matplotlib.pyplot as plt
import cv2
import numpy as np
import sklearn
import sys
from  net_model import NetModel
from drive_data import DriveData
from config import Config
from image_process import ImageProcess



 
###############################################################################
#
class DriveTrain:
    
    ###########################################################################
    # data_path = 'path_to_drive_data'  e.g. ../data/2017-09-22-10-12-34-56'
    def __init__(self, data_path):
        model_name = data_path[data_path.rfind('/'):] # get folder name
        model_name = model_name.strip('/')
        csv_path = data_path + '/' + model_name + '.csv' # use it for csv file name 
        
        self.csv_path = csv_path
        self.train_generator = None
        self.valid_generator = None
        self.train_hist = None
        self.drive = None
        
        self.config = Config() #model_name)
        
        self.data_path = data_path
        #self.model_name = model_name
        
        self.drive = DriveData(self.csv_path)
        self.net_model = NetModel(data_path,sys.argv[2])
        self.image_process = ImageProcess()
        
        
    ###########################################################################
    #
    def _prepare_data(self):
    
        self.drive.read()
        
        from sklearn.model_selection import train_test_split
        
        samples = list(zip(self.drive.image_names, self.drive.measurements))
        self.train_data, self.valid_data = train_test_split(samples, 
                                            test_size=self.config.valid_rate)
        
        self.num_train_samples = len(self.train_data)
        self.num_valid_samples = len(self.valid_data)
        
        print('Train samples: ', self.num_train_samples)
        print('Valid samples: ', self.num_valid_samples)
    
                                          
    ###########################################################################
    #
    def _build_model(self, show_summary=True):

        def _generator(samples, batch_size=self.config.batch_size):
            num_samples = len(samples)
            while True: # Loop forever so the generator never terminates
                samples = sklearn.utils.shuffle(samples)
                for offset in range(0, num_samples, batch_size):
                    batch_samples = samples[offset:offset+batch_size]
        
                    images = []
                    measurements = []
                    for image_name, measurement in batch_samples:
                        image_path = self.data_path + '/' + image_name + \
                                     self.config.fname_ext
                        image = cv2.imread(image_path)
                        #imageplot = plt.imshow(image)
                        image = cv2.resize(image, (self.config.image_size[0],
                                                   self.config.image_size[1]))
                        image = self.image_process.process(image)
                        images.append(image)
        
                        steering_angle, throttle = measurement
                        
                        if abs(steering_angle) < self.config.jitter_tolerance:
                            steering_angle = 0
                            
                        #measurements.append(measurement)
                        #measurements.append(steering_angle)
                        measurements.append(steering_angle*self.config.raw_scale)
                        # add the flipped image of the original
                        images.append(cv2.flip(image,1))
                        measurement = (steering_angle*-1.0, measurement[1]) 
                        #measurements.append(measurement)
                        #measurements.append(steering_angle*-1.0)
                        measurements.append(steering_angle*self.config.raw_scale*-1.0)
        
                        #print(image_path, steering_angle)
                    X_train = np.array(images)
                    y_train = np.array(measurements)

                    yield sklearn.utils.shuffle(X_train, y_train)
        
        self.train_generator = _generator(self.train_data)
        self.valid_generator = _generator(self.valid_data)
        
        if (show_summary):
            self.net_model.model.summary()
    
    ###########################################################################
    #
    def _start_training(self):
        
        if (self.train_generator == None):
            raise NameError('Generators are not ready.')
        
        ######################################################################
        # callbacks
        from keras.callbacks import ModelCheckpoint, EarlyStopping
        
        # checkpoint
        callbacks = []
        checkpoint = ModelCheckpoint(self.net_model.name+'.h5', monitor='val_loss', 
                                     verbose=1, save_best_only=True, mode='min')
        callbacks.append(checkpoint)
        
        # early stopping
        earlystop = EarlyStopping(monitor='val_loss', min_delta=0, patience=0, 
                                  verbose=1, mode='min')
        callbacks.append(earlystop)
        
        self.train_hist = self.net_model.model.fit_generator(
                self.train_generator, 
                steps_per_epoch=self.num_train_samples//self.config.batch_size, 
                epochs=self.config.num_epochs, 
                validation_data=self.valid_generator,
                validation_steps=self.num_valid_samples//self.config.batch_size,
                verbose=1, callbacks=callbacks)
    

    ###########################################################################
    #
    def _plot_training_history(self):
    
        print(self.train_hist.history.keys())
        
        ### plot the training and validation loss for each epoch
        plt.plot(self.train_hist.history['loss'])
        plt.plot(self.train_hist.history['val_loss'])
        plt.ylabel('mse loss')
        plt.xlabel('epoch')
        plt.legend(['training set', 'validatation set'], loc='upper right')
        plt.show()
        
    ###########################################################################
    #
    def train(self, show_summary=True):
        
        self._prepare_data()
        self._build_model(show_summary)
        self._start_training()
        self.net_model.save()
        self._plot_training_history()
