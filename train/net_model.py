#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 14:35:34 2017

@author: jaerock
"""

from keras.models import Sequential
from keras.layers import Lambda, Dropout, Flatten, Dense
from keras.layers import Conv2D, MaxPooling2D #, Cropping2D
from keras import losses, optimizers
from config import Config

class NetModel:
    def __init__(self, model_path, model_id):
        self.model = None
        model_name = model_path[model_path.rfind('/'):] # get folder name
        self.name = model_name.strip('/')

        self.model_path = model_path
        self.config = Config()

        self._model(model_id)
        
    ###########################################################################
    def _model(self, model_id):
        
        input_shape = (self.config.image_size[1], self.config.image_size[0],
                       self.config.image_size[2])

        if (model_id=='camera_model'):
            self.model = Sequential([
                Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
                Conv2D(24, (5, 5), activation='relu', padding='valid'),
                #MaxPooling2D(pool_size=(2, 2), strides=(1, 1)),
                Conv2D(36, (5, 5), activation='relu', padding='valid'),
                #MaxPooling2D(pool_size=(2, 2)),
                Conv2D(48, (5, 5), activation='relu', padding='valid'),
                #MaxPooling2D(pool_size=(2, 2)),
                Conv2D(64, (3, 3), activation='relu', padding='valid'),
                #MaxPooling2D(pool_size=(2, 2)),
                Conv2D(64, (3, 3), activation='relu', padding='valid'),
                #MaxPooling2D(pool_size=(2, 2)),
                Flatten(),
                Dense(100, activation='relu'),
                Dense(50, activation='relu'),
                Dense(10, activation='relu'),
                Dense(self.config.num_outputs)])
            self._compile()
        if(model_id=='lidar_model'):
            self.model = Sequential([
                Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
                Conv2D(24, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(36, (3, 3), activation='relu', padding='valid'),
		        Conv2D(36, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(48, (3, 3), activation='relu', padding='valid'),
                Conv2D(48, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(64, (3, 3), activation='relu', padding='valid'),
                Conv2D(64, (3, 3), activation='relu', padding='valid'),
                Flatten(),
                Dense(512, activation='relu'),
                Dropout(0.5),
                Dense(256, activation='relu'),
                Dense(50, activation='relu'),
                Dense(self.config.num_outputs)])
            self._compile()
        if(model_id=='stacked_model'):
            self.model = Sequential([
                Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
                Conv2D(24, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(36, (3, 3), activation='relu', padding='valid'),
		        Conv2D(36, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(48, (3, 3), activation='relu', padding='valid'),
                Conv2D(48, (3, 3), activation='relu', padding='valid'),
                MaxPooling2D(pool_size=(2, 2), strides=(2, 2)),
                Conv2D(64, (3, 3), activation='relu', padding='valid'),
                Conv2D(64, (5, 5), activation='relu', padding='valid'),
                Flatten(),
                Dropout(0.5),
                Dense(512, activation='relu'),
                Dense(256, activation='relu'),
                Dense(50, activation='relu'),
                Dense(self.config.num_outputs)])
            self._compile()
        
        
    ##########################################################################
    #
    def _compile(self):
        self.model.compile(loss=losses.mean_squared_error,
                  optimizer=optimizers.Adam())


    ###########################################################################
    #
    # save model
    def save(self):
        
        json_string = self.model.to_json()
        open(self.model_path+'.json', 'w').write(json_string)
        self.model.save_weights(self.model_path+'.h5', overwrite=True)
    
    
    ###########################################################################
    # model_path = '../data/2007-09-22-12-12-12.
    def load(self):
        
        from keras.models import model_from_json
        
        self.model = model_from_json(open(self.model_path+'.json').read())
        self.model.load_weights(self.model_path+'.h5')
        self._compile()
        
    ###########################################################################
    #
    # show summary
    def summary(self):
        self.model.summary()
        
