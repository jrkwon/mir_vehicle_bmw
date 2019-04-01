#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 9 16:14:50 2018

@author: MIR-LAB
"""


import rospy
import numpy as np
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from drive_run import DriveRun
from image_converter import ImageConverter
from config import Config
from image_process import ImageProcess
import cv2
#image reszie parameter, same as used for training 
xMin = 0
yMin = 0
xMax = 640
yMax = 310

class NeuralControl:
    def __init__(self):
        rospy.init_node('BMW_controller')
        directory = rospy.get_param('~model_directory')
        model_id = rospy.get_param('~model_id')
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = rospy.Rate(10)
        self.drive= DriveRun(directory,model_id)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.controller_cb)
        self.image = None
        self.image_processed = False

    def controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        img=img[yMin:yMax,xMin:xMax]
        img = cv2.resize(img,(160,70))
        self.image = self.image_process.process(img)
        self.image_processed = True


if __name__ == "__main__":
    try:
        neural_control = NeuralControl()
        while not rospy.is_shutdown():
            if neural_control.image_processed == True:
                prediction = neural_control.drive.run(neural_control.image)
		print(prediction)
		joy_pub = rospy.Publisher('/joy', Joy, queue_size = 10)
	        rate = rospy.Rate(10)
      	  	joy_data = Joy()
#####################################################################################    
        	if(prediction[0][0] < 0.1 and prediction[0][0] > -0.1):
            		print("straight")
            		prediction[0][0] = prediction[0][0]
            		joy_data.axes = [0,0,0,0,0,0]
            		joy_data.buttons = [0,0,0,1,0,0,0,1,0,0,0,0]
#####################################################################################
        	elif(prediction[0][0] < 0.5 and prediction[0][0] > 0.1):
            		print("Left")
            		prediction[0][0] = prediction[0][0]
            		joy_data.axes = [0,0,0,0,1.0,0]
            		joy_data.buttons = [0,0,0,0,0,0,0,1,0,0,0,0]
#####################################################################################
        	elif(prediction[0][0] < 1 and prediction[0][0] > 0.5):
            		print("Ext.Left")
            		prediction[0][0] = prediction[0][0]
            		joy_data.axes = [0,0,0,0,0,1.0]
            		joy_data.buttons = [0,0,0,0,0,0,0,1,0,0,0,0]
#####################################################################################        
        	elif(prediction[0][0] < -0.1 and prediction[0][0] > -0.5):
            		print("Right")
            		prediction[0][0] = -prediction[0][0]
            		joy_data.axes = [0,0,0,0,-1.0,0]
            		joy_data.buttons = [0,0,0,0,0,0,0,1,0,0,0,0]
#####################################################################################
        	elif(prediction[0][0] < -0.5 and prediction[0][0] > -1):
            		print("Ext.Right")
            		prediction[0][0] = prediction[0][0]
            		joy_data.axes = [0,0,0,0,0,-1.0]
            		joy_data.buttons = [0,0,0,0,0,0,0,1,0,0,0,0]
#####################################################################################
        	joy_pub.publish(joy_data)
        	print(prediction[0][0])
		camera_pub=rospy.Publisher('/camera_prediction', Float32, queue_size = 1)
		camera_pub.publish(prediction[0][0])
                neural_control.image_processed = False
                neural_control.rate.sleep()

    except KeyboardInterrupt:
	   print ('\nShutdown requested. Exiting...')
