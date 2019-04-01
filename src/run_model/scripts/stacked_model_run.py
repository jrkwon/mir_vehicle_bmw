#!/usr/bin/env python

import datetime
import os
import cv2
import time
import rospy
import sys
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess
from std_msgs.msg import String
xMin = 0
yMin = 0
xMax = 640
yMax = 310


class NeuralControl:
    def __init__(self):
        rospy.init_node('BMW_controller')
		directory = rospy.get_param('~model_directory')
        model_id = rospy.get_param('~model_id')
        self.rate = rospy.Rate(10)
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.drive= DriveRun(directory,model_id)
        rospy.Subscriber('/image_topic_2', Image, self.controller_cb)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.recorder1)
        self.image = None
        self.image_processed = False
    def recorder1(self, image):
	cam_img = self.ic.imgmsg_to_opencv(image)
	cropImg = cam_img[yMin:yMax,xMin:xMax]
	global newimg
        newimg = cv2.resize(cropImg,(160,50))	

    def controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        img_resize = cv2.resize(img,(160,50))
	stacked_img = np.concatenate((img_resize,newimg), axis=0)
    
        self.image = self.image_process.process(stacked_img)
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
                neural_control.image_processed = False
                neural_control.rate.sleep()

    except KeyboardInterrupt:
          print ('\nShutdown requested. Exiting...')
    
