#!/usr/bin/env python

from termcolor import colored   # for installing python package for colored texted in terminal sudo pip install termcolor
import rospy
import cv2
import os
import rospy
import datetime
import time
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from image_converter import ImageConverter
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import os 
xMin = 0
yMin = 0
xMax = 640
yMax = 310
vehicle_steer = 0
ic = ImageConverter()
time=(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
rospy.init_node('stacked_image_acquistion')
path =(str(rospy.get_param('~stacked_images_directory'))  +str(time) + '/')

if os.path.exists(path + 'stacked_only'):
    print('path exists. continuing...')
else:
    os.makedirs(path + 'stacked_only')


stacked_only_text = open(str(path) + 'stacked_only/' + str(time) + ".txt", "w+")

def callback(value):
    global vehicle_steer
    #vehicle_vel = value.linear.x
    vehicle_steer = value.y
    
def camera_callback(data): 
    img1 = ic.imgmsg_to_opencv(data)
    global time_stamp
    time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
    cropImg = img1[yMin:yMax,xMin:xMax]
    global newimg
    newimg = cv2.resize(cropImg,(160,50))
    
    

def lidar_callback(data):
    img = ic.imgmsg_to_opencv(data)
    img_resize = cv2.resize(img,(160,50))
    stacked_img = np.concatenate((img_resize,newimg), axis=0)
    cv2.imwrite(str(path) + 'stacked_only/' + str(time_stamp) + '.jpg',stacked_img)
    stacked_only_text.write(str(time_stamp) + ',' + str(vehicle_steer) + "\r\n")

def controller(data):
    check=data.buttons[4]
    if (check==1):
        print colored('Recording Stacked', 'green')
        rospy.Subscriber("/encoder_pulse", Vector3, callback)
        global camera_sub, lidar_sub
        camera_sub=rospy.Subscriber('/usb_cam/image_raw', Image, camera_callback)
        lidar_sub=rospy.Subscriber('/lidar_image', Image, lidar_callback)

    if (data.buttons[1]==1):
        check=0
        print colored('Not recording Stacked', 'red')
        lidar_sub.unregister()
        camera_sub.unregister()

def main():
   rospy.Subscriber('/joy', Joy, controller)
   rospy.spin()

if __name__ == '__main__':
    main()
