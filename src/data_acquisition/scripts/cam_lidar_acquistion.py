#!/usr/bin/env python

from termcolor import colored   
import rospy
import cv2
import os
import rospy
import datetime
import time
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from image_converter import ImageConverter
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import os

vehicle_steer = 0
xMin = 0
yMin = 0
xMax = 640
yMax = 310

ic = ImageConverter()
time=(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
rospy.init_node('cam_lidar_acquistion')
path =  (str(rospy.get_param('~Cam_lidar_images_directory')) +str(time) + '/')

if os.path.exists(path + 'camera_only'):
    print('path exists. continuing...')
else:
    os.makedirs(path + 'camera_only')

if os.path.exists(path + 'lidar_only'):
    print('path exists. continuing...')
else:
    os.makedirs(path + 'lidar_only')

camera_text = open(str(path) + 'camera_only/' + str(time) + ".txt", "w+")
lidar_text = open(str(path) + 'lidar_only/' + str(time) + ".txt", "w+")

def callback(value):
    global vehicle_steer
    vehicle_steer = value.y
    
def camera_callback(data):
    img = ic.imgmsg_to_opencv(data)
    cropImg = img[yMin:yMax,xMin:xMax]
    resize_img = cv2.resize(cropImg,(160,70))
    time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
    cv2.imwrite(str(path) + 'camera_only/' + str(time_stamp) + '.jpg',resize_img)
    camera_text.write(str(time_stamp) + ',' + str(vehicle_steer) + "\r\n")

def lidar_callback(data):
    img = ic.imgmsg_to_opencv(data)
    resize_img = cv2.resize(img,(160,70))
    time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
    cv2.imwrite(str(path) + 'lidar_only/' + str(time_stamp) + '.jpg',resize_img)
    lidar_text.write(str(time_stamp) + ',' + str(vehicle_steer) + "\r\n")

def controller(data):
    if (data.buttons[4]==1):
        print colored('Recording cam and lidar', 'green')
        rospy.Subscriber("/encoder_pulse", Vector3, callback)
        global camera_sub, lidar_sub
        camera_sub=rospy.Subscriber('/usb_cam/image_raw', Image, camera_callback)
        lidar_sub=rospy.Subscriber('/lidar_image', Image, lidar_callback)

    if (data.buttons[1]==1):
        check=0
        print colored('Not recording cam and lidar', 'red')
        lidar_sub.unregister()
        camera_sub.unregister()


def main():
   rospy.Subscriber('/joy', Joy, controller)
   rospy.spin()

if __name__ == '__main__':
    main()
