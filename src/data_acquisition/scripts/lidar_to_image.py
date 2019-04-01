#!/usr/bin/env python
from __future__ import print_function 
#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

max_dist = 6
img_width = 200
img_height = 200
img_channel = 3
num_scans = 360
 
class image_converter:
   
    def __init__(self):
        self.image_pub = rospy.Publisher("lidar_image",Image,queue_size=1)
   
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/scan",LaserScan,self.callback)

   

    def callback(self,data):
        img = np.zeros((img_height, img_width, img_channel), np.uint8)
        
	for i in range(0, num_scans):
            d = data.ranges[i]
            x = max_dist - d*math.sin(math.radians(i))
            y = max_dist - d*math.cos(math.radians(i))

 	    # scale
            ix = int(x * img_width  / (max_dist*2))
            iy = int(y * img_height / (max_dist*2)) 

            img[iy,ix] = [(data.intensities[i])*42.5, 255, d*42.5] 

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

   
def main(args):
    rospy.init_node('lidar_to_image', anonymous=True)
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
