#!/usr/bin/env python

import sys
import os
import time
import datetime
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
path =  str(os.environ['HOME']) + '/Desktop/Cam-Lidar Data/'+str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '/')

if os.path.exists(path):
    print('path exists. continuing...')
else:
    os.makedirs(path)
ranges = open(str(path) + str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")) + ".txt", "w+")


def main():
    rospy.init_node('distance_finder')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

def callback(msg):
    distance_pub = rospy.Publisher('/distance', Float32, queue_size = 1)
    rate = rospy.Rate(10)
    Distance = Float32    
    Right_distance1 = msg.ranges[88]
    Right_distance2 = msg.ranges[89]
    Right_distance3 = msg.ranges[90]
    Avg_right_distance = (Right_distance1+Right_distance2+Right_distance3)/3
    #print(Avg_right_distance)
    Left_distance1 = msg.ranges[268]
    Left_distance2 = msg.ranges[269]
    Left_distance3 = msg.ranges[270]
    Avg_left_distance = (Left_distance1+Left_distance2+Left_distance3)/3
    #print(Avg_left_distance)
    Distance = Avg_right_distance-Avg_left_distance
    #Distance = Right_distance2 - Left_distance2
    if (Distance <= 0.2 and Distance >= -0.2):
        Distance = 0.0
    print(Distance)
    #distance_pub.publish(Distance)
    time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
    ranges.write(str(time_stamp) + ','+str(Avg_left_distance) +str(Avg_right_distance) +"\r\n")

if __name__ == '__main__':
    main()
