#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

x = 0 
# 'do nothing' means stop. if any of joystick buttons is not pressed, this means 
# sending a stop command

def controller_callback(data):
    control_pub = rospy.Publisher('/vehicle_control', UInt16, queue_size = 1)
    rate = rospy.Rate(10)
    x = UInt16()
    if(data.buttons[7] == 1.0):
        print('forward')
        x=ord('8')
    if(data.buttons[6] == 1.0):
        print('backward')
        x=ord('2')
    if(data.axes[4] == -1.0):
        print('right')
        x=ord('6')
    if(data.axes[5] == -1.0):
        print('Extreme_right')
        x=ord('3')
    if(data.axes[5] == 1.0):
        print('Extreme_left')
        x=ord('1')
    if(data.axes[4] == 1.0):
        print('left')
        x=ord('4')
    if(data.buttons[2] == 1):
        print('increase_speed')
        x=ord('+')
    if(data.buttons[0] == 1):
        print('decrease_speed')
        x=ord('-')
    if(data.buttons[3] == 1):
        print('reset')
        x=ord('*')

    control_pub.publish(x)
    
def main():
    rospy.init_node('joy2vehicle')
    rospy.Subscriber('/joy', Joy, controller_callback)
    rospy.spin()

if __name__=='__main__':
    main()
