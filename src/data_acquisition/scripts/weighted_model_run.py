#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import rostopic
from rostopic import  ROSTopicHz
import time
from termcolor import colored
from sensor_msgs.msg import Joy
class Server:
    def __init__(self):
        self.lidar = None
        self.camera = None
        self.lidar_pre=0
        self.i=1
        
    def lidar_callback(self, msg):
        self.lidar = msg
        self.lidar_pre=self.lidar.data
        print('Lidar-' +str(self.lidar))       

    def camera_callback(self, msg):
        self.camera = msg
        print('Camera-'+ str(self.camera))
        self.prediction()
            
    def prediction(self):
        global Prediction
        if self.lidar==None :
                if(self.i>factor):
                    self.i=1
                print("Iteration= "+str(self.i))
                sign = lambda a: (a>0) - (a<0)
                sign.lidar=sign(self.lidar_pre)
                sign.camera=sign(self.camera.data)
                if (sign.lidar ==sign.camera):
                    rest=(self.i/10.0)
                    fact=(1-rest)
                    Prediction=((rest*self.camera.data)+(self.lidar_pre*fact))
                else:
                    Prediction=self.camera.data    
                self.i += 1
        else :
            Prediction=(self.lidar.data)
            self.lidar=None
        print colored('prediction =' +str(Prediction),'green')
        steering_commands(Prediction)

def steering_commands(prediction):
    #rospy.init_node('Merged_Output', anonymous=True)
    joy_pub = rospy.Publisher('/joy', Joy, queue_size = 10)
    rate = rospy.Rate(10)
    joy_data = Joy()

    if (prediction < 0.1 and prediction > -0.1):
        print colored("straight",'red')
        joy_data.axes = [0,0,0,0,0,0]

    elif (prediction < 0.5 and prediction > 0.1):
        print colored("Left",'red')
        joy_data.axes = [0,0,0,0,1,0]

    elif (prediction < 1 and prediction > 0.5):
        print colored("Ext.Left",'red')
        joy_data.axes = [0,0,0,0,0,1.0]
        
    elif (prediction < -0.1 and prediction > -0.5):
        print colored("Right",'red')
        joy_data.axes = [0,0,0,0,-1.0,0]

    elif (prediction < -0.5 and prediction > -1):
        print colored("Ext.Right",'red')
        joy_data.axes = [0,0,0,0,0,-1.0]

    joy_data.buttons = [0,0,0,1,0,0,0,1,0,0,0,0]
    joy_pub.publish(joy_data)

def getting_rates():
    r = rostopic.ROSTopicHz(-1)
    s = rospy.Subscriber('/Lidar_Prediction', Float32, r.callback_hz, callback_args='/Lidar_Prediction')
    l = rospy.Subscriber('/Camera_Prediction', Float32, r.callback_hz, callback_args='/Camera_Prediction')
    rospy.sleep(1)
    k=r.get_hz('/Lidar_Prediction')
    print('lidar_rate')
    print(k[0])
    print('camera_rate')
    h=r.get_hz('/Camera_Prediction')
    print(h[0])
    global factor
    factor=round(h[0]/k[0],0)
    print(factor)

if __name__ == '__main__':
    rospy.init_node('combined')
    getting_rates()
    rospy.sleep(5)
    server = Server()
    rospy.Subscriber('/Lidar_Prediction', Float32 , server.lidar_callback)
    rospy.Subscriber('/Camera_Prediction', Float32, server.camera_callback)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
 	except KeyboardInterrupt:
             break
             
