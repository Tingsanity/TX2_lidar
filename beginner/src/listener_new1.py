#!/usr/bin/env python3
import rospy
import math
# import tf
import threading
import numpy as np
import csv
# import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
# from pykalman import KalmanFilter
from beginner.msg import Person
import fusion

M_PI = 3.14159265358979323846


class person(threading.Thread):
    def __init__(self, sub_name, pub_name):
        threading.Thread.__init__(self)
        self.imu_data = np.zeros((1,5))
        rospy.init_node('imu_listener1')
        self.pub = rospy.Publisher(pub_name, Person, queue_size=10)
        self.sub = rospy.Subscriber(sub_name, Imu, self.imu_callback)
        self.yaw = 0.0
        self.sensor_id = pub_name
        #-----counter parameter-----
        self.window_size = 30  
        self.threshold = 3
        self.influence = 0
        self.normal = 0
        self.constant = 0.02

        self.signals = np.zeros(self.window_size)
        self.filteredY = np.zeros(self.window_size)
        self.avgFilter = np.zeros(self.window_size)
        self.stdFilter = np.zeros(self.window_size)
        self.count = 0
        self.flag = 0
        self.close = 1
        self.count_flag = 0
        self.acc = 0
        
        self.past = 0
        self.now = 0
        self.nex = 0

        #------state features-----
        self.MaxMin = 0
        self.StateSize = 50
        self.DirSize = 50
        self.StateList = np.zeros(self.StateSize)
        self.StateFlag = 0
        self.state = 50
        self.LCSS = 0
        self.states = []
        self.DirList = []
    def imu_callback(self, data):

        ori_x = data.orientation.x
        ori_y = data.orientation.y
        ori_z = data.orientation.z
        ori_w = data.orientation.w

        yaw_z = (math.atan2(2*(ori_w*ori_x+ori_y*ori_z), 1-2*(ori_x*ori_x+ori_y*ori_y))*180/M_PI+360) % 360
        pitch_y = (math.asin(2*(ori_w*ori_y-ori_z*ori_x))*180/M_PI+360) %360
        roll_x = (math.atan2(2*(ori_w*ori_z+ori_x*ori_y), 1-2*(ori_y*ori_y+ori_z*ori_z))*180/M_PI+360) %360

        #------
        q = [0, 0, 0, 0]
        g = [0, 0, 0]
        acc = [0, 0 ,0]
        linear_acc = [0, 0, 0]

        q[0] = data.orientation.w
        q[1] = data.orientation.x
        q[2] = data.orientation.y
        q[3] = data.orientation.z

        acc[0] = data.linear_acceleration.x
        acc[1] = data.linear_acceleration.y
        acc[2] = data.linear_acceleration.z

        g[0] = 2 * (q[1] * q[3] - q[0] * q[2]) *9.81
        g[1] = 2 * (q[0] * q[1] + q[2] * q[3]) *9.81
        g[2] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 9.81 

        linear_acc[0] = acc[0] - g[0]
        linear_acc[1] = acc[1] - g[1]
        linear_acc[2] = acc[2] - g[2]

        
        # print("x:{}, y:{}, Z:{}".format(roll, roll_x, yaw))
        
        # print("x:{}, y:{}, Z:{}".format(linear_acc[0], linear_acc[1], linear_acc[2]))
        # print("x:{}, y:{}, Z:{},".format(g[0], g[1], g[2]))
        # print("x:{}, y:{}, Z:{}\n".format(acc[0], acc[1], acc[2]))
        # rospy.loginfo('yaw:[{}]\npitch:[{}]\nroll:[{}] \n'.format(yaw_z, pitch_y, roll_x))

        # self.yaw = roll_x - 70 case1
        
        # !!! You have to transform the lidar coordinate to sensor coordinate 
        self.yaw = roll_x - 340
        if(self.yaw<0):
            self.yaw+=360
        self.yaw = self.dir_state(self.yaw)

        self.LCSS = self.LCSS_state(self.yaw) 
        if((self.flag <= (self.window_size-1)) & self.close):
            self.inital_threshold(linear_acc)
            self.flag = self.flag + 1
        else:
            self.update_threshold(linear_acc)

    def dir_state(self,x):
        if 337.5>=x and x>=292.5:
            return 315
        elif 292.5>=x and x>=247.5:
            return 270
        elif 247.5>=x and x>=202.5:
            return 225
        elif 202.5>=x and x>=157.5:
            return 180
        elif 157.5>=x and x>=112.5:
            return 135
        elif 112.5>x and x>=67.5:
            return 90
        elif 67.5>x and x>=22.5:
            return 45
        elif 22.5>x and x>0:
            return 360
        elif 360>=x and x>337.5:
            return 360

    def LCSS_state(self,x):
        if 337.5>=x and x>=292.5:
            return 2
        elif 292.5>=x and x>=247.5:
            return 3
        elif 247.5>=x and x>=202.5:
            return 4
        elif 202.5>=x and x>=157.5:
            return 5
        elif 157.5>=x and x>=112.5:
            return 6
        elif 112.5>x and x>=67.5:
            return 7
        elif 67.5>x and x>=22.5:
            return 8
        elif 22.5>x and x>0:
            return 9
        elif 360>=x and x>337.5:
            return 9
    #------
    def inital_threshold(self, data):
        # print("inital",self.flag)
        # rospy.loginfo(self.filteredY)
        # initial_data = data.linear_acceleration.y
        initial_data = data[1]
        self.filteredY[self.flag] = initial_data
        self.avgFilter[self.flag] = np.mean(self.filteredY)
        self.stdFilter[self.flag] = np.std(self.filteredY)   
        # print("flag{}".format(self.flag))
        # print("initial data",self.filteredY)
        # print(ospy.Subscriber("Alex", Person, self.sensor1_callback)
        self.sub_s2 = rospy.Subscriber("Ben", Person, self.sensor2_callback)
        self.sub_s3 = rospy.Subscriber("Mary", Person, self.sensor3_callback)("avgFilter{}/n".format(self.avgFilter))
        self.norospy.Subscriber("Alex", Person, self.sensor1_callback)
        self.sub_s2 = rospy.Subscriber("Ben", Person, self.sensor2_callback)
        self.sub_s3 = rospy.Subscriber("Mary", Person, self.sensor3_callback)#mal = np.mean(self.filteredY)   
        self.acc = self.filteredY[self.flag]
         
        self.MaxMin = np.max(self.filteredY) - np.min(self.filteredY)
        if(self.MaxMin>2):
            self.StateList[self.StateFlag] = 1 #1:walking
        else:
            self.StateList[self.StateFlag] = 0 #0:standing
        self.StateFlag = self.StateFlag+1
        self.DirList.append(self.yaw)
    def update_threshold(self, NewData):
        # rospy.loginfo(self.filteredY)
        # new_data = NewData.linear_acceleration.y
        new_data = NewData[1]
        self.close = 0
        if(self.flag==self.window_size):
            self.flag = self.flag - 1
        # print("flag__:{}".format(self.flag))
        # print(self.flag)
        self.past = (self.flag+self.window_size-1)%(self.window_size)
        self.now = self.flag 
        self.nex = (self.flag+1)%(self.window_size)
        self.acc = self.filteredY[self.now]    
        # print("flag:{}".format(self.flag))
        # print("NewData:{:.4f}".format(new_data))
        # print("Data[{}]:{:.4f},Data[{}]:{:.4f},Data[{}]:{:.4f}".format(past, self.filteredY[past], now, self.filteredY[now], nex, new_data))
        # print("avg:{:.4f},std:{:.4f},New-avg:{:.4f},threshold*std:{:.4f}".format(self.avgFilter[past], self.stdFilter[past], abs(self.filteredY[now] - self.avgFilter[past]), self.threshold*self.stdFilter[past]))
        if abs(self.filteredY[self.now] - self.avgFilter[self.past]) > self.threshold * self.stdFilter[self.past] + self.constant:
            if self.filteredY[self.now] > self.avgFilter[self.past]:
                if ((self.filteredY[self.now]>=new_data) & (self.filteredY[self.now]>=self.filteredY[self.past])):
                    self.count = self.count+1
                self.signals[self.now] = 1
            else:
                self.signals[self.now] = 0
            self.filteredY[self.nex] = self.influence * new_data + (1 - self.influence) * self.filteredY[self.now]
            
        else:
            self.count_flag = 0
            self.signals[self.now] = 0
            self.filteredY[self.nex] = (new_data)
        # print("signals:{}".format(self.signals[0]))
        # print("counts:{}\n".format(self.count))
        self.avgFilter[self.now] = np.mean(self.filteredY)
        self.stdFilter[self.now] = np.std(self.filteredY)
        self.flag = self.nex

        self.MaxMin = np.max(self.filteredY) - np.min(self.filteredY)
        # print("MaxMin:{}\n".format(self.MaxMin))
        # print("average:{}\n".format(self.avgFilter[self.past]))

        # --- check person is moving or not --- 
        if self.MaxMin>=2:
            # print("walking")
            self.StateList[self.StateFlag] = 1 #1:walking
        else: 
            # print("standing")
            self.StateList[self.StateFlag] = 0 #0:standing

        if(self.StateFlag==(self.StateSize-1)):
            d = np.argmax(np.bincount(self.StateList.astype(np.int32)))
            if (d==0):
                rospy.loginfo("sensor:{} standing".format(self.sensor_id))
            else:
                rospy.loginfo("sensor:{} walking".format(self.sensor_id))
            self.state = d
            self.states.append(d)
            # rospy.loginfo("sensor:{}".format(self.states))
        self.StateFlag = (self.StateFlag+1) % (self.StateSize)
        self.DirList.append(self.yaw)
        # print("dir:{}".format(self.DirList))
        if(len(self.DirList)==50):
            dir = np.argmax(np.bincount(np.array(self.DirList,dtype=int)))
            self.states.append(dir)
            self.DirList.clear()

        
    #-----
    
    def run(self):
        r = rospy.Rate(20)
        msg = Person()
        while not rospy.is_shutdown():
            header = Header(stamp=rospy.Time.now())
            header.frame_id = 'imu'

            msg.header = header
            msg.user_id = self.sensor_id
            msg.direction = self.yaw
            msg.velocity = 0
            msg.LCSS_state = self.LCSS
            self.StateList = self.StateList.astype(np.int32)
            msg.state = self.state
            msg.acc = self.acc
            msg.threshold_upper = self.avgFilter[self.past] + self.threshold * self.stdFilter[self.past]
            msg.threshold_low = self.avgFilter[self.past] - self.threshold * self.stdFilter[self.past]
            msg.avg = self.avgFilter[self.now]
            msg.signal = self.signals[self.now]

            # rospy.loginfo(msg)
            self.pub.publish(msg)
            r.sleep()


            


if __name__ =='__main__':


    p1 = person("android/imu1", "Chiu")
    p1.start()

    p = person("android/imu2", "Alex")
    p.start()

    #p2 = person("android/imu3", "Ben")
    #p2.start()

    #p3 = person("android/imu4", "Mary")
    #p3.start()
