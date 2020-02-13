#!/usr/bin/env python3
import rospy
import math
import pandas as pd
# import tf
import numpy as np
import editdistance
# import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
# from pykalman import KalmanFilter
from beginner.msg import Person
from obstacle_detector.msg import Obstacles,CircleObstacle,SegmentObstacle
from collections import defaultdict
from collections import Counter
from dtaidistance import dtw

    
class fusioner:
    def __init__(self):
        rospy.init_node("fusioner")
        self.sub_h = rospy.Subscriber("obstacles", Obstacles, self.human_callback)

        self.person_num = 4
        self.lidarinit = False 
        self.ids = []
        self.remove_id = []

        self.sub_s = rospy.Subscriber("Chiu", Person, self.sensor_callback)
        self.sub_s1 = rospy.Subscriber("Alex", Person, self.sensor1_callback)
        self.sub_s2 = rospy.Subscriber("Ben", Person, self.sensor2_callback)
        self.sub_s3 = rospy.Subscriber("Mary", Person, self.sensor3_callback)

        self.pub_pairing = rospy.Publisher("obstacles2", Obstacles, queue_size=10)    
        self.human_direction = np.zeros((10,1))
        self.human_direction_old = np.zeros((10,1))
        self.sensor_direction = np.zeros((10,2))
        self.Circle_vec = []
        self.Circle_vec_2 = []
        self.Circle_vec_3 = []
        self.Circle_vec_4 = []
        self.Segment_vec = []

        self.sensor_state = 10

        #----state----
        self.right = 0 
        self.left = 0
        self.DirSize = 50
        self.Direction_List = np.zeros(self.DirSize)
        self.DirFlag = 0
        self.target = 0
        self.frame = 0
        #---sensor 1----
        self.StateSize = 30
        self.State_Sensor = np.zeros(self.StateSize)
        self.State_Lidar = np.zeros(self.StateSize)
        self.State_Sflag = 0
        self.State_Lflag = 0
        self.lidar_dir = 0
        
        
        self.turn_round = defaultdict(list)
        self.human_action = {}

        self.total_sensor_states = []
        self.total_lidar_states = []
        self.total_sensor_states_LCSS = []
        self.total_sensor_four_states = []
        #---dict----
        self.longterm_pairing = defaultdict(list)
        self.AccuracyList = defaultdict(list)
        self.d = defaultdict(list)
        self.s = defaultdict(list)
        self.state_table = defaultdict(list)
        self.four_state_table = defaultdict(list)
        self.sensor_table = defaultdict(list)
        self.sensor_dir = {}
        self.dir_table = {}
        self.LCSS_table = {}
        self.dtw_distance = {}
        self.edit_distance = {}
        self.state_table_LCSS = defaultdict(list)
        self.id_set = set()

        self.pairing_result = {}

        self.alpha = 0.4
        self.fusion_flag = 0
        self.fusion_size = 100


        self.li = []
        self.short_long_counter = 0
        self.counter = 0
        self.longtermAccuracy = defaultdict(list)
        self.NoPairingAccuracy = defaultdict(list)

    # --- LCS ---
    def lcs(self,a, b):
        lena = len(a)
        lenb = len(b)
        c = [[0 for i in range(lenb + 1)] for j in range(lena + 1)]
        flag = [[0 for i in range(lenb + 1)] for j in range(lena + 1)]
        for i in range(lena):
            for j in range(lenb):
                if a[i] == b[j]:
                    c[i + 1][j + 1] = c[i][j] + 1
                    flag[i + 1][j + 1] = 'ok'
                elif c[i + 1][j] > c[i][j + 1]:
                    c[i + 1][j + 1] = c[i + 1][j]
                    flag[i + 1][j + 1] = 'left'
                else:
                    c[i + 1][j + 1] = c[i][j + 1]
                    flag[i + 1][j + 1] = 'up'
        return c, flag
    
    def printLcs(self,flag, a, i, j):
        if i == 0 or j == 0:
            return
        if flag[i][j] == 'ok':
            self.printLcs(flag, a, i - 1, j - 1)
            self.li.append(a[i-1])
        elif flag[i][j] == 'left':
            self.printLcs(flag, a, i, j - 1)
        else:
            self.printLcs(flag, a, i - 1, j)
    
    # --- fusion algorithm ---
    def CalculateDistanceMatrix(self, lidar_table, sensor_table, idchange):
        df = pd.DataFrame(columns = sensor_table.keys(), index = lidar_table.keys())
        df1 = pd.DataFrame(columns = sensor_table.keys(), index = lidar_table.keys())
        self.pairing_result.clear()
        num_sensor = len(sensor_table.keys())
        num_lidar = len(df)
        for sensor_key,sensor_values in sensor_table.items():
            for lidar_key,lidar_values in lidar_table.items():
                df.loc[lidar_key, sensor_key] = 360/ (360 + dtw.distance(np.array(lidar_values), np.array(sensor_values)))
                c, flag = self.lcs(lidar_values, sensor_values)
                self.printLcs(flag, lidar_values, len(lidar_values), len(sensor_values))
                df1.loc[lidar_key, sensor_key] = len(self.li)
                print('lidar:{}, sensor:{},distance:{}'.format(lidar_key, sensor_key, self.li))
                self.li.clear()
        print(df.var())
        print(df)
        x = num_sensor
        flag = 0
        tmp = 'x'
        for sensor_key,sensor_values in sensor_table.items():
            lidar_id = df1[sensor_key].astype('float64').idxmax()
            print("no pairing sensor:{}, choose id:{}".format(sensor_key, lidar_id))
            if(tmp==lidar_id):
                flag = 1
            tmp = lidar_id
            self.NoPairingAccuracy[sensor_key].append(lidar_id)
        if(flag==1):
            self.counter = self.counter+1
        for i in range(num_lidar):
            if(x==0):
                break
            else:
                x = x-1
            if(len(df)==1):
                choose_sensor_name = df.columns[0]
                choosing_lidar_id = (df.index[0])
                self.AccuracyList[choose_sensor_name].append(choosing_lidar_id)
            else:
                # pairing algorithm
                choose_sensor_name = df.var().idxmax() # sensor which has the biggest var select first
                choosing_lidar_id = df[choose_sensor_name].astype('float64').idxmax() # select the lidar which has the highest similarity
                self.AccuracyList[choose_sensor_name].append(choosing_lidar_id)    
            print('sensor:{} choose lidar {}'.format(choose_sensor_name, choosing_lidar_id))
            # --- self.pairing_result = {'Lidar_id': 'sensor name'} like {'10': 'Alex', '6': 'Chiu'} ---
            self.pairing_result[df[choose_sensor_name].astype('float64').idxmax()] = choose_sensor_name

            # --- self.longterm_pairing = 'sensor':['Lidar_id', similarity score] like {'Alex': ['10', 0.96], 'Chiu': ['6', 0.85]} ---
            if(len(self.longterm_pairing[choose_sensor_name])==0):
                self.longterm_pairing[choose_sensor_name].append(df[choose_sensor_name].astype('float64').idxmax())
                self.longterm_pairing[choose_sensor_name].append(df[choose_sensor_name].astype('float64').max())
            else:
                
                if idchange: #19/12/23
                    if self.longterm_pairing[choose_sensor_name][0] in self.remove_id:
                        self.longterm_pairing[choose_sensor_name][1] = 0
                

                if(self.longterm_pairing[choose_sensor_name][0] == df[choose_sensor_name].astype('float64').idxmax()):
                    self.longterm_pairing[choose_sensor_name][1] = self.alpha*self.longterm_pairing[choose_sensor_name][1] + (1 - self.alpha) * df[choose_sensor_name].astype('float64').max()
                   
                else:
                    if(((1 - self.alpha) * self.longterm_pairing[choose_sensor_name][1]) >= (df[choose_sensor_name].astype('float64').max())):
                        self.longterm_pairing[choose_sensor_name][1] = (1 - self.alpha) * self.longterm_pairing[choose_sensor_name][1]
                        self.short_long_counter = self.short_long_counter + 1
                    else:
                        self.longterm_pairing[choose_sensor_name][1] = df[choose_sensor_name].astype('float64').max()
                        self.longterm_pairing[choose_sensor_name][0] = df[choose_sensor_name].astype('float64').idxmax()
            # --- delete selected combination ---
            df = df.drop(index=[choosing_lidar_id])
            df = df.drop([choose_sensor_name], axis=1)
        #self.remove_id.clear()
        print(self.NoPairingAccuracy)
        print("no pairing counter",self.counter)    
        print(self.pairing_result)
        print(self.longterm_pairing)
        print("difference:",self.short_long_counter)
        for sensor_name, choose_id in self.NoPairingAccuracy.items():
            print("no pairing sensor:{}, {}/{}={}%".format(sensor_name, max(np.bincount(np.array(choose_id).astype(np.int32))), len(choose_id), max(np.bincount(np.array(choose_id).astype(np.int32)))/len(choose_id)*100))
        for sensor_name, choose_id in self.AccuracyList.items():
            print("short_term sensor:{}, {}/{}={}%".format(sensor_name, max(np.bincount(np.array(choose_id).astype(np.int32))), len(choose_id), max(np.bincount(np.array(choose_id).astype(np.int32)))/len(choose_id)*100))
            # rospy.loginfo("sensor:{}, {}".format(sensor_name, choose_id))
        for keys, values in self.longterm_pairing.items():
            self.longtermAccuracy[keys].append(values[0])
        for sensor_name, choose_id in self.longtermAccuracy.items():
            print("long_term sensor:{}, {}/{}={}%".format(sensor_name, max(np.bincount(np.array(choose_id).astype(np.int32))), len(choose_id), max(np.bincount(np.array(choose_id).astype(np.int32)))/len(choose_id)*100))
            # rospy.loginfo("sensor:{}, {}".format(sensor_name, choose_id))    
        print("-----------------------------------------------------")
        del df

    def human_callback(self,data):
        self.Circle_vec = data.circles
        self.Segment_vec = data.segments
        idmem = []
        idchange = False
        for circle in self.Circle_vec:
            id = circle.id
            idmem.append(id)
            #self.id_set.add(id)
            if not id in self.ids:
                #idchange = True
                self.ids.append(id)
            # --- self.d = 'lidar_id':[state, ....] like {'10':[1,1,1,0,1]}---
            self.d[str(id)].append(circle.state)
            # --- self.dir_table = 'id': moving direction like {'6': 360.0, '10': 135.0} ---
            self.dir_table[str(id)]  = circle.direction
            self.LCSS_table[str(id)] = circle.LCSS_state
          
            # --- tag the sensor name on lidar ----
            for keys, values in self.longterm_pairing.items():
                if(circle.id == values[0]):
                    circle.id =  keys +"("+str(round(circle.center.x,1))+","+str(round(circle.center.y,1))+")"

        # --- once obstacle id change remove old id ---
        if True:
            for idt in self.ids:
                if not idt in idmem:
                    self.remove_id.append(idt)
                    self.ids.remove(idt)
                    del self.d[str(idt)]
                    del self.dir_table[str(idt)]
                    del self.LCSS_table[str(idt)]
                    del self.state_table[str(idt)]
                    idchange = True
        #print("IDS:{},idmem:{},state_table{}" .format(self.ids,idmem,self.state_table.keys()))
        if(self.fusion_flag == self.fusion_size-1):
            for human_id in self.ids:
                state_count = Counter(self.d[str(human_id)])
                if(len(state_count)):
                    l = state_count.most_common(1)[0][0]
                else:
                    continue
                # --- self.state_table = 'LiDAR_id':[state list] like []
                if(l==0):
                    self.state_table[str(human_id)].append(0)
                    self.four_state_table[str(human_id)].append(0)
                    self.state_table_LCSS[str(human_id)].append(0)
                else:
                    self.state_table[str(human_id)].append(self.dir_table[str(human_id)])
                    self.state_table_LCSS[str(human_id)].append(self.LCSS_table[str(human_id)])
                    print("----------------", self.state_table)
                print("Lidar id:{}, total list:{}".format(human_id, self.state_table[str(human_id)]))
                distance = dtw.distance(np.array(self.total_sensor_states), np.array(self.state_table[str(human_id)]))
                edit_distance = editdistance.eval(self.total_sensor_states_LCSS, self.state_table_LCSS[str(human_id)])
                self.dtw_distance[str(human_id)] = distance
                self.edit_distance[str(human_id)] = edit_distance
                del self.d[str(human_id)]

            self.CalculateDistanceMatrix(self.state_table, self.sensor_table, idchange)
        self.fusion_flag = (self.fusion_flag+1) % self.fusion_size
        
    def sensor_callback(self,data):
        id = data.user_id
        self.State_Sensor[self.State_Sflag] = data.state
        self.State_Sflag = (self.State_Sflag + 1) % self.StateSize
        self.s[id].append(data.state)
        self.sensor_dir[id] = data.direction
        
        if(self.State_Sflag==10):
            for key,values in self.s.items():
                state = Counter(values)
                if(len(state)):
                    s = state.most_common(1)[0][0]
                else:
                    continue
                if(s==0):
                    self.sensor_table[key].append(0)
                else:
                    self.sensor_table[key].append(self.sensor_dir[key])
            state_count = Counter(self.s[id])
            if(len(state_count)):
                l = state_count.most_common(1)[0][0]
            else:
                print("state counter is empty")
            
            s = np.argmax(np.bincount(self.State_Sensor.astype(np.int32)))
            if (l==0):
                self.total_sensor_states.append(0)
                self.total_sensor_states_LCSS.append(0)
                self.total_sensor_four_states.append(0)
                self.sensor_state = 0 #for each lidar 
            else:
                self.total_sensor_states.append(data.direction)
                self.total_sensor_states_LCSS.append(data.LCSS_state)
                self.sensor_state = data.direction
            print(self.sensor_table)
            
            for key in list(self.s.keys()):
                print(key)
                del self.s[key]

        
    def sensor1_callback(self,data):
        id = data.user_id
        self.s[id].append(data.state)
        self.sensor_dir[id] = data.direction

    def sensor2_callback(self,data):
        id = data.user_id
        self.s[id].append(data.state)
        self.sensor_dir[id] = data.direction

    def sensor3_callback(self,data):
        id = data.user_id
        self.s[id].append(data.state)
        self.sensor_dir[id] = data.direction
        
    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            obstacles_ = Obstacles()
            obstacles_.circles.clear()
            obstacles_.segments.clear()

            header_ = Header(stamp=rospy.Time.now())
            header_.frame_id = "laser"
            obstacles_.header = header_
            
            for i in self.Circle_vec:
                obstacles_.circles.append(i)
            for i in self.Circle_vec_2:
                obstacles_.circles.append(i)
            for i in self.Circle_vec_3:
                obstacles_.circles.append(i)
            for i in self.Circle_vec_4:
                obstacles_.circles.append(i)
            
            self.pub_pairing.publish(obstacles_)
            r.sleep()
   
if __name__ =='__main__':

    f = fusioner()
    f.run()
