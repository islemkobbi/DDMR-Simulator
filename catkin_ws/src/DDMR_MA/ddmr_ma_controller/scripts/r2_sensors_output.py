#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from ddmr_ma_controller.msg import Yout, State, APpose, obs_act

import random
import numpy as np

import TD3MA

class sensors_output:
    def __init__(self):
        rospy.init_node('r2_sensors_output')

        self.model = TD3MA.load_model()

        self.obs = np.zeros([1,1,6])

        self.X = [0, 0, 0]

        self.AP1 = [-7.5,0]
        self.AP2 = [0,-7.5]
        self.AP3 = [5,7]
        self.AP4 = [8.5,0]

        self.XY1r = [0,0,0]
        self.XY2r = [0,0,0]
        self.XY3r = [0,0,0]

        self.XY1p = [0,0,0]
        self.XY2p = [0,0,0]
        self.XY3p = [0,0,0]

        self.rd = np.zeros([6])

        self.sens_out = rospy.Publisher("/robot2/sens_out", Yout, queue_size=10)
        self.ap_pose_out = rospy.Publisher("/robot2/ap_pose", APpose, queue_size=10)
        self.pose_w = rospy.Publisher("/robot2/pose_w", obs_act, queue_size=10)


        rospy.Subscriber('/robot1/odom', Odometry, self.r1_odom_callback)
        rospy.Subscriber('/robot1/estimated_state', State, self.r1_est_state_callback)

        rospy.Subscriber('/robot2/odom', Odometry, self.r2_odom_callback)
        rospy.Subscriber('/robot2/estimated_state', State, self.r2_est_state_callback)

        rospy.Subscriber('/robot3/odom', Odometry, self.r3_odom_callback)
        rospy.Subscriber('/robot3/estimated_state', State, self.r3_est_state_callback)

    def r1_odom_callback(self, odom: Odometry):
        self.XY1r = [odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_yaw(odom)]
        
        
    def r2_odom_callback(self, odom: Odometry):
        self.XY2r = [odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_yaw(odom)]

    def r3_odom_callback(self, odom: Odometry):
        self.XY3r = [odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_yaw(odom)]

    def get_yaw(self, odom : Odometry):


        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w

        # Normalize the quaternion
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        
        # Convert quaternion to Euler angles
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
        return yaw

    def r1_est_state_callback(self, S: State):
        self.XY1p = [S.X[0], S.X[1]]

    def r2_est_state_callback(self, S: State):

        self.XY2p = [S.X[0], S.X[1]]
    
    def r3_est_state_callback(self, S: State):
        self.XY3p = [S.X[0], S.X[1]]

    def get_errors(self, X, Y):
        if X > 0 and Y > 0.5 * X + 2.5 :
            ed1 = (random.random()-0.5)*1
        else:
            ed1 = (random.random()-0.5)*0.2

        
        ed2 = (random.random()-0.5)*0.2

        if X < -5 and Y > -0.5 * X + 1 :
            ed3 = (random.random()-0.5)*1
        else:
            ed3 = (random.random()-0.5)*0.2

        if X < -5 and Y > 0.5 * X + 5 :
            ed4 = (random.random()-0.5)*5
        elif X < 0 and Y > X + 2:
            ed4 = (random.random()-0.5)*1
        else:
            ed4 = (random.random()-0.5)*0.2



        return [ed1, ed2, ed3, ed4]

    def cal_real_des(self):
        X = self.XY2r


        AP5 = [self.XY1r[0], self.XY1r[1]]
        AP6 = [self.XY3r[0], self.XY3r[1]]

        ed2 = self.get_errors(X[0], X[1])

        D1 = (self.AP1[0] - X[0])**2 + (self.AP1[1] - X[1])**2 + ed2[0]
        D2 = (self.AP2[0] - X[0])**2 + (self.AP2[1] - X[1])**2 + ed2[1]
        D3 = (self.AP3[0] - X[0])**2 + (self.AP3[1] - X[1])**2 + ed2[2]
        D4 = (self.AP4[0] - X[0])**2 + (self.AP4[1] - X[1])**2 + ed2[3]
        D5 = (AP5[0] - X[0])**2 + (AP5[1] - X[1])**2
        D6 = (AP6[0] - X[0])**2 + (AP6[1] - X[1])**2 + random.random()*10
        #rospy.loginfo(self.XY3r)
        return [D1 ,D2, D3, D4, D5, D6]
    
    def cal_est_des(self):
        X = self.XY2p

        AP5 = [self.XY1p[0], self.XY1p[1]]
        AP6 = [self.XY3p[0], self.XY3p[1]]

        D1 = (self.AP1[0] - X[0])**2 + (self.AP1[1] - X[1])**2
        D2 = (self.AP2[0] - X[0])**2 + (self.AP2[1] - X[1])**2
        D3 = (self.AP3[0] - X[0])**2 + (self.AP3[1] - X[1])**2
        D4 = (self.AP4[0] - X[0])**2 + (self.AP4[1] - X[1])**2
        D5 = (AP5[0] - X[0])**2 + (AP5[1] - X[1])**2
        D6 = (AP6[0] - X[0])**2 + (AP6[1] - X[1])**2
        
        #rospy.loginfo([D1 ,D2, D3, D4, D5, D6])
        return [D1 ,D2, D3, D4, D5, D6]

    def getobs(self):

        self.rd = self.cal_real_des()
        rd = self.rd
        pd = self.cal_est_des()

        ed = np.array([rd[0]-pd[0], rd[1]-pd[1], rd[2]-pd[2], rd[3]-pd[3], rd[4]-pd[4], rd[5]-pd[5]])
        ed = np.reshape(ed,[1,1,6])

        self.obs = np.append(self.obs,ed,axis=0)
        self.obs = self.obs[-10:,:,:]
        #rospy.loginfo(self.obs.shape)

    def pub_out(self, x):
        
        sel = self.model.predict(self.obs)
        sel= np.reshape(sel[-1,:,:],[6])

        l = np.zeros([4])
        for j in range(4):
            i = np.argmax(sel)
            sel[i] = 0
            l[j] = i
        
        l = np.sort(l)


        if x == 1 :
            l= np.array([0,1,2,3])

        msg3 = obs_act()
        msg3.y = np.append(l,np.zeros(6),axis=0)
        self.pose_w.publish(msg3)

        C = np.array([self.AP1[:2], self.AP2[:2], self.AP3[:2], self.AP4[:2], self.XY1p[:2], self.XY3p[:2]],dtype=float)
        rospy.loginfo(sel)
        msg1 = APpose()
        msg1.AP1 = C[int(l[0]),:].T.tolist()
        msg1.AP2 = C[int(l[1]),:].T.tolist()
        msg1.AP3 = C[int(l[2]),:].T.tolist()
        msg1.AP4 = C[int(l[3]),:].T.tolist()
        self.ap_pose_out.publish(msg1)




        Y = self.rd
        msg2 = Yout()
        msg2.D1 = Y[int(l[0])]
        msg2.D2 = Y[int(l[1])]
        msg2.D3 = Y[int(l[2])]
        msg2.D4 = Y[int(l[3])]
        msg2.theta = self.XY2r[2]
        self.sens_out.publish(msg2)

if __name__ == '__main__':

    sensors = sensors_output()

    rate = rospy.Rate(10)

    sensors.getobs()
    sensors.pub_out(1)
    rospy.sleep(2)

    while not rospy.is_shutdown():

        sensors.getobs()
        sensors.pub_out(0)

        rate.sleep()