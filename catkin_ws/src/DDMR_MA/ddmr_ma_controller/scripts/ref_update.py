#!/usr/bin/env python3

import rospy

from ddmr_ma_controller.msg import State
import numpy as np

class up_ref:
    def __init__(self):
        rospy.init_node('update_ref')

        self.AP1 = [-7.5,1]
        self.AP2= [-7.5,1]
        self.AP3 = [-7.5,1]

        self.D1 = 10
        self.D2 = 10
        self.D3 = 10

        self.i1 = 0
        self.i2 = 0
        self.i3 = 0

        self.P1 = np.array([[-7,0],[-5,-1],[-2,0],[0,0],[3,4],[4.5,6],[6,6]])
        self.P2 = np.array([[-7,0],[-5,-1],[-2,0],[0,0],[3,2],[4.5,3],[6,3]])
        self.P3 = np.array([[-7,0],[-5,-1],[-2,0],[0,0],[3,0],[4.5,0],[6,0]])

        self.ref_r1 = rospy.Publisher("/robot1/ref_state", State, queue_size=10)
        self.ref_r2 = rospy.Publisher("/robot2/ref_state", State, queue_size=10)
        self.ref_r3 = rospy.Publisher("/robot3/ref_state", State, queue_size=10)

        rospy.Subscriber('/robot1/estimated_state', State, self.r1_est_state_callback)
        rospy.Subscriber('/robot2/estimated_state', State, self.r2_est_state_callback)
        rospy.Subscriber('/robot3/estimated_state', State, self.r3_est_state_callback)


    def r1_est_state_callback(self, XY: State):
        self.D1 = np.sqrt((self.AP1[0] - XY.X[0])**2 + (self.AP1[1] - XY.X[1])**2)
        
    def r2_est_state_callback(self, XY: State):
        self.D2 = np.sqrt((self.AP2[0] - XY.X[0])**2 + (self.AP2[1] - XY.X[1])**2)

    def r3_est_state_callback(self, XY: State):
        self.D3 = np.sqrt((self.AP3[0] - XY.X[0])**2 + (self.AP3[1] - XY.X[1])**2)


    def pub_out(self):
        d = 0.75
        if self.D1 < d and self.i1 < 7 :
            self.AP1 = self.P1[self.i1,:]
            self.i1 = self.i1 + 1

        msg1 = State()
        msg1.X = [self.AP1[0],self.AP1[1],0,0,0]
        self.ref_r1.publish(msg1)

        if self.D2 < d and self.i2 < 7 :
            self.AP2 = self.P2[self.i2,:]
            self.i2 = self.i2 + 1

        msg2 = State()
        msg2.X = [self.AP2[0],self.AP2[1],0,0,0]
        self.ref_r2.publish(msg2)

        if self.D3 < d and self.i3 < 7 :
            self.AP3 = self.P3[self.i3,:]
            self.i3 = self.i3 + 1

        msg3 = State()
        msg3.X = [self.AP3[0],self.AP3[1],0,0,0]
        self.ref_r3.publish(msg3)

if __name__ == '__main__':

    update = up_ref()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        rospy.loginfo([update.i1,update.i2,update.i3])
        rospy.loginfo([update.D1,update.D2,update.D3])
        update.pub_out()

        rate.sleep()
