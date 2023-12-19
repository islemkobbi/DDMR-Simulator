#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from ddmr_ma_controller.msg import Yout, State, obs_act, APpose


class ekf:
    def __init__(self):
        rospy.init_node('r1_ekf')

        self.X = np.array([-7.5, 3, -1.57, 0, 0])

        self.AP1 = [-7.5,0]
        self.AP2 = [0,-7.5]
        self.AP3 = [5,7]
        self.AP4 = [8.5,0]

        self.t = 1000
        self.t_prev = 0
        self.t2_prev = 0

        self.V = 0
        self.W = 0

        self.dV = 0
        self.dW = 0

        self.Xpp = self.X

        self.Y = np.array([0, 0, 0, 0, 0])

        self.P = np.eye(5)

        self.qr = np.ones([10])*0.01

        self.estimated_state = rospy.Publisher("/robot1/estimated_state", State, queue_size=10)
        self.obs = rospy.Publisher("/robot1/obs", obs_act, queue_size=10)

        rospy.Subscriber('/robot1/sens_out', Yout, self.Y_callback)
        rospy.Subscriber('/robot1/cmd_vel', Twist, self.acc_callback)
        rospy.Subscriber('/clock', Clock, self.gettime_callback)
        rospy.Subscriber('/robot1/qr_act', obs_act, self.get_qr_callback)
        rospy.Subscriber('/robot1/ap_pose', APpose, self.get_APs_callback)

    def get_APs_callback(self, Y: APpose): 
        self.AP1 = Y.AP1
        self.AP2 = Y.AP2
        self.AP3 = Y.AP3
        self.AP4 = Y.AP4

    def Y_callback(self, Y: Yout):
        self.Y = np.array([Y.theta, Y.D1, Y.D2, Y.D3, Y.D4])
        rospy.loginfo(Y)

    def get_qr_callback(self, msg: obs_act):

        self.qr = np.array(msg.y)

    def acc_callback(self, cmd: Twist) :

        V = cmd.linear.x
        W = cmd.angular.z

        if (self.V == 0 or self.W == 0):
            self.W = W
            self.V = V
            dV = 0
            dW = 0
            
        else:
            dt = self.t - self.t_prev 

            if dt == 0 : 
                dt = 10000
            
            dV = (V - self.V)/ dt
            dW = (W - self.W)/ dt
            self.W = W
            self.V = V

        self.t_prev = self.t

        self.dV = dV
        self.dW = dW        


    def gettime_callback(self, time: Clock):
        self.t = time.clock.secs + time.clock.nsecs /1000000000

    def robot_move(self):

        dV = self.dV
        dW = self.dW

        X = self.X
        dX = [X[3]*np.cos(2), X[3]*np.sin(2), X[4], dV, dW]

        dt = self.t - self.t2_prev
        
        X = self.X + np.multiply(dt,dX)
        
        self.t2_prev = self.t

        theta = X[2]
        D1 = (self.AP1[0] - X[0])**2 + (self.AP1[1] - X[1])**2
        D2 = (self.AP2[0] - X[0])**2 + (self.AP2[1] - X[1])**2
        D3 = (self.AP3[0] - X[0])**2 + (self.AP3[1] - X[1])**2
        D4 = (self.AP4[0] - X[0])**2 + (self.AP4[1] - X[1])**2
        
        Y = np.array([theta, D1 ,D2, D3, D4])
        return X, Y, dt
    
    
    def cal_X(self):
        
        X, Yp, dt = self.robot_move()
        e_y = self.Y - Yp

        Fjac = np.array([[.0, .0,-X[3]*np.sin(X[2]), np.cos(X[2]), 0],
                         [.0, .0, X[3]*np.cos(X[2]), np.sin(X[2]), 0],
                         [.0, .0, 0, 0, 1],
                         [.0, .0, 0, 0, 0],
                         [.0, .0, 0, 0, 0]]) * dt + np.eye(5)
        
        Hjac = np.array([[.0, .0, 1, 0, 0],
                         [2*(X[0] - self.AP1[0]), 2*(X[1] - self.AP1[1]), 0, 0, 0],
                         [2*(X[0] - self.AP2[0]), 2*(X[1] - self.AP2[1]), 0, 0, 0],
                         [2*(X[0] - self.AP3[0]), 2*(X[1] - self.AP3[1]), 0, 0, 0],
                         [2*(X[0] - self.AP4[0]), 2*(X[1] - self.AP4[1]), 0, 0, 0]])
        
        P = self.P
        Q = np.diag(self.qr[0:5])
        R = np.diag(self.qr[5:])

        X_new = X
        P_new = np.matmul(np.matmul(Fjac, P), Fjac.T ) + Q

        HP = np.matmul(Hjac, P_new)
        RHPHT = R + np.matmul(HP, Hjac.T)
        
        IRHPHT = np.linalg.inv(RHPHT)
        
        PHT = np.matmul(P_new, Hjac.T)
        
        K = np.matmul(PHT, IRHPHT)
        
        Ke = np.matmul(K, e_y)
        
        self.X = X_new + Ke
        
        self.P = np.matmul((np.eye(5) - np.matmul(K, Hjac)), P_new)


        self.X[3] = self.V
        self.X[4] = self.W


        msg = State()
        msg.X = self.X
        self.estimated_state.publish(msg)


        X = self.X
        theta = X[2]
        D1 = (self.AP1[0] - X[0])**2 + (self.AP1[1] - X[1])**2
        D2 = (self.AP2[0] - X[0])**2 + (self.AP2[1] - X[1])**2
        D3 = (self.AP3[0] - X[0])**2 + (self.AP3[1] - X[1])**2
        D4 = (self.AP4[0] - X[0])**2 + (self.AP4[1] - X[1])**2

        e_y = self.Y - np.array([theta, D1 ,D2, D3, D4])

        msg2 = obs_act()
        msg2.y = np.append(e_y,self.X - self.Xpp,axis=0)
        self.Xpp = self.X
        self.obs.publish(msg2)
        rospy.loginfo(X)
        
if __name__ == '__main__':

    estimator = ekf()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        estimator.cal_X()
        #rospy.loginfo(estimator.X)
        rate.sleep()