#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from ddmr_ma_controller.msg import State, Yout, obs_act, APpose
from rosgraph_msgs.msg import Clock
import numpy as np 
import os


class save_data:
    def __init__(self):
        rospy.init_node('save_data')

        self.X = None
        self.Xr = None
        self.Xp = None
        self.Y = None
        self.qr = None
        self.AP = None


        self.t0 = None
        self.t = None

        self.Xl = None
        self.Xrl = None
        self.Xpl = None
        self.tl = None
        self.Yl = None
        self.qrl = None
        self.APl = None

        rospy.Subscriber('/robot2/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/robot2/estimated_state', State, self.est_callback)
        rospy.Subscriber('/clock', Clock, self.gettime_callback)
        rospy.Subscriber('/robot2/ref_state', State, self.ref_callback)
        rospy.Subscriber('/robot2/sens_out', Yout, self.get_sensors_callback)
        rospy.Subscriber('/robot2/qr_act', obs_act, self.get_qr_callback)
        rospy.Subscriber('/robot2/pose_w', obs_act, self.get_ap_w_callback)


    def get_sensors_callback(self, Y: Yout):
        self.Y = np.array([[Y.theta,Y.D1,Y.D2,Y.D3,Y.D4]])

    def get_ap_w_callback(self, Y: obs_act):
        self.AP = np.array(Y.y[0:4])
        self.AP = np.reshape(self.AP,[1,4])

    def gettime_callback(self, time: Clock):
        t = time.clock.secs + time.clock.nsecs /1000000000
        if self.t0 == None :
            self.t0 = t
        
        t = t - self.t0

        t = np.array(t)
        self.t = np.reshape(t,[1,1])

    def odom_callback(self, odom: Odometry):
        X = [odom.pose.pose.position.x, odom.pose.pose.position.y, self.get_yaw(odom)]

        X = np.array(X)
        self.X = np.reshape(X,[1,3])

    def get_qr_callback(self, qr: obs_act):
        Y = np.array(qr.y)
        self.qr = np.reshape(Y,[1,10])


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
    
    def est_callback(self, S: State):
        X = S.X[0:3]

        X = np.array(X)
        self.Xp = np.reshape(X,[1,3])

    def ref_callback(self, S: State):
        X = S.X[0:3]

        X = np.array(X)
        self.Xr = np.reshape(X,[1,3])
    

if __name__ == '__main__':

    data = save_data()
    rate = rospy.Rate(100)

    path = os.path.join(os.path.abspath("/home"),"ik/catkin_ws/src/DDMR_MA/ddmr_ma_controller/scripts/r2_data/")

    while (data.tl is None or data.qrl is None or data.APl is None or data.Xl is None or data.Xpl is None or data.Xrl is None or data.Yl is None) and not rospy.is_shutdown() :
        data.tl = data.t
        data.Xl = data.X
        data.Xpl = data.Xp
        data.Yl = data.Y
        data.Xrl = data.Xr
        data.qrl = data.qr
        data.APl = data.AP
        rospy.loginfo([data.t , data.X, data.Xp])
        rate.sleep()
    
    while not rospy.is_shutdown():
        
        data.tl = np.append(data.tl,data.t,axis=0)
        data.Xl = np.append(data.Xl,data.X,axis=0)
        data.Xpl = np.append(data.Xpl,data.Xp,axis=0)
        data.Xrl = np.append(data.Xrl,data.Xr,axis=0)
        data.Yl = np.append(data.Yl,data.Y,axis=0)
        data.qrl = np.append(data.qrl,data.qr,axis=0)
        data.APl = np.append(data.APl,data.AP,axis=0)
        #rospy.loginfo('adding')
        rate.sleep()

        if (data.X[0,0]- 6)**2 + (data.X[0,1]- 3)**2 < 0.2 :
            rospy.loginfo('saving')
            np.savetxt(path + "real_pose.csv", data.Xl, delimiter=',')
            np.savetxt(path + "est_pose.csv", data.Xpl, delimiter=',')
            np.savetxt(path + "refrence.csv", data.Xrl, delimiter=',')
            np.savetxt(path + "outputs.csv", data.Yl, delimiter=',')
            np.savetxt(path + "qr.csv", data.qrl, delimiter=',')
            np.savetxt(path + "time.csv", data.tl, delimiter=',')
            np.savetxt(path + "AP_w.csv", data.APl, delimiter=',')



    