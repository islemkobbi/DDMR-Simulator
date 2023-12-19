#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ddmr_ma_controller.msg import State


class robot_controller:
    def __init__(self):
        rospy.init_node('r1_robot_controller')

        self.Xr = [0, 1, 0, 0, 0] 
        self.X = self.Xr
        self.Xp = self.Xr
        
        self.cmdvel = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/robot1/estimated_state', State, self.est_state_callback)
        rospy.Subscriber('/robot1/ref_state', State, self.get_ref_callback)


    def odom_callback(self, odom: Odometry):
        self.X = [odom.pose.pose.position.x, odom.pose.pose.position.y,self.get_yaw(odom)]

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
        
    def est_state_callback(self, msg: State):
        self.Xp = msg.X
        #rospy.loginfo(msg.X)

    def get_ref_callback(self, msg: State):
        self.Xr = msg.X

    def cal_eV(self):

        K_x = 2
        K_y = 6
        K_theta = 0.37

        X = self.Xp
        Xr = [0,0,0]

        a = abs(self.Xr[1] - self.X[1])
        b = abs(self.Xr[0] - self.X[0])

        if a > 0.2 or b > 0.2 :
            xa = a*0.2/max(a,b)
            xb = b*0.2/max(a,b)
        else:
            xa = 0.02
            xb = 0.02

        if abs(a) > xa:
            Xr[1] = X[1] + np.sign(self.Xr[1] - self.X[1])*xa
        else:
            Xr[1] = self.Xr[1]

        if abs(b) > xb:
            Xr[0] = X[0] + np.sign(self.Xr[0] - self.X[0])*xb
        else:
            Xr[0] = self.Xr[0]

        e = np.matmul(np.array([[np.cos(X[2]), np.sin(X[2]), 0],
                                [-np.sin(X[2]), np.cos(X[2]), 0],
                                [0, 0, 1]]),
                      np.array([[Xr[0] - X[0]], [Xr[1] - X[1]], [0]]))

        vr = K_x * e[0]
        wr = (K_y * e[1] + K_theta * np.sin(e[2])) * vr

        return [vr, wr]


    def cmd_vel(self):

        vr, wr = self.cal_eV()

        msg = Twist()
        msg.linear.x = vr
        msg.angular.z = wr
        self.cmdvel.publish(msg)

        rospy.loginfo('[ ' + str(vr) + ', ' + str(wr) + ']')


if __name__ == '__main__':

    controller = robot_controller()

    rate = rospy.Rate(100)

    rospy.sleep(5)

    while not rospy.is_shutdown():

        controller.cmd_vel()

        rate.sleep()
