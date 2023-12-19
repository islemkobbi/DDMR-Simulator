#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from ddmr_ma_controller.msg import State


class print_err:
    def __init__(self):
        rospy.init_node('print_err')

        self.X = [0, 0, 0]
        self.Xp = self.X

        self.err = rospy.Publisher("/robot1/est_err", State, queue_size=10)

        rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/robot1/estimated_state', State, self.est_callback)

    def odom_callback(self, odom: Odometry):
        self.X = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                  odom.pose.pose.orientation.z]
        
    def est_callback(self, S: State):
        self.Xp = S.X[0:3]

    def pub_out(self):
        
        err = [self.X[0] - self.Xp[0], self.X[1] - self.Xp[1], self.X[2] - self.Xp[2]]

        msg = State()
        msg.X = [err[0], err[1], err[2], 0, 0]

        self.err.publish(msg)

        rospy.loginfo(err)
    

if __name__ == '__main__':

    err = print_err()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        err.pub_out()

        rate.sleep()
