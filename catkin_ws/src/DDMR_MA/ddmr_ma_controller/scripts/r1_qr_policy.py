#!/usr/bin/env python3

import rospy
import numpy as np

from ddmr_ma_controller.msg import obs_act

import TD3

class qr_policy:
    def __init__(self):
        rospy.init_node('r1_qr_policy')

        self.model = TD3.load_model()

        self.obs = np.zeros([1,1,10])

        self.act = rospy.Publisher("/robot1/qr_act", obs_act, queue_size=10)

        rospy.Subscriber('/robot1/obs', obs_act, self.getobs_callback)

    def getobs_callback(self, obs: obs_act):

        o = np.array(obs.y)
        o = np.reshape(o,[1,1,10])
        self.obs = np.append(self.obs,o,axis=0)
        self.obs = self.obs[-10:,:,:]
        rospy.loginfo(self.obs.shape)


    def get_act(self):
        #rospy.loginfo(self.obs)

        qr = self.model.predict(self.obs)
        qr_out = np.power(10,-qr[-1,:,:]*10)

        #qr_out = np.ones(10)*0.01
        
        msg = obs_act()
        msg.y = np.reshape(qr_out,[10]).tolist()
        rospy.loginfo(msg.y)
        self.act.publish(msg)
        
    

if __name__ == '__main__':

    QR_P = qr_policy()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        QR_P.get_act()
        rate.sleep()