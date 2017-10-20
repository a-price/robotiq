#!/usr/bin/env python

from __future__ import division
import rospy
from sensor_msgs.msg import JointState
from robotiq_c_model_control.msg import CModel_robot_input

def state_callback(state):
    pos = state.gPO / 255.0 * 0.8575
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.header.frame_id = 'robotiq_85_adapter_link'
    js.name.append('robotiq_85_left_knuckle_joint')
    js.position.append(pos)

    pub.publish(js)

if __name__ == '__main__':
    rospy.init_node('robotiq_joint_monitor')

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    sub = rospy.Subscriber('CModelRobotInput', CModel_robot_input, state_callback)
    rospy.spin()
