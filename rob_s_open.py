#!/usr/bin/env python

import openravepy
import IPython

import rospy
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
class Robotiq_gripper(object):
    def __init__(self):
        self.A = Finger()    
        self.B = Finger()
        self.C = Finger()

    
class Finger():
    def __init__(self):
        self.status = [(0,0,0,0,0,0)]*3
        self.joints = [(0,0,0)]*3
        self.g = [0]*3
        self.max_angle = (70, 90, 43)
        self.min_3 = -55

    def update_individual(index, new_g):
        pass

    def update_status():
        pass

    def update_joints(status):


def gripper_model():
    env = openravepy.Environment()
    env.Load("env.xml")
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0]

    gripper = Robotiq_gripper()
    rospy.init_node('GripperDisplay')
    rospy.Subscriber("/gripper_server/input", inputMsg.SModel_robot_input, gripper.update_joints)
    rospy.spin()

if __name__ == "__main__":
    gripper_model()
    