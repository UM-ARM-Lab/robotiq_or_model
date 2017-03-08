#!/usr/bin/env python

import openravepy
import IPython
import numpy as np
import rospy
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg


class Robotiq_gripper(object):
    def __init__(self, robot):
        self.gripper = robot
        self.A = Finger()    
        self.B = Finger()
        self.C = Finger()

    def update_fingers(self, status):
        self.A.no_obj_update(status.gPOA)
        self.B.no_obj_update(status.gPOB)
        self.C.no_obj_update(status.gPOC)
    
class Finger():
    def __init__(self):
        self.phase = 1
        self.state = [0,0,0,0,0,0]
        self.joints = (0,0,0)
        self.g = 0
        self.max_angle = (70, 90, 43)
        self.min_3 = -55

        self.m1 = 70/140
        self.m2 = 90/100
        
    def m3(self):
        return -55 + (43 + 55)/(255 - self.g)

    def update_status(self):
        check_limits()
        check_contacts()
    
    def check_limits(self, deltaG):
        for i in range(3):
            if self.state[i+3]  == 0 and self.joints[i] >= self.max_angle[i]:
                self.state[i+3] = 1

            if self.state[i+3] == 1 and (self.joints[i] + deltaG) < self.max_angle[i]:
                self.state[i+3] = 0

        if self.state[6] == 0 and self.joints[3] <= self.min_angle[3]:
            self.state[6] = -1

        elif self.state[6] == -1 and self.joints[3] + deltaG > self.min_3:
            self.state[6] = 0

    #function describes joint angles when there is no object present in the system
    def no_obj_update(self, g):
        m1 = self.max_angle[0]/140
        m2 = self.max_angle[1]/100

        theta1 = 0
        theta2 = 0
        theta3 = 0

        if g <= 110:
            theta1 = m1*g
            theta2 = 0
            theta3 = -m1*g
        elif g <= 140:
            theta1 = m1*g
            theta2 = 0
            theta3 = self.min_3
        elif g <= 240:
            theta1 = self.max_angle[0]
            theta2 = m2*(g - 140)
            theta3 = self.min_3
        else:
            theta1 = self.max_angle[0]
            theta2 = self.max_angle[1]
            theta3 = self.min_3

        self.joints = (theta1, theta2, theta3)
        return

    # def update_joints(self, g):
    #     delta1 = 0
    #     delta2 = 0
    #     delta3 = 0

    #     deltaG = g - self.g

    #     if self.phase == 1:
    #         delta1 = self.m1*deltaG
    #         delta3 = -self.m1*deltaG
    #     elif self.phase == 2:
    #         delta2 = self.m2*deltaG
    #         delta3 = -delta2
    #     elif self.phase == 3:
    #         delta3 = self.m3() * deltaG

    #     self.joints = self.joints + (delta1, delta2, delta3)

    #     self.update_status(deltaG)
    #     self.g = g
    

def gripper_model():
    env = openravepy.Environment()
    env.Load("env.xml")
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0]

    # gripper = Robotiq_gripper(robot)
    IPython.embed()
    # rospy.init_node('GripperDisplay')
    # rospy.Subscriber("/gripper_server/input", inputMsg.SModel_robot_input, gripper.update_fingers)
    # rospy.spin()

if __name__ == "__main__":
    gripper_model()
    