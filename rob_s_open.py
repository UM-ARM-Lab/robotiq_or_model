#!/usr/bin/env python

import openravepy
import IPython
import numpy as np
import rospy
import math
import itertools
# from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
class Test:
    def __init__(self, g):
        self.gPOA = g
        self.gPOB = g
        self.gPOC = g

def Deg2Rad(angles):
    rads = []
    for angle in angles:
        rads.append(angle*math.pi/180)
    # print rads
    return rads


    
class Finger():
    def __init__(self):
        # self.phase = 1
        # self.state = [0,0,0,0,0,0]
        self.joints = [0.0, 0.0,0.0,0.0]
        self.g = 0
        self.max_angle = (70.0, 90.0, 43.0)
        self.min_3 = -55.0
        self.mode = 0

        
    def m3(self):
        return -55 + (43 + 55)/(255 - self.g)

    def update_status(self):
        check_limits()
        check_contacts()
    
    def check_limits(self, deltaG):
        for i in range(1,4):
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
        m1 = self.max_angle[0]/140.0
        m2 = self.max_angle[1]/100.0
        
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
            print "3"
            theta1 = self.max_angle[0]
            theta2 = m2*(g - 140)
            theta3 = self.min_3
        else:
            "4"
            theta1 = self.max_angle[0]
            theta2 = self.max_angle[1]
            theta3 = self.min_3
 
        self.joints = [self.mode, theta1, theta2, theta3]

        return


class Robotiq_gripper(object):
    def __init__(self, robot):
        self.gripper = robot
        self.A = Finger()    
        self.B = Finger()
        self.C = Finger()
        self.mode = 0

    def update_mode(self, new_mode):
        new_joint = 0.0
        if(new_mode >= 0):
            new_joint = new_mode*16
        else:
            new_joint = new_mode*10

        self.B.mode = -new_joint
        self.C.mode = new_joint 

    def update_joints(self, status):
        self.A.no_obj_update(status.gPOA)
        self.B.no_obj_update(status.gPOB)
        self.C.no_obj_update(status.gPOC)

    def update_model(self):
        self.gripper.SetDOFValues(list(itertools.chain(Deg2Rad(self.A.joints), 
                                                       Deg2Rad(self.B.joints), 
                                                       Deg2Rad(self.C.joints))))
    # def get_effector_pos(self):
    #     manip = self.gripper.SetActiveManipulator("FingerA")
    #     T = manip.GetEndEffectorTransform()
        



def gripper_model():
    env = openravepy.Environment()
    env.Load("env.xml")
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    # viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetBodies()[0]

    gripper = Robotiq_gripper(robot)
    while(1):
        mode = int(raw_input("set mode: "))
        # g = 0
        g = int(raw_input("set value of g: "))
        test = Test(g)
        gripper.update_mode(mode)
        gripper.update_joints(test)
        gripper.update_model()

    IPython.embed()


if __name__ == "__main__":
    gripper_model()
    