#!/usr/bin/env python3
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.ros import replay_trajectory
from horizon.utils import utils
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline

import std_msgs.msg
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from datetime import datetime

import rospkg
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from std_msgs.msg import Float64
import casadi as cs
import numpy as np
import rospy
import subprocess
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
# from std_msgs.msg import Float64
import std_msgs
from std_srvs.srv import Empty, EmptyResponse
from cartesian_interface.pyci_all import *
from std_msgs.msg import Int32MultiArray, Float64MultiArray
import pkgutil
import scipy.io
import sys

def openDagana(publisher):
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(0.9, 0.1, 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        print("posTrajectory[", posPointNum,"] = ", posTrajectory[posPointNum])
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()



def closeDagana(publisher):
    daganaRefRate = rospy.Rate(100.0)
    posTrajectory = np.linspace(0.1, 0.9, 100).tolist()
    for posPointNum in range(len(posTrajectory)):
        print( "posTrajectory[posPointNum] = ", posTrajectory[posPointNum] )
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)

        daganaRefRate.sleep()
rospy.init_node('horizon_wbc_node')

pub_dagana = rospy.Publisher('/xbotcore/gripper/dagana_2/command', JointState, queue_size=1)

# # print("sys.argv.count = ", sys.argv.count)
# # exit()
# if len(sys.argv) > 1:
#     switch = sys.argv[1]

# print("switch: ", switch)
# if switch == 1:
#     daganaRefRate = rospy.Rate(100.0)
#     posTrajectory = np.linspace(1, 0.2, 100).tolist()
#     for posPointNum in range(len(posTrajectory)):
#         print("posPointNum = ", posPointNum)
#         daganaMsg = JointState()
#         daganaMsg.position.append(posTrajectory[posPointNum])
#         publisher.publish(daganaMsg)
#         daganaRefRate.sleep()
#     print("Gripper should be open! Continuing..")
# elif switch == 0:
#     closeDagana(pub_dagana)
# rospy.spin() 


switch = 0
# openDagana(pub_dagana)
if len(sys.argv) > 1:
    switch = int(sys.argv[1])
print("switch: ", switch)
if switch == 1:
    openDagana(pub_dagana)
elif switch == 0:
    closeDagana(pub_dagana)
