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

rospy.init_node('horizon_wbc_node')
# load urdf and srdf to create model
urdf = rospy.get_param('robot_description', default='')
if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param('robot_description_semantic', default='')
if srdf == '':
    raise print('urdf semantic not set')

index_ = 0
ns = 0
# with open('/home/wang/horizon_wbc/output_1.txt', 'r') as file:
#     lines = file.readlines()
filename = rospkg.RosPack().get_path('centauro_long_task') + "/trajectory/pick.txt"
with open(filename, 'r') as file:
    lines = file.readlines()
matrix = []

global value 
for line in lines:
    if index_ % 2 == 0: 
        value = [float(x) for x in line.strip().split()]
        matrix.append(value)
        ns = ns + 1
    index_ = index_ + 1

# for i in range(15):
#     value[0] -= i * 0.002
#     matrix.append(value)
#     ns = ns + 1



ns = ns - 1
T = 5.
dt = T / ns
prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)
print("ns = ", ns) # 120
print("dt = ", dt) # 0.04

# try construct RobotInterface
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

# Xbot
robot = xbot.RobotInterface(cfg)
model_fk = robot.model()


ctrl_mode_override = {
    'j_wheel_1': xbot.ControlMode.Velocity(),
    'j_wheel_2': xbot.ControlMode.Velocity(),
    'j_wheel_3': xbot.ControlMode.Velocity(),
    'j_wheel_4': xbot.ControlMode.Velocity(),
    'ankle_yaw_1': xbot.ControlMode.Velocity(),
    'ankle_yaw_2': xbot.ControlMode.Velocity(),
    'ankle_yaw_3': xbot.ControlMode.Velocity(),
    'ankle_yaw_4': xbot.ControlMode.Velocity()
}
robot.setControlMode(ctrl_mode_override)
robot.sense()
q_init = robot.getJointPositionMap()
# robot.setControlMode('position')

# server
opendoor_flag = False
def opendoor(req):
    global opendoor_flag
    opendoor_flag = not opendoor_flag
    return EmptyResponse()
service = rospy.Service('opendoor', Empty, opendoor)

# q_init = {
#     "ankle_pitch_1": -0.301666,
#     "ankle_pitch_2": 0.301666,
#     "ankle_pitch_3": 0.301667,
#     "ankle_pitch_4": -0.30166,
#     "ankle_yaw_1": 0.7070,
#     "ankle_yaw_2": -0.7070,
#     "ankle_yaw_3": -0.7070,
#     "ankle_yaw_4": 0.7070,
#     "d435_head_joint": 0,
#     "hip_pitch_1": -1.25409,
#     "hip_pitch_2": 1.25409,
#     "hip_pitch_3": 1.25409,
#     "hip_pitch_4": -1.25409,
#     "hip_yaw_1": -0.746874,
#     "hip_yaw_2": 0.746874,
#     "hip_yaw_3": 0.746874,
#     "hip_yaw_4": -0.746874,
#     "j_arm1_1": 0.520149,
#     "j_arm1_2": 0.320865,
#     "j_arm1_3": 0.274669,
#     "j_arm1_4": -2.23604,
#     "j_arm1_5": 0.0500815,
#     "j_arm1_6": -0.781461,
#     "j_arm2_1": 0.520149,
#     "j_arm2_2": -0.320865,
#     "j_arm2_3": -0.274669,
#     "j_arm2_4": -2.23604,
#     "j_arm2_5": -0.0500815,
#     "j_arm2_6": -0.781461,
#     "knee_pitch_1": -1.55576,
#     "knee_pitch_2": 1.55576,
#     "knee_pitch_3": 1.55576,
#     "knee_pitch_4": -1.55576,
#     "torso_yaw": 3.56617e-13,
#     "velodyne_joint": 0,
#     "dagana_2_claw_joint": 0.
# }

base_init = np.array([0, 0, 0.8, 0, 0, 0, 1])

urdf = urdf.replace('continuous', 'revolute')

fixed_joints_map = dict()
# fixed_joints_map.update({'j_wheel_1': 0., 'j_wheel_2': 0., 'j_wheel_3': 0., 'j_wheel_4': 0.})

kin_dyn  = casadi_kin_dyn.CasadiKinDyn(urdf)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init)


# print(model.getState())
# model.fk("arm1_8")
# model.fk("j_wheel_1")
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('centauro_long_task') + '/config/centauro_wbc_config.yaml')


pm = pymanager.PhaseManager(ns+1) # ns = 69
print("ns = " ,ns)
c_timelines = dict() # [contact_x] -> timeline
for c in model.getContactMap():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')
    print("c = ", c)
stance_duration = 10
for c in model.getContactMap():
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    if ti.getTask(f'{c}') is not None:
        stance_phase.addItem(ti.getTask(f'{c}'))
    else:
        raise Exception(f'Task {c} not found')
    

for c in model.getContactMap():
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    print("c = ", c)
    

    while c_timelines[c].getEmptyNodes() > 0:
        print("c_timelines[c].getEmptyNodes() = ", c_timelines[c].getEmptyNodes())
        c_timelines[c].addPhase(stance)


pm_timelines =  pm.getTimelines()


