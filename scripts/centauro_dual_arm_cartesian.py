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


def openDagana(publisher):
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(1, 0.2, 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        # print("posPointNum = ", posPointNum)
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()
    print("Gripper should be open! Continuing..")



def closeDagana(publisher):
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(0.2, 0.9, 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)

        daganaRefRate.sleep()



# package_path = [path for _, path, _ in pkgutil.iter_modules(xbot_interface.__path__)]
# model_names = [name for _, name, _ in pkgutil.iter_modules(package_path)]
# print("model_names = ")
# print(model_names)





rospy.init_node('horizon_wbc_node')
# load urdf and srdf to create model
urdf = rospy.get_param('robot_description', default='')
if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param('robot_description_semantic', default='')
if srdf == '':
    raise print('urdf semantic not set')

index_ = 0
ns = 100



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

pm = pymanager.PhaseManager(ns+1)

c_timelines = dict()
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
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)


# forward kinematics
FK = (kin_dyn.fk('arm1_8'))
n_q = kin_dyn.nq()
n_q = kin_dyn.nq()
n_v = kin_dyn.nv()
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
# CONTROL variables
q_ddot = prb.createInputVariable('q_ddot', n_v)

pos = FK(q=q)['ee_pos']
# prb.createCost("tracking_ee_position", pos)

reference = prb.createParameter("desired_ee_position", 3, nodes=range(ns + 1))
prb.createResidual('arm1_trajectory', 20 * (pos - reference))

# the initial arm1 position
    # L_Arm_position:
        #    0.50576
        #    0.211833
        #    0.244283

    # R_Arm_position: 
        #   0.524846
        #  -0.224064
        #   0.274411

# the arm target position
des_arm1 = np.array([0.8753, 0.2821, 0.06667])

# the arm initial position
init_arm1 = np.array([0.50576, 0.211833, 0.244283])


diff = des_arm1 - init_arm1
segment_size = diff / (ns-1)  
segments = []
for i in range(100):
    current_point = init_arm1 + i * segment_size
    segments.append(current_point)
segments = np.array(segments)
print("segments size = ", segments.shape)
reference.assign(segments.T)


#
# reference.assign(matrix 21 x 100)

model.q.setBounds(model.q0, model.q0, nodes=0)
model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=0)
model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=ns)

q_min = kin_dyn.q_min()
q_max = kin_dyn.q_max()

# prb.createResidual('lower_limits', 30 * utils.barrier(model.q[-3] - q_min[-3]))
# prb.createResidual('upper_limits', 30 * utils.barrier1(model.q[-3] - q_max[-3]))

f0 = [0, 0, kin_dyn.mass() / 4 * 9.81]
for cname, cforces in model.getContactMap().items():
    for c in cforces:
        c.setInitialGuess(f0)

ti.finalize()
# ti.load_initial_guess()
ti.bootstrap()




solution = ti.solution
print("solution['q'].shape[1] = ", solution['q'].shape)
# print(solution['q'][12, :])
# exit()
## publish plot data


# publish solution server

publish_bool = False
def start_plot(req):
    global publish_bool
    publish_bool = not publish_bool
    return EmptyResponse()
service = rospy.Service('plot', Empty, start_plot)

# rospy.Service('service_name', ServiceType, callback_function).


time = 0
i = 0
rate = rospy.Rate(1./dt)

pub_dagana = rospy.Publisher('/xbotcore/gripper/dagana_2/command', JointState, queue_size=1)
# opendrawer_flag
# opendoor_flag
# while not opendoor_flag:
#     rate.sleep()
msg = JointState()

## publish plot data
pub_sol = rospy.Publisher('pose_topic_sol', Pose, queue_size=1)
pub_ref = rospy.Publisher('pose_topic_ref', Pose, queue_size=1)
pub_state = rospy.Publisher('centauro_state', Float64MultiArray, queue_size=1)



T_end = 3.5
# T = T_end
# Tee = model_fk.getPose('base_link')
# print('end effector pose w.r.t. world frame is:\n{}'.format(Tee))
# print(type(Tee))
# print(Tee.translation)
num_T = T // dt
print("num_T = ", num_T+1)
data = np.zeros((3, int(num_T+1)))
print("solution['a'].shape = ", solution['a'].shape)

while time <= T:
    solution['q'][44,i] = 0.0
    solution['v'][43,i] = 0.0

    if i >= solution['a'].shape[1]:
        i = solution['a'].shape[1] - 1
    ## update model
    q = model_fk.getJointPosition()
    qdot = solution['v'][:,i]
    qddot = solution['a'][:,i]
    print("solution['a'].shape = ", solution['a'].shape)
    q += dt * qdot + 0.5 * pow(dt, 2) * qddot
    qdot += dt * qddot
    model_fk.setJointPosition(q)
    model_fk.setJointVelocity(qdot)
    model_fk.setJointAcceleration(qddot)
    model_fk.update()
    ## generate data and save
    Tee = model_fk.getPose('dagana_2_tcp')
    # print('end effector pose w.r.t. world frame is:\n{}'.format(Tee))
    # print(Tee)
    # print(type(Tee.translation))
    # print(Tee.translation.shape)
    print("i = ", i)
    print("time = ", time)
    
    print("data.shape = ", data.shape)
    data[0, i] = Tee.translation[0]
    data[1, i] = Tee.translation[1]
    data[2, i] = Tee.translation[2]

    robot.setPositionReference(solution['q'][7:,i])
    robot.setVelocityReference(solution['v'][6:,i])
    robot.move() 
    i += 1
    time += dt
    rate.sleep()
