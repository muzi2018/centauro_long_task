#!/usr/bin/python3
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
import phase_manager.pyrosserver as pyrosserver

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

import cartesian_interface.roscpp_utils as roscpp
import cartesian_interface.pyci as pyci
import cartesian_interface.affine3
import horizon.utils.analyzer as analyzer

from base_estimation.msg import ContactWrenches
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from scipy.spatial.transform import Rotation

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from kyon_controller.msg import WBTrajectory

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import time

def imu_callback(msg: Imu):
    global base_pose
    base_pose = np.zeros(7)
    base_pose[3:] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])


rospy.init_node('horizon_wbc_node')
'''
Load urdf and srdf
'''
urdf = rospy.get_param('robot_description', default='')

if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param('robot_description_semantic', default='')
if srdf == '':
    raise print('urdf semantic not set')



'''
Initialize Horizon problem
'''
ns = 40
T = 2.
dt = T / ns
prb = Problem(ns, receding=True, casadi_type=cs.SX) 
prb.setDt(dt)

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

# Xbot
robot = xbot.RobotInterface(cfg)
exit()
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

base_pose = np.array([0.07, 0., 0.8, 0., 0., 0., 1.])



'''fixed joint 1
wheels_map: {'j_wheel_1': 0.0, 'j_wheel_2': 0.0, 'j_wheel_3': 0.0, 'j_wheel_4': 0.0}
'''
wheels = [f'j_wheel_{i + 1}' for i in range(4)]
wheels_map = dict(zip(wheels, 4 * [0.]))

'''fixed joint 2
ankle_yaws_map: {'ankle_yaw_1': 0.7853981633974483, 'ankle_yaw_2': -0.7853981633974483, 'ankle_yaw_3': -0.7853981633974483, 'ankle_yaw_4': 0.7853981633974483}
'''
ankle_yaws = [f'ankle_yaw_{i + 1}' for i in range(4)]
ankle_yaws_map = dict(zip(ankle_yaws, [np.pi/4, -np.pi/4, -np.pi/4, np.pi/4]))
print("ankle_yaws_map: {}".format(ankle_yaws_map))

'''fixed joint 3
arm_joints_map: {'j_arm1_1': 0.75, 'j_arm1_2': 0.1, 'j_arm1_3': 0.2, 'j_arm1_4': -2.2, 'j_arm1_5': 0.0, 'j_arm1_6': -1.3, 'j_arm2_1': 0.75, 'j_arm2_2': 0.1, 'j_arm2_3': -0.2, 'j_arm2_4': -2.2, 'j_arm2_5': 0.0, 'j_arm2_6': -1.3}
'''
arm_joints = [f'j_arm1_{i + 1}' for i in range(6)] + [f'j_arm2_{i + 1}' for i in range(6)]
arm_joints_map = dict(zip(arm_joints, [0.75, 0.1, 0.2, -2.2, 0., -1.3, 0.75, 0.1, -0.2, -2.2, 0.0, -1.3]))

'''fixed joint 4 and 5
'''
torso_map = {'torso_yaw': 0.}
head_map = {'d435_head_joint': 0.0, 'velodyne_joint': 0.0}

fixed_joint_map = dict()
fixed_joint_map.update(wheels_map)
fixed_joint_map.update(ankle_yaws_map)
fixed_joint_map.update(arm_joints_map)
fixed_joint_map.update(torso_map)
fixed_joint_map.update(head_map)


# replace continuous joints with revolute
urdf = urdf.replace('continuous', 'revolute')

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map
                                 )
rospy.set_param('robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('centauro_long_task') + '/config/centauro_wbc_armswing.yaml')

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)

# phase manager handling
c_timelines = dict()
for c in model.cmap.keys():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

short_stance_duration = 5
stance_duration = 15
flight_duration = 15
c_i = 0

# for c in model.getContactMap():
#     c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
#     prb.createResidual(f'{c}_ori', c_ori.T - np.array([0, 0, 1]))

for c in model.getContactMap():
    c_i += 1  # because contact task start from contact_1
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
    if ti.getTask(f'contact_{c_i}') is not None:
        stance_phase.addItem(ti.getTask(f'contact_{c_i}'))
        stance_phase_short.addItem(ti.getTask(f'contact_{c_i}'))
    else:
        raise Exception('task not found')

    # flight phase normal
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot + 0.01, 0.1, [None, 0, None]))
    if ti.getTask(f'z_contact_{c_i}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)
    else:
        raise Exception('task not found')

    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
    cost_ori = prb.createResidual(f'{c}_ori', 5. * (c_ori.T - np.array([0, 0, 1])))
    flight_phase.addCost(cost_ori)

for c in model.cmap.keys():
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
# ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e1 * utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))

# finalize taskInterface and solve bootstrap problem
ti.finalize()

rs = pyrosserver.RosServerClass(pm)
def dont_print(*args, **kwargs):
    pass
ti.solver_rti.set_iteration_callback(dont_print)

ti.bootstrap()
# exit()
ti.load_initial_guess()
solution = ti.solution

print("end ................")


rate = rospy.Rate(1 / dt)

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
    # if i == 50:
    #     closeDagana(pub_dagana)
    time += dt
    rate.sleep()














# model_fk = robot.model()


# ctrl_mode_override = {
#     'j_wheel_1': xbot.ControlMode.Velocity(),
#     'j_wheel_2': xbot.ControlMode.Velocity(),
#     'j_wheel_3': xbot.ControlMode.Velocity(),
#     'j_wheel_4': xbot.ControlMode.Velocity(),
#     'ankle_yaw_1': xbot.ControlMode.Velocity(),
#     'ankle_yaw_2': xbot.ControlMode.Velocity(),
#     'ankle_yaw_3': xbot.ControlMode.Velocity(),
#     'ankle_yaw_4': xbot.ControlMode.Velocity()
# }
# robot.setControlMode(ctrl_mode_override)

# q_init = robot.getJointPosition()
# q_init = robot.eigenToMap(q_init)

# # robot.setControlMode('position')



# # q_init = {
# #     "ankle_pitch_1": -0.301666,
# #     "ankle_pitch_2": 0.301666,
# #     "ankle_pitch_3": 0.301667,
# #     "ankle_pitch_4": -0.30166,
# #     "ankle_yaw_1": 0.7070,
# #     "ankle_yaw_2": -0.7070,
# #     "ankle_yaw_3": -0.7070,
# #     "ankle_yaw_4": 0.7070,
# #     "d435_head_joint": 0,
# #     "hip_pitch_1": -1.25409,
# #     "hip_pitch_2": 1.25409,
# #     "hip_pitch_3": 1.25409,
# #     "hip_pitch_4": -1.25409,
# #     "hip_yaw_1": -0.746874,
# #     "hip_yaw_2": 0.746874,
# #     "hip_yaw_3": 0.746874,
# #     "hip_yaw_4": -0.746874,
# #     "j_arm1_1": 0.520149,
# #     "j_arm1_2": 0.320865,
# #     "j_arm1_3": 0.274669,
# #     "j_arm1_4": -2.23604,
# #     "j_arm1_5": 0.0500815,
# #     "j_arm1_6": -0.781461,
# #     "j_arm2_1": 0.520149,
# #     "j_arm2_2": -0.320865,
# #     "j_arm2_3": -0.274669,
# #     "j_arm2_4": -2.23604,
# #     "j_arm2_5": -0.0500815,
# #     "j_arm2_6": -0.781461,
# #     "knee_pitch_1": -1.55576,
# #     "knee_pitch_2": 1.55576,
# #     "knee_pitch_3": 1.55576,
# #     "knee_pitch_4": -1.55576,
# #     "torso_yaw": 3.56617e-13,
# #     "velodyne_joint": 0,
# #     "dagana_2_claw_joint": 0.
# # }

# base_init = np.array([0, 0, 0.8, 0, 0, 0, 1])

# urdf = urdf.replace('continuous', 'revolute')

# fixed_joints_map = dict()
# # fixed_joints_map.update({'j_wheel_1': 0., 'j_wheel_2': 0., 'j_wheel_3': 0., 'j_wheel_4': 0.})

# ################################################################
# ### Setting symbol-based model kin_dynamic
# #################################################################

# kin_dyn  = casadi_kin_dyn.CasadiKinDyn(urdf)

# model = FullModelInverseDynamics(problem=prb,
#                                  kd=kin_dyn,
#                                  q_init=q_init,
#                                  base_init=base_init)



# # print(model.getState())
# # model.fk("arm1_8")
# # model.fk("j_wheel_1")
# bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
# process = subprocess.Popen(bashCommand.split(), start_new_session=True)

# ti = TaskInterface(prb=prb, model=model)
# ti.setTaskFromYaml(rospkg.RosPack().get_path('centauro_long_task') + '/config/centauro_wbc_armswing.yaml')

# # exit()


# pm = pymanager.PhaseManager(ns+1)

# c_timelines = dict()
# for c in model.getContactMap():
#     c_timelines[c] = pm.createTimeline(f'{c}_timeline')
#     print("c = ", c)
# stance_duration = 10
# for c in model.getContactMap():
#     stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
#     if ti.getTask(f'{c}') is not None:
#         stance_phase.addItem(ti.getTask(f'{c}'))
#     else:
#         raise Exception(f'Task {c} not found')
    

# for c in model.getContactMap():
#     stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
#     while c_timelines[c].getEmptyNodes() > 0:
#         c_timelines[c].addPhase(stance)

# matrix_np = np.array(matrix)
# matrix_np_ = matrix_np

# # matrix_np_ = np.zeros((matrix_np.shape[0], matrix_np.shape[1] + 1))
# # matrix_np_[:, 0:3] = matrix_np[:, 0:3]


# # for i in range(matrix_np.shape[0]):
# #     ori_vector = matrix_np[i, 3:6].flatten()
# #     r = R.from_rotvec(ori_vector)
# #     quat = r.as_quat()
# #     matrix_np_[i, 3:7] = quat
# # matrix_np_[:, 8:] = matrix_np[:, 7:]


# print("matrix_np.shape = ", matrix_np.shape)
# print("matrix_np_.shape = ", matrix_np_.shape)

# # exit()


# reference = prb.createParameter('upper_body_reference', 23, nodes=range(ns+1))
# # for i in range(21):
# #     reference[i] = matrix[i][0]
# #    x y z;4 quan; yaw_joint , 6 left arm, 6 right arm, 1 grippers + 2 headjoints = 7 + 15

# prb.createResidual('upper_body_trajectory', 5 * (cs.vertcat(model.q[:7], model.q[-16:]) - reference))
# # print(matrix_np_.shape)
# # exit()

# reference.assign(matrix_np_.T)
# print (reference.shape)

# #
# # reference.assign(matrix 21 x 100)

# model.q.setBounds(model.q0, model.q0, nodes=0)
# # model.q[0].setBounds(model.q0[0] + 1, model.q0[0] + 1, nodes=ns)
# model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=0)
# model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=ns)

# q_min = kin_dyn.q_min()
# q_max = kin_dyn.q_max()


# print(kin_dyn.joint_names())
# print(q_min)
# print(q_max)
# prb.createResidual('lower_limits', 30 * utils.barrier(model.q[-3] - q_min[-3]))
# prb.createResidual('upper_limits', 30 * utils.barrier1(model.q[-3] - q_max[-3]))

# # prb.createResidual('support_polygon', wheel1 - whheel2 = fixed_disanace)
# f0 = [0, 0, kin_dyn.mass() / 4 * 9.81]
# for cname, cforces in model.getContactMap().items():
#     for c in cforces:
#         c.setInitialGuess(f0)

# ti.finalize()
# # ti.load_initial_guess()
# ti.bootstrap()




# solution = ti.solution
# print("solution['q'].shape[1] = ", solution['q'].shape)
# # print(solution['q'][12, :])
# # exit()
# ## publish plot data


# # publish solution server

# publish_bool = False
# def start_plot(req):
#     global publish_bool
#     publish_bool = not publish_bool
#     return EmptyResponse()
# service = rospy.Service('plot', Empty, start_plot)

# # rospy.Service('service_name', ServiceType, callback_function).


# time = 0
# i = 0
# rate = rospy.Rate(1./dt)

# pub_dagana = rospy.Publisher('/xbotcore/gripper/dagana_2/command', JointState, queue_size=1)
# # opendrawer_flag
# # opendoor_flag
# # while not opendoor_flag:
# #     rate.sleep()
# msg = JointState()

# ## publish plot data
# pub_sol = rospy.Publisher('pose_topic_sol', Pose, queue_size=1)
# pub_ref = rospy.Publisher('pose_topic_ref', Pose, queue_size=1)
# pub_state = rospy.Publisher('centauro_state', Float64MultiArray, queue_size=1)



# T_end = 3.5
# # T = T_end
# # Tee = model_fk.getPose('base_link')
# # print('end effector pose w.r.t. world frame is:\n{}'.format(Tee))
# # print(type(Tee))
# # print(Tee.translation)
# num_T = T // dt
# print("num_T = ", num_T+1)
# data = np.zeros((3, int(num_T+1)))
# print("solution['a'].shape = ", solution['a'].shape)

# while time <= T:
#     solution['q'][44,i] = 0.0
#     solution['v'][43,i] = 0.0

#     if i >= solution['a'].shape[1]:
#         i = solution['a'].shape[1] - 1
#     ## update model
#     q = model_fk.getJointPosition()
#     qdot = solution['v'][:,i]
#     qddot = solution['a'][:,i]
#     print("solution['a'].shape = ", solution['a'].shape)
#     q += dt * qdot + 0.5 * pow(dt, 2) * qddot
#     qdot += dt * qddot
#     model_fk.setJointPosition(q)
#     model_fk.setJointVelocity(qdot)
#     model_fk.setJointAcceleration(qddot)
#     model_fk.update()
#     ## generate data and save
#     Tee = model_fk.getPose('dagana_2_tcp')
#     # print('end effector pose w.r.t. world frame is:\n{}'.format(Tee))
#     # print(Tee)
#     # print(type(Tee.translation))
#     # print(Tee.translation.shape)
#     print("i = ", i)
#     print("time = ", time)
    
#     print("data.shape = ", data.shape)
#     data[0, i] = Tee.translation[0]
#     data[1, i] = Tee.translation[1]
#     data[2, i] = Tee.translation[2]

#     robot.setPositionReference(solution['q'][7:,i])
#     robot.setVelocityReference(solution['v'][6:,i])
#     robot.move() 
#     i += 1
#     # if i == 50:
#     #     closeDagana(pub_dagana)
#     time += dt
#     rate.sleep()
# # now = datetime.now()
# # time_str = now.strftime("%m%d_%H%_M")
# # current_directory = os.getcwd()
# # file_name = f'{current_directory}/data/open_drawer_sim_{time_str}.mat'
# # scipy.io.savemat(file_name, {'xyz': data})
# # print(f"Data saved to {file_name}")


# # scipy.io.savemat('open_drawer_sim_2110.mat', {'xyz': data})
# # scipy.io.savemat('open_door_sim_2053.mat', {'xyz': data})




# # print("solution['q] = ", solution['q'].shape) #solution['q] =  (47, 94)

# # while not rospy.is_shutdown():
# #     for i in range(solution['q'].shape[1]):
# #         pose = Pose()
# #         pose.position.x = solution['q'][0,i] 
# #         pose.position.y = solution['q'][1,i]
# #         pose.position.z = solution['q'][2,i]
# #         pose.orientation.x = solution['q'][3,i]
# #         pose.orientation.y = solution['q'][4,i]
# #         pose.orientation.z = solution['q'][5,i]
# #         pose.orientation.w = solution['q'][6,i]
# #         pub_sol.publish(pose)
# #         pose.position.x = matrix_np_[i,0] 
# #         pose.position.y = matrix_np_[i,1]
# #         pose.position.z = matrix_np_[i,2]
# #         pose.orientation.x = matrix_np_[i,3]
# #         pose.orientation.y = matrix_np_[i,4]
# #         pose.orientation.z = matrix_np_[i,5]
# #         pose.orientation.w = matrix_np_[i,6]
# #         pub_ref.publish(pose)

# # publish solution
    


#     # repl = replay_trajectory.replay_trajectory(prb.getDt(), kin_dyn.joint_names(), solution['q'], kindyn=kin_dyn, trajectory_markers=model.getContactMap().keys())
#     # repl.replay(is_floating_base=True)
#     # rate.sleep()
