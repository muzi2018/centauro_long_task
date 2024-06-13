from cartesian_interface.pyci_all import *
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState




# gripper state info callback
def daganaStateCallback(data):
    '''
    Observes dagana's state, i.e. position and effort
    '''
    if not rospy.is_shutdown():
        global daganaPosition, daganaEffort
        daganaPosition = data.position[0]
        daganaEffort = data.effort[0]

# gripper close
def closeDagana(clPosition, publisher):
    '''
    Takes care of closing the dagana gripper by publishing reference position and effort
    :param gripperDetails: contains information regarding the reference position and effort to be achieved on the motor
    :param publisher: the ros publisher object for sending commands to the gripper
    '''
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(daganaPosition, clPosition, 1000).tolist()
    tauTrajectory = np.linspace(daganaEffort, -clPosition, 1000).tolist()
    # for posPointNum in range(len(posTrajectory)):
    #     daganaMsg = JointState()
    #     daganaMsg.position.append(posTrajectory[posPointNum])
    #     publisher.publish(daganaMsg)
    #     daganaRefRate.sleep()
    for tauPointNum in range(len(tauTrajectory)):
        daganaMsg = JointState()
        daganaMsg.effort.append(tauTrajectory[tauPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()

    print("Gripper closed.")

# gripper open
def openDagana(opPosition, publisher):
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(daganaPosition, opPosition, 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()
    print("Gripper open!")






if __name__ == "__main__":
    rospy.init_node("centauro_grasp")
    
    # gripper initialization
    daganaSub = rospy.Subscriber("/xbotcore/gripper/dagana_2/state", JointState, daganaStateCallback)
    daganaPub = rospy.Publisher("/xbotcore/gripper/dagana_2/command", JointState, queue_size=1)

    cli = pyci.CartesianInterfaceRos()

    task_name = 'arm2_8'
    larm = cli.getTask(task_name)

    cli.update()

    Tref, _, _ = larm.getPoseReference() # just return the pose ref, skip vel & acc
    print(Tref)

    waypoints = []
    waypoints_ = []

    # first waypoint
    wp = pyci.WayPoint()
    wp.frame.translation = [ 0.5248, -0.2241,  0.2744]
    wp.frame.quaternion = [ 0.2096,  0.4346,  0.8604, -0.1639]
    wp.time = 2.0
    waypoints.append(wp)

    wp = pyci.WayPoint()
    wp.frame.translation = [ 0.6845, -0.2458,  0.1121]
    wp.frame.quaternion = [ -0.514, -0.4998,  0.4498,  0.5326]
    wp.time = 4.0
    waypoints.append(wp)
    larm.setWayPoints(waypoints) # this sends the action goal
    larm.waitReachCompleted(8.0) 
    rospy.sleep(2)

    closeDagana(8, publisher=daganaPub)

    wp = pyci.WayPoint()
    wp.frame.translation = [ 0.6844, -0.2543,  0.1967]
    wp.frame.quaternion = [ 0.5966,  0.6298,  -0.244, -0.4335]
    wp.time = 4.0
    waypoints_.append(wp)
    larm.setWayPoints(waypoints_) # this sends the action goal
    larm.waitReachCompleted(6.0) 
    # wp = pyci.WayPoint()
    # wp.frame.translation = [ 0.6844, -0.2543,  0.1967]
    # wp.frame.quaternion = [ 0.5966,  0.6298,  -0.244, -0.4335]
    # wp.time = 8.0
    # waypoints.append(wp)


