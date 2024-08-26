#!/usr/bin/env python3
import rospy
from plannerbase import PathPlanner
# from nav_msgs.msg import OccupancyGrid
if __name__ == '__main__':
    try:
        rospy.init_node('path_planner', anonymous=True)
        planner = PathPlanner(xscale=50.0,yscale=50.0)
        # path_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
