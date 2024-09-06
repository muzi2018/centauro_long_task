#!/usr/bin/env python3
import rospy
# import path_planner
import heapq
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# from tf import TransformListener, ExtrapolationException, LookupException
# from tf2_msgs.msg import TFMessage
import math
import sys
import numpy as np
sys.setrecursionlimit(1000000)

class Node:
    def __init__(self, index, pos,g_score=float('inf'), f_score=float('inf')):
        self.index = index
        self.pos=pos
        self.g_score = g_score
        self.f_score = f_score
        self.parent = None
        # self.dir=np.zeros(2)
        self.id=0
        self.obs=0
    def to_pose_stamped(self,pt):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.w = 1.0

        return pose
class Grid:
    def __init__(self, x_low:float,y_low:float,x_up:float,y_up:float,index_weight:int,index_height:int,resolution:float,points_list: list):
        self.gl_xl = x_low
        self.gl_yl = y_low
        self.gl_xu = x_up
        self.gl_yu = y_up

        self.resolution = resolution
        self.GLX_SIZE = index_weight
        self.GLY_SIZE = index_height
        self.inv_resolution = 1.0 / self.resolution
        self.GridNodeMap = [[Node(None, None) for j in range(self.GLY_SIZE)] for i in range(self.GLX_SIZE)]
        for i in range(self.GLX_SIZE):
            for j in range(self.GLY_SIZE):
                tmpIdx = np.array([i,j])
                pos = self.gridIndex2coord(tmpIdx)
                self.GridNodeMap[i][j] = Node(tmpIdx, pos)
        for point in points_list:
            if point[2] > 1.8 or point[2] < 0:
                continue
            self.setObs(point[0],point[1])

    def setObs(self, coord_x, coord_y):
        if coord_x < self.gl_xl or coord_y < self.gl_yl  or coord_x >= self.gl_xu or coord_y >= self.gl_yu:
            return
        index= self.coord2gridIndex(pt=[coord_x,coord_y])
        self.GridNodeMap[index[0]][index[1]].obs = 1
    def gridIndex2coord(self, index):
        pt = np.zeros(2)
        pt[0] = (index[0] + 0.5) * self.resolution + self.gl_xl
        pt[1] = (index[1] + 0.5) * self.resolution + self.gl_yl
        return pt
    def coord2gridIndex(self, pt):
        idx = np.zeros((2,), dtype=int)
        idx[0] = min(max(int((pt[0] - self.gl_xl) * self.inv_resolution),0),self.GLX_SIZE-1)
        idx[1] = min(max(int((pt[1] - self.gl_yl) * self.inv_resolution),0),self.GLY_SIZE-1)

        return idx
    def update(self,points: list):
        for point in points:
            self.setObs(point[0],point[1])

class AStar2D(Grid):
    def __init__(self, x_weight:float, y_height:float, resolution:float, points_list: list):
        self._x_low=-x_weight/2.0
        self._y_low=-y_height/2.0
        self._x_up=x_weight/2.0
        self._y_up=y_height/2.0
        self._index_weight=int(x_weight/resolution)
        self._index_height=int(y_height/resolution)
        self._resolution=resolution
        Grid.__init__(self, x_low=self._x_low,y_low=self._y_low,x_up=self._x_up,y_up=self._y_up,index_weight=self._index_weight,index_height=self._index_height,resolution=self._resolution,points_list=points_list)
        # self.gridAll=_grid
        self.open_set = []
        self.start_node = None
        self.goal_node = None
        self.Flag_done=False
    def chenkIndexObs(self,index):
        return index[0]>=0 and index[0]<self.GLX_SIZE \
        and index[1]>=0 and index[1]<self.GLY_SIZE \
        and (self.GridNodeMap[index[0]][index[1]].obs==1)
    def chenkCoordObs(self,pt):
        index=self.coord2gridIndex(pt)
        if self.GridNodeMap[index[0]][index[1]].obs==1:
            print("in obs")
        return pt[0]>=self.gl_xl and pt[0]<self.gl_xu \
        and pt[1]>=self.gl_yl and pt[1]<self.gl_yu \
        and (self.GridNodeMap[index[0]][index[1]].obs==1)
    def to_pose_stamped(self,pt):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.w = 1.0

        return pose
    def heuristic(self, nodeone,nodetwo):
        dx = abs(nodeone.index[0] - nodetwo.index[0])
        dy = abs(nodeone.index[1] - nodetwo.index[1])
        h=(dx**2+dy**2)**0.5
        return h
    def AstarGetSucc(self, current:Node):
        neighbors = []
        edgeCostSets=[]
        this_x = current.index[0]
        this_y = current.index[1]
        this_coord=current.pos
        n_x, n_y=0,0
        dist=0.0
        temp_ptr=None
        n_coord=np.zeros(2)
        for i in range(-1,2):
            for j in range(-1,2):
                    if i == 0 and j == 0:
                        continue
                    n_x = int(this_x + i)
                    n_y = int(this_y + j)
                    if (n_x < 0) or (n_x > (self.GLX_SIZE - 1)) or (n_y < 0) or (n_y > (self.GLY_SIZE - 1) ):
                        continue
                    if self.chenkIndexObs(index=[n_x, n_y]):
                        continue
                    temp_ptr = self.GridNodeMap[n_x][n_y]
                    if temp_ptr.id == -1 :
                        continue
                    n_coord = temp_ptr.pos
                    if (abs(n_coord[0] - this_coord[0]) < 1e-6) and (abs(n_coord[1] - this_coord[1]) < 1e-6):
                        rospy.loginfo("Error: Not expanding correctly!")
                    dist = math.sqrt( (n_coord[0] - this_coord[0]) **2+\
                    (n_coord[1] - this_coord[1]) **2)
                    neighbors.append(temp_ptr)
                    edgeCostSets.append(dist)
        return neighbors,edgeCostSets
    def get_path(self):
        if self.Flag_done:
            path=[]
            gridPath=[]
            ptr = self.goal_node
            while(ptr.parent!= None):
                gridPath.append(ptr.index)
                path.append(ptr.pos)
                ptr = ptr.parent
            gridPath.append(ptr.index)
            path.append(ptr.pos)
            gridPath.reverse()
            path.reverse()
            return gridPath,path
        else:
            return [],[]
    def search(self, start, goal):
        self.Flag_done=False
        gridStart=self.coord2gridIndex(start)
        gridGoal=self.coord2gridIndex(goal)
        if self.chenkIndexObs(gridStart) or self.chenkIndexObs(gridGoal):
            rospy.loginfo("Start or Goal in obs!Please check and re-input")
        elif start[0]<self.gl_xl or start[1]<self.gl_yl \
        or start[0]>self.gl_xu or start[1]>self.gl_yu \
        or goal[0]<self.gl_xl or goal[1]<self.gl_yl \
        or goal[0]>self.gl_xu or goal[1]>self.gl_yu :
            rospy.loginfo("Start or Goal over the boundary!Set lower:"+str((self.gl_xl,self.gl_yl))+"and upper:"+str((self.gl_xu,self.gl_yu)))
        else:
            rospy.loginfo("Start searching...")
            self.goal_node = Node(index=gridGoal,pos=goal)
            self.start_node = Node(index=gridStart,pos=start, g_score=0)
            self.start_node.f_score=self.heuristic(self.start_node,self.goal_node)
            self.start_node.id=1
            self.open_set.append(self.start_node)
            self.GridNodeMap[gridStart[0]][gridStart[1]].id=1
            edgeCostSets=[]
            neighbors=[]
            while self.open_set:
                current = self.open_set[0]
                self.open_set.pop(0)
                self.GridNodeMap[current.index[0]][current.index[1]].id=-1
                if (current.index == self.goal_node.index).all():
                    rospy.loginfo("Find!")
                    self.Flag_done=True
                    self.goal_node.parent=current
                    return 
                neighbors,edgeCostSets=self.AstarGetSucc(current)
                for i in range(len(neighbors)):
                    neighbor=neighbors[i]
                    if neighbor.id==0:
                        neighbor.g_score=current.g_score+ edgeCostSets[i]
                        neighbor.f_score=neighbor.g_score+self.heuristic(neighbor,self.goal_node)
                        neighbor.parent=current
                        self.open_set.append(neighbor)
                        neighbor.id=1
                        continue
                    elif neighbor.id==1:
                        if neighbor.g_score>(current.g_score+edgeCostSets[i]):
                            neighbor.g_score = current.g_score + edgeCostSets[i]
                            neighbor.f_score = neighbor.g_score + self.heuristic(neighbor,self.goal_node)
                            neighbor.parent = current
                    else:
                        continue
            rospy.loginfo("No path!")



class PathPlanner:
    def __init__(self,xscale:float,yscale:float):
        self.start = None
        self.goal = None
        self.astar=None
        self._xweight=xscale
        self._yheight=yscale
        self.is_goal_reached = False
        self.is_map_loaded = False
        self.is_map_update = False

        self.map_subscriber = rospy.Subscriber("/map", PointCloud2, self.map_callback)
        # self.start_subscriber = rospy.Subscriber("/tf", TFMessage, self.start_callback)
        self.start_subscriber = rospy.Subscriber("/start_point", PoseStamped, self.start_callback)
        self.goal_subscriber = rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        # self.transform_listener = TransformListener()

        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        # self.mover_publisher = rospy.Publisher("/move", Path, queue_size=1)
      
    def map_callback(self, data: PointCloud2):
        points=list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z")))
        if self.astar==None:
            rospy.loginfo("initial map")
            self.astar = AStar2D(x_weight=self._xweight,y_height=self._yheight,resolution=0.5,points_list=points)
            self.is_map_loaded = True
        else:
            rospy.loginfo("update map")
            self.astar.update(points)
            self.is_map_update = True
        
    def start_callback(self, start_point:PoseStamped):
        self.start=start_point.pose
        rospy.loginfo("Received start point")
        # self.mover.robot_position = self.start

    def goal_callback(self, data: PoseStamped) -> bool:
        rospy.loginfo("Received goal")
        self.goal = data.pose
        self.is_goal_reached = False
        if self.is_map_loaded == True:
            if self.astar.chenkCoordObs(pt=[self.goal.position.x,self.goal.position.y]) or not self.is_map_loaded or not self.start:
                rospy.loginfo("Goal can't be reached")
                return False

            return self.calculate_path()

    def calculate_path(self):
        self.astar.search(start=[self.start.position.x,self.start.position.y],goal=[self.goal.position.x,self.goal.position.y])
        gridpath,path=self.astar.get_path()
        if len(path) > 0:
            # path_list = self.moving_average(path_list)
            path_msg = self.display_path(path)
            # self.mover_publisher.publish(path_msg)
            return True
        self.display_path([])

        return False
    
    def send_goal(self, pose: Pose) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = pose

        return self.goal_callback(goal)

    def cancel_goal(self):
        # self.mover.initialize_stop()
        self.is_goal_cancelled = True

    def moving_average(self, path: list, window: int = 4) -> list:
        window_queue = []
        smoothed_path = [path[0]]

        for node in path:
            if len(window_queue) == window:
                smoothed_path.append(sum(window_queue) / window)  # Mean
                window_queue.pop(0)

            window_queue.append(node)

        return smoothed_path + [self.goal]

    def display_path(self, path_nodes: list) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        print(len(path_nodes))
        for pose in path_nodes:
            path.poses.append(self.astar.to_pose_stamped(pose))
        # print(path)
        self.path_publisher.publish(path)

        return path

