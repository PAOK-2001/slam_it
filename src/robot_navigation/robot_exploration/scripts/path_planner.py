import math
import cv2
import rospy 
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
import matplotlib.pyplot as plt
from utils.planners import astar
from utils.common import *

DEBUG = False

class PathPlanner():
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('path_planner', anonymous=True)
        self.rate = rospy.Rate(10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=2)
        self.path = None
        self. goal = None
        self.grid_map = None
        self.np_grid = None
        self.robot_position = None
        self.path_found = False
        self.ROWS, self.COLS = None, None
        self.kernel = np.ones((3, 3), np.uint8) 
        # Callbacks
        rospy.Subscriber(f"/inflated_map", OccupancyGrid, self.gridmap_callback)
        rospy.Subscriber(f"/{self.slam_namespace}/localization_pose", PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber("/frontier", PointStamped, self.goal_callback)
        rospy.Subscriber(f"/goal", PointStamped, self.goal_callback)

    def gridmap_callback(self, map: OccupancyGrid):
         self.path_found = False
         self.grid_map = map # (0-100) meaning probability that there is obstacle, -1 if unknown
         np_grid = np.array(self.grid_map.data).reshape(self.grid_map.info.height, self.grid_map.info.width)
         self.np_grid = self.get_inflated_map(np_grid)
         self.ROWS = self.grid_map.info.height
         self.COLS = self.grid_map.info.width

    def get_inflated_map(self, map):
        temp = map.astype(np.float32)
        temp = cv2.dilate(temp, self.kernel, iterations=4)
        if DEBUG:
            cv2.imshow("MAP",cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR))
            cv2.waitKey(1)
            print("Dilated map", temp.shape)
        return temp

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

    def goal_callback(self, new_goal):
        if self.goal is not None:
            if new_goal.point != self.goal.point: self.path_found = False  # Toggle path finding
        self.goal = new_goal

    def get_cell_from_pose(self, pose_x, pose_y):
        grid_origin_x, grid_origin_y = (self.grid_map.info.origin.position.x, self.grid_map.info.origin.position.y)
        grid_resolution = self.grid_map.info.resolution
        grid_x = int((pose_x - grid_origin_x) / grid_resolution)
        grid_y = int((pose_y - grid_origin_y) / grid_resolution)
        return (grid_y, grid_x)
    
    def get_pose_from_cell(self, cell):
        cell_x = cell[1] * self.grid_map.info.resolution + self.grid_map.info.origin.position.x
        cell_y = cell[0] * self.grid_map.info.resolution + self.grid_map.info.origin.position.y
        return (cell_x, cell_y)

    def build_path(self, route):
        path = Path()
        path.header.frame_id = self.goal.header.frame_id
        for i in range(len(route) -1, -1, PATH_STEP):
            curr_pose = PoseStamped()
            curr_pose.header.frame_id = self.goal.header.frame_id 
            x, y = self.get_pose_from_cell(route[i])
            curr_pose.pose.position.x = x
            curr_pose.pose.position.y = y
            path.poses.append(curr_pose)
        return path
    
    def plan(self):
        while not rospy.is_shutdown():
            if(not self.path_found and self.robot_position is not None and self.grid_map is not None and self.goal is not None):
                path = None
                start_cell = self.get_cell_from_pose(pose_x= self.robot_position.pose.pose.position.x, 
                                                     pose_y= self.robot_position.pose.pose.position.y)
                
                goal_cell = self.get_cell_from_pose(pose_x= self.goal.point.x, 
                                                    pose_y=self.goal.point.y)
                
                
                rospy.loginfo(f"Searching for path...")
                path = astar(map= self.np_grid, start= start_cell, goal=goal_cell)
            
                if(path is not None):
                    rospy.loginfo(f"Found path")
                    self.path_found = True
                    self.path = self.build_path(path)
                    self.path_pub.publish(self.path)

            else:
                if self.path is not None:
                    self.path_pub.publish(self.path)
        
            self.rate.sleep()

if __name__ == "__main__":
    planer = PathPlanner()
    planer.plan()