#!/usr/bin/env python
# TODO: if there is time at the end, benchmark against C++

import rospy
import math
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from utils.common import *

DEBUG = False
COV_THRESH = 90.0
FRONTIER_TIMEOUT = 15
RATE = 0.15
class FrontierExplorer():
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('frontier_explorer', anonymous=True)
        self.rate = rospy.Rate(RATE)
        rospy.Subscriber("/inflated_map", OccupancyGrid, self.gridmap_callback)
        rospy.Subscriber("/filtered_pose", PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber("/coverage", Float32, self.coverage_check)
        rospy.Subscriber("/stop", Bool, self.stop_callback)
        self.frontier_pub = rospy.Publisher('/frontier', PointStamped, queue_size=2)
        # Variables
        self.kernel = np.ones((3, 3), np.uint8)
        self.grid_map = None
        self.robot_position = None
        self.blacklist = []
        self.counter  = 0
        self.prev_frontier = np.array([])
        self.stop = False

    def gridmap_callback(self, map: OccupancyGrid):
         self.grid_map = map # (0-100) meaning probability that there is obstacle, -1 if unknown

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

    def stop_callback(self, msg):
        if msg.data == True:
            self.stop = True

    def coverage_check(self, coverage: Float32):
        if coverage.data >= COV_THRESH:
            self.stop = True
            rospy.loginfo("""
                            ############################
                                EXPLORATION COMPLETE
                                RETURNING TO INITIAL 
                                    POSITION
                            ############################
                            """)
        else:
            self.stop = False

    def calculate_obstacle_density(self, cell, grid, norm = True):
        row, col = cell[0], cell[1]
        surrounding_cells = grid[row-CELL_PERIMETER:row+CELL_PERIMETER, col-CELL_PERIMETER:col+CELL_PERIMETER]
        obstacle_mask = surrounding_cells >= OBSTACLE_TOKEN
        obstacle_cells = surrounding_cells[obstacle_mask]
        score = len(obstacle_cells)
        if norm: score = score/MAX_COUNT
        return score

    def calculate_unknown_density(self, cell, grid, norm = True):
        row, col = cell[0], cell[1]
        surrounding_cells = grid[row-CELL_PERIMETER:row+CELL_PERIMETER, col-CELL_PERIMETER:col+CELL_PERIMETER]
        unknow_mask = surrounding_cells == UNKNOWN_TOKEN
        unknown_cells = surrounding_cells[unknow_mask]
        score = len(unknown_cells)
        if norm: score = score/MAX_COUNT
        return score 

    def get_cell_distance(self, cell, norm = True):
        if self.robot_position is None:
            return 0
        
        cell_x = cell[1] * self.grid_map.info.resolution + self.grid_map.info.origin.position.x
        cell_y = cell[0] * self.grid_map.info.resolution + self.grid_map.info.origin.position.y

        robot_x = self.robot_position.pose.pose.position.x
        robot_y = self.robot_position.pose.pose.position.y

        distance = (cell_x - robot_x)**2 + (cell_y - robot_y)**2
        distance = math.sqrt(distance)
        if norm: distance = distance/MAX_DISTANCE
        return distance

    def identify_frontiers(self):
        # Cast grid_map into numpy array for efficient processing
        if self.grid_map is None:
            return [],[]
        
        np_grid = np.array(self.grid_map.data).reshape(self.grid_map.info.height, self.grid_map.info.width)
        # Mask out explored and obstacle cells sections
        # Frontier cells are boundary cells between the explored and unexplored areas of the map. They cannot be obstacles
        temp = np_grid.astype(np.float32)
        known_space = np.where(temp==0,1.0,0.0)
        unknown_space = np.where(temp==-1,1.0,0.0)
        dilated_known_space = cv2.dilate(known_space, self.kernel, iterations=1)
        borders_cv = cv2.bitwise_and(dilated_known_space, unknown_space)
        frontier_cells = np.argwhere(borders_cv==1)

        frontier_cells = frontier_cells.tolist()
        filtered_frontier_cells = np.array([cell for cell in frontier_cells if cell not in self.blacklist])
        return filtered_frontier_cells, np_grid

    def evaluate_frontiers(self, frontiers, grid):
        if self.grid_map is  None:
            return []
        
        scores = []
        for cell in frontiers:
            # Naive assement, assign more points to closer frontiers.
            distance = self.get_cell_distance(cell)
            #Favor node that are near to alot of unknown cells
            unknown_density = self.calculate_unknown_density(cell, grid)
            # Penalize if the frontier is covered by obstacles. TODO: perform rechability analysis
            obstacle_density = self.calculate_obstacle_density(cell, grid)
            # Combine the distance and obstacle density factors into a score
            score = P*unknown_density - A*obstacle_density - G*distance
            scores.append(score)
        return scores

    def select_goal(self, frontier_cells, scores):
        best_index = np.argmax(scores)
        goal_cell = frontier_cells[best_index]

        goal_pose = PointStamped()
        
        goal_pose.header.frame_id = self.grid_map.header.frame_id
        goal_pose.point.x = goal_cell[1] * self.grid_map.info.resolution + self.grid_map.info.origin.position.x
        goal_pose.point.y = goal_cell[0] * self.grid_map.info.resolution + self.grid_map.info.origin.position.y

        return goal_pose, goal_cell
    
    def debug_plot(self, np_grid, frontier_cells):
        plt.imshow(np_grid, cmap='gray_r', origin='lower')
        plt.title('Occupancy Grid Map with Frontiers')
        plt.colorbar(label='Occupancy Probability')
        plt.scatter(frontier_cells[:, 1], frontier_cells[:, 0], c='r', marker='o', label='Frontiers')
        plt.legend()
        plt.show()

    def blacklist_zone(self, cell):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1),
                     (0,2),(0,-2),(2,0),(-2,0),(2,1),(2,-1),(-2,1),(1,2), (-1,2), (1,-2), (-1,-2)] 
        
        self.blacklist.append(cell.tolist())
        for i, j in neighbors:
            neighbor = [cell[0] + i, cell[1] + j]
            self.blacklist.append(neighbor) 

    def explore(self):
        while not rospy.is_shutdown():
            if self.grid_map is not None and self.robot_position is not None:
                if self.stop:
                    goal_pose = PointStamped()
        
                    goal_pose.header.frame_id = self.grid_map.header.frame_id
                    goal_pose.point.x = 0
                    goal_pose.point.y = 0

                    self.frontier_pub.publish(goal)
                    pass 
                
                frontier_cells, np_grid = self.identify_frontiers()
                scores = self.evaluate_frontiers(frontier_cells, np_grid)
                goal, goal_cell = self.select_goal(frontier_cells, scores)


                if goal_cell.all() == self.prev_frontier.all():
                    if self.counter * (1/RATE) >  FRONTIER_TIMEOUT:
                        rospy.loginfo("Blacklisted zone")
                        self.blacklist_zone(goal_cell)
                    self.counter+=1

                else: 
                    self.counter = 0

                self.prev_frontier = goal_cell
                self.frontier_pub.publish(goal)
                if(DEBUG):
                    self.debug_plot(np_grid, frontier_cells)

            self.rate.sleep()
                
if __name__ == "__main__":
    explorer = FrontierExplorer()
    explorer.explore()