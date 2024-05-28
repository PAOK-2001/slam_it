#!/usr/bin/env python
# TODO: if there is time at the end, benchmark against C++

import rospy
import math
import time
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from utils.common import *

DEBUG = False

class FrontierExplorer():
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('frontier_explorer', anonymous=True)
        self.rate = rospy.Rate(0.15)
        rospy.Subscriber(f"/inflated_map", OccupancyGrid, self.gridmap_callback)
        rospy.Subscriber(f"/filtered_pose", PoseWithCovarianceStamped , self.pose_callback)
        self.frontier_pub = rospy.Publisher('/frontier', PointStamped, queue_size=2)
        # Variables
        self.grid_map = None
        self.robot_position = None
    
    def gridmap_callback(self, map: OccupancyGrid):
         self.grid_map = map # (0-100) meaning probability that there is obstacle, -1 if unknown

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

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
        visited_mask = np_grid != UNKNOWN_TOKEN
        obstacle_mask = np_grid != OBSTACLE_TOKEN

        # Frontier cells are boundary cells between the explored and unexplored areas of the map. They cannot be obstacles
        frontier_mask = np.logical_and(visited_mask, obstacle_mask)
        frontier_cells = np.argwhere(frontier_mask)
        rospy.loginfo(f"Found {len(frontier_cells)} frontier cells")
        return frontier_cells, np_grid

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

        return goal_pose
    
    def debug_plot(self, np_grid, frontier_cells):
        plt.imshow(np_grid, cmap='gray_r', origin='lower')
        plt.title('Occupancy Grid Map with Frontiers')
        plt.colorbar(label='Occupancy Probability')
        plt.scatter(frontier_cells[:, 1], frontier_cells[:, 0], c='r', marker='o', label='Frontiers')
        plt.legend()
        plt.show()

    def explore(self):
        while not rospy.is_shutdown():
            if self.grid_map is not None and self.robot_position is not None:
                frontier_cells, np_grid = self.identify_frontiers()
                scores = self.evaluate_frontiers(frontier_cells, np_grid)
                goal = self.select_goal(frontier_cells, scores)
                self.frontier_pub.publish(goal)
                if(DEBUG):
                    self.debug_plot(np_grid, frontier_cells)

                # breakpoint()
            self.rate.sleep()
                
if __name__ == "__main__":
    explorer = FrontierExplorer()
    explorer.explore()