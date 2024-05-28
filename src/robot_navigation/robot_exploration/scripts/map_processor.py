#!/usr/bin/env python
import rospy 
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from utils.planners import astar
from utils.common import *

DEBUG = False

class MapProcessor:
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('map_processor')
        self.kernel = np.ones((3, 3), np.uint8)
        # Callbacks
        self.map_publisher = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size= 10)
        rospy.Subscriber(f"/{self.slam_namespace}/grid_map", OccupancyGrid, self.gridmap_callback)

  
    def gridmap_callback(self, map: OccupancyGrid):
        # Unpack map and inflate obstacles
        grid_map = map 
        np_grid = np.array(grid_map.data).reshape(grid_map.info.height, grid_map.info.width)
        inflated_grid = self.get_inflated_map(np_grid)
        inflated_grid = np.maximum(np_grid,inflated_grid)
        inflated_grid = inflated_grid.flatten()
        inflated_grid = inflated_grid.tolist()
        # Publish new map
        map_msg = OccupancyGrid()
        map_msg.header = map.header
        map_msg.info = map.info
        map_msg.data = inflated_grid
        self.map_publisher.publish(map_msg)

    def get_inflated_map(self, map):
        temp = map.astype(np.float32)
        temp = np.where(temp>1,temp,-1)
        temp = cv2.dilate(temp, self.kernel, iterations=5)
        if DEBUG:
            cv2.imshow("MAP", cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR))
            cv2.waitKey(1)
            print("Dilated map", temp.shape)
        return temp.astype(int)

if __name__ == "__main__":
    processor = MapProcessor()
    rospy.spin() 