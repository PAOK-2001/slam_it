import math
import heapq
import rospy 
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from utils.planners import a_star

class PathPlanner():
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('path_planner', anonymous=True)
        self.rate = rospy.Rate(100)
        rospy.Subscriber(f"/{self.slam_namespace}/grid_map", OccupancyGrid, self.gridmap_callback)
        rospy.Subscriber(f"/{self.slam_namespace}/localization_pose", PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber(f"/goal", PoseStamped, self.goal_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=2)

        self. goal = None
        self.grid_map = None
        self.np_grid = None
        self.robot_position = None
        self.path_found = False
        self.ROWS, self.COLS = None, None

    def gridmap_callback(self, map: OccupancyGrid):
         self.grid_map = map # (0-100) meaning probability that there is obstacle, -1 if unknown
         self.np_grid = np.array(self.grid_map.data).reshape(self.grid_map.info.height, self.grid_map.info.width)
         self.ROWS = self.grid_map.info.height
         self.COLS = self.grid_map.info.width

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

    def goal_callback(self, goal):
        self.goal = goal

    def get_cell_from_pose(self, pose_x, pose_y):
        grid_origin_x, grid_origin_y = (self.grid_map.info.origin.position.x, self.grid_map.info.origin.position.y)
        grid_resolution = self.grid_map.info.resolution
        grid_x = int((pose_x - grid_origin_x) / grid_resolution)
        grid_y = int((pose_y - grid_origin_y) / grid_resolution)
        return [grid_y, grid_x]
    
    def get_pose_from_cell(self, cell):
        cell_x = cell[1] * self.grid_map.info.resolution + self.grid_map.info.origin.position.x
        cell_y = cell[0] * self.grid_map.info.resolution + self.grid_map.info.origin.position.y
        return (cell_x, cell_y)

    def get_neighbors(self, cell):
        x, y = cell[0], cell[1]
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if 0 <= x + dx < len(self.map) and 0 <= y + dy < len(self.map[0]) and self.map[x + dx][y + dy] == 0:
                    neighbors.append((x + dx, y + dy))
        return neighbors
    
    def euclidean_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def plan(self):
        while not rospy.is_shutdown():
            if(not self.path_found):
                pass
            else:
                pass
            self.rate.sleep()

if __name__ == "__main__":
    planer = PathPlanner()
    planer.plan()