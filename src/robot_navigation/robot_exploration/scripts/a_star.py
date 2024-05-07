import math
import heapq
import rospy 
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
        self.robot_position = None
        self.path_found = False
        self.ROWS, self.COLS = None, None

    def gridmap_callback(self, map: OccupancyGrid):
         self.grid_map = map # (0-100) meaning probability that there is obstacle, -1 if unknown
         self.ROWS = self.grid_map.info.height
         self.COLS = self.grid_map.info.width

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

    def goal_callback(self, goal):
        self.goal = goal

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
    
    def a_star_search(self):
        rospy.loginfo(f"Searching for path")
    
    def plan(self):
        while not rospy.is_shutdown():
            if(not self.path_found):
                self.a_star_search()
            else:
                pass
            self.rate.sleep()
        raise NotImplemented

if __name__ == "__main__":
    planer = PathPlanner()
    planer.plan()