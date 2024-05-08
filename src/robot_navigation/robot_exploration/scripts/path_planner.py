import math
import rospy 
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from utils.planners import a_star

class PathPlanner():
    def __init__(self):
        self.slam_namespace = "rtabmap"
        # Init nodes and define pubs and subs
        rospy.init_node('path_planner', anonymous=True)
        self.rate = rospy.Rate(100)
        rospy.Subscriber(f"/{self.slam_namespace}/grid_map", OccupancyGrid, self.gridmap_callback)
        rospy.Subscriber(f"/{self.slam_namespace}/localization_pose", PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber(f"/goal", PointStamped, self.goal_callback)
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

    def goal_callback(self, new_goal):
        if new_goal != self.goal: self.path_found = False  # Toggle path finding
        self.goal = new_goal

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

    def build_path(path):
        raise NotImplemented
    
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