import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid

class MapCoverageCalculator:
    def __init__(self):
        rospy.init_node('map_coverage_calculator')
        self.map_sub = rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.map_callback)
        self.total_coverage = rospy.Publisher('/coverage', Float32, queue_size= 10)
        self.rel_coverage =  rospy.Publisher("/rel_coverage", Float32, queue_size= 10)
        

    def map_callback(self, map):
        grid_map = map 
        np_grid = np.array(grid_map.data).reshape(grid_map.info.height, grid_map.info.width)
        known_cells = (np_grid != -1).sum()
        unknown_cells = (np_grid == -1).sum()
        # Total coverage
        coverage_area = known_cells * (grid_map.info.resolution ** 2)
        # Relative coverage
        relative_coverage = known_cells/(unknown_cells + known_cells)
        # Publish
        self.total_coverage.publish(coverage_area)
        self.rel_coverage.publish(relative_coverage)

if __name__ == '__main__':
    MapCoverageCalculator()
    rospy.spin()
 
