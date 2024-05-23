import rospy
import math
from visualization_msgs.msg import MarkerArray

class ObjectFilter:
    def __init__(self):
        rospy.init_node('landmark_filter')
        self.marker_pub = rospy.Publisher("/filtered_landmarks", MarkerArray, queue_size=10)
        rospy.Subscriber("/landmarks", MarkerArray, self.marker_array_callback)
        self.min_distance = 0.5
        self.markers = MarkerArray()

    def calculate_distance(self, marker1, marker2):
        return math.sqrt((marker1.pose.position.x - marker2.pose.position.x) ** 2 +
                         (marker1.pose.position.y - marker2.pose.position.y) ** 2 +
                         (marker1.pose.position.z - marker2.pose.position.z) ** 2)
        
    def marker_array_callback(self, new_markers):
        for i in range(len(new_markers.markers)):
            add_marker = True
            for j in range(len(self.markers.markers)):
                # Check if the namespace is the same
                if new_markers.markers[i].ns == self.markers.markers[j].ns:
                    # Check the distance between markers
                    dist = self.calculate_distance(new_markers.markers[i], self.markers.markers[j])
                    if dist < self.min_distance:
                        add_marker = False 
                        break
            if add_marker:
                self.markers.markers.append(new_markers.markers[i])
        self.marker_pub.publish(self.markers)

if __name__ == '__main__':
    obj_filter = ObjectFilter()
    rospy.spin()