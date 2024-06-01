import sys
import rospy
from std_msgs.msg import Bool

class EmergencyNode(object):
    def __init__(self):
        rospy.init_node('override_node', anonymous=False)
        self.stop_pub = rospy.Publisher("/stop", Bool, queue_size = 10)
        self.stop = False

    def refresh_state(self, msg):
        self.drone_status = msg
    
    def keyboard_checker(self):
        while not rospy.is_shutdown():
            input()
            rospy.loginfo("Toggle stop")
            self.stop = not self.stop
            self.stop_pub.publish(self.stop)
        
if __name__ == '__main__':
    print("--OVERRIDER CONSOLE--")
    print(" ")
    print("Press any key to toggle exploration")
    security_node = EmergencyNode()
    security_node.keyboard_checker()