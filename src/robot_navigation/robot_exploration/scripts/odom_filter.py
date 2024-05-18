import rospy
import math
import numpy as np
import rospy.rosconsole
import time
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from utils.kalman_filter import KalmanFilter
from tf.transformations import euler_from_quaternion

KF_METHOD = 'kelly'
# Filter Node
class FilterNode:
    def __init__(self):
        # ROS params
        rospy.init_node('filter_node')
        self.rate = rospy.Rate(100)
        # Robot params
        _WHEELRADIUS = rospy.get_param("/wheel_base")
        _WHEELBASE = rospy.get_param("/wheel_radius")
        self.r = _WHEELRADIUS
        self.d = _WHEELBASE
        self.h = 0
        self. phi = np.array([[_WHEELRADIUS/_WHEELBASE, -_WHEELRADIUS/_WHEELBASE]])
        # Subscriber and publisher
        self.odom_sub =  rospy.Subscriber('odom', Odometry, self.odometry_callback)
        self.wr_sub   =  rospy.Subscriber('wr', Float32, self.right_wheel_callback)
        self.wl_sub   =  rospy.Subscriber('wl', Float32, self.left_wheel_callback)
        self.speed_sub  =  rospy.Subscriber('cmd_vel', Twist, self.speed_callback)
        self.filter_pub = rospy.Publisher('filtered_odom', Odometry, queue_size=10)
        # Filter params
        self.obs_mat = np.eye(3)
        self.state_covar = np.eye(3)
        self.process_covar = np.eye(3)*0.005
        self.measurments_covar = np.eye(3)*100
        # Messages
        self.original_odom =  None
        self.cmd_vel = None
        # State vectors
        self.pose = None
        self.wheel_speeds = np.array([0, 0])
        self.speeds = np.array([0,0])
        # Build KF 
        x_hat = np.array([[0, 0, 0]]).T  # Initial States
        # Init kalman filter
        system_settings = (self.measurments_covar, self.process_covar, self.state_covar, self.obs_mat)
        self.kf = KalmanFilter(system_settings, x_hat)

    def odometry_callback(self, msg):
        # Unpack message
        self.original_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.x
        orientation_q = msg.pose.pose.orientation
        orientation_q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_q)
        theta = yaw
        # Pose to numpy
        self.pose = np.array([[x, y, theta]]).T
        self.kf.update(self.pose)

    def right_wheel_callback(self, msg):
        self.wheel_speeds[0] = msg.data

    def left_wheel_callback(self, msg):
        self.wheel_speeds[1] = msg.data

    def speed_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        self.speeds[0] = linear_vel
        self.speeds[1] = angular_vel

    def apply_kalman_filter(self):
        prev_time = time.time()
        
        while not rospy.is_shutdown():
            if self.pose is not None  and self.wheel_speeds is not None:
                dt = time.time()-prev_time
                # breakpoint()
                if KF_METHOD == 'kelly': 
                    v = self.r*(self.wheel_speeds[0] + self.wheel_speeds[1])/2
                
                else:
                    v = self.speeds[0]
                    
                theta = self.pose[2,0]
                A = np.array([[1, 0, -v*math.sin(theta) * dt],
                              [0, 1, v*math.cos(theta) * dt],
                              [0, 0, 1]])

                if KF_METHOD == 'kelly':
                    D = np.array([[(self.r/2)*np.cos(self.pose[2][0])-((self.h*self.r)/self.d)*np.sin(self.pose[2][0]), (self.r/2)*np.cos(self.pose[2][0])+((self.h*self.r)/self.d)*np.sin(self.pose[2][0])],
                                [(self.r/2)*np.sin(self.pose[2][0])+((self.h*self.r)/self.d)*np.cos(self.pose[2][0]), (self.r/2)*np.sin(self.pose[2][0])-((self.h*self.r)/self.d)*np.cos(self.pose[2][0])]])
                    
                    B = np.array([D[0],
                                  D[1],
                                  self.phi[0]])
                    B = B * dt
                    inputs = self.wheel_speeds
                
                else:
                    B = np.array([[np.cos(theta),np.sin(theta),0],
                                 [0,0,1]]).T
                    
                    inputs = self.speeds
                
                x_hat = self.kf.predict(A = A, B = B, u = inputs)

                # Implement Kalman filter for state estimation
                odom_msg = Odometry()
                odom_msg.child_frame_id = self.original_odom.child_frame_id
                odom_msg.header = self.original_odom.header

                odom_msg.pose.pose.position.x = x_hat[0][0]
                odom_msg.pose.pose.position.y = x_hat[1][0]
                odom_msg.pose.pose.orientation.w = x_hat[2][0] #TODO: Convert angle to quaternion for finished odom OR puiblish as pose.

                prev_time = time.time() 
                self.filter_pub.publish(odom_msg)
                
if __name__ == "__main__":
    filter = FilterNode()
    filter.apply_kalman_filter()