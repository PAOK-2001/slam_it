import rospy
import math
import numpy as np
import rospy.rosconsole
import time
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Quaternion
from utils.kalman_filter import KalmanFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler

KF_METHOD = 'jacobian'
# Filter Node
class FilterNode:
    def __init__(self):
        # ROS params
        rospy.init_node('filter_node')
        self.rate = rospy.Rate(100)
        # Robot params
        _WHEELRADIUS = rospy.get_param("/wheel_radius")
        _WHEELBASE = rospy.get_param("/wheel_base")
        self.r = _WHEELRADIUS
        self.d = _WHEELBASE
        self.h = 0.001
        self. phi = np.array([[_WHEELRADIUS/_WHEELBASE, -_WHEELRADIUS/_WHEELBASE]])
        # Filter params
        self.obs_mat = np.eye(3, dtype= np.float32)
        self.state_covar = np.eye(3, dtype= np.float32)

        self.process_covar = np.array([[0.1, 0, 0],
                                       [0, 0.1, 0],
                                       [0, 0, 0.7]], dtype= np.float32) # Q

        self.measurments_covar = np.array([[0.07, 0 ,0],
                                           [0, 0.07, 0],
                                           [0, 0, 0.007]], dtype= np.float32)
        # Messages
        self.original_pose =  None
        self.prev_pose = None
        self.cmd_vel = None
        # State vectors
        self.pose = None
        self.wheel_speeds = np.array([[0.0, 0.0]], dtype= np.float32)
        self.speeds = np.array([[0.0,0.0]], dtype= np.float32)
        # Build KF 
        system_settings = (self.measurments_covar, self.process_covar, self.state_covar, self.obs_mat)
        self.x_hat = np.array([[0, 0, 0]], dtype = np.float32).T 
        self.kf = KalmanFilter(system_settings, self.x_hat)
        # Subscriber and publisher
        rospy.Subscriber(f"/rtabmap/localization_pose", PoseWithCovarianceStamped , self.pose_callback)
        rospy.Subscriber('wr', Float32, self.right_wheel_callback)
        rospy.Subscriber('wl', Float32, self.left_wheel_callback)
        rospy.Subscriber('cmd_vel', Twist, self.speed_callback)
        self.filter_pub = rospy.Publisher('filtered_pose', PoseWithCovarianceStamped, queue_size=10)

    def pose_callback(self, msg):
        # Unpack message
        self.original_pose = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_q)
        theta = yaw
        # Pose to numpy
        self.pose = np.array([[x, y, theta]]).T
        
    def right_wheel_callback(self, msg):
        self.wheel_speeds[0][0] = msg.data

    def left_wheel_callback(self, msg):
        self.wheel_speeds[0][1] = msg.data

    def speed_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        self.speeds[0][0] = linear_vel
        self.speeds[0][1] = angular_vel

    def get_pose_msg(self, robot_states):
        pose_msg = PoseWithCovarianceStamped()
        quat_msg = Quaternion()
        
        yaw = robot_states[2][0] 
        quat = quaternion_from_euler(0, 0, yaw)
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]
        
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = robot_states[0][0]
        pose_msg.pose.pose.position.y = robot_states[1][0]
        pose_msg.pose.pose.orientation = quat_msg
        
        return pose_msg
        
    def apply_kalman_filter(self):
        prev_time = time.time()
        while not rospy.is_shutdown():
            if self.pose is not None  and self.wheel_speeds is not None:
                dt = time.time()-prev_time
                if KF_METHOD == 'kelly': 
                    v = self.r*(self.wheel_speeds[0][0] + self.wheel_speeds[0][1])/2
                    
                else:
                    v = self.r*(self.wheel_speeds[0][0] + self.wheel_speeds[0][1])/2
            
                theta = self.pose[2][0]
                A = np.array([[0, 0, 0],
                              [0, 0, 0],
                              [0, 0, 0]]) #TODO: add dt to system transition mat

                if KF_METHOD == 'kelly':
                    D = np.array([[(self.r/2)*np.cos(theta)-((self.h*self.r)/self.d)*np.sin(theta), (self.r/2)*np.cos(theta)+((self.h*self.r)/self.d)*np.sin(theta)],
                                [(self.r/2)*np.sin(theta)+((self.h*self.r)/self.d)*np.cos(theta), (self.r/2)*np.sin(theta)-((self.h*self.r)/self.d)*np.cos(theta)]])
                    
                    B = np.array([D[0],
                                  D[1],
                                  self.phi[0]])
                    B = B * dt
                    inputs = self.wheel_speeds
                
                else:
                    inputs = self.wheel_speeds.T
                    dim_mat = np.array([[self.r/2, self.r/2],
                                        [self.r/self.d, -self.r/self.d]])
                    B = np.array([[np.cos(theta),0],
                                 [np.sin(theta),0],
                                 [0, 1]])
                    B = np.dot(B, dim_mat)

                self.x_hat  = self.kf.predict(A = A, B = B, u = inputs, dt = dt)
                robot_theta = self.x_hat [2][0]

                if(robot_theta > math.pi):
                    robot_theta = robot_theta - 2*math.pi

                elif robot_theta < - math.pi: 
                    robot_theta = robot_theta + 2*math.pi

                self.x_hat [2][0] =robot_theta

                if self.original_pose != self.prev_pose:
                    self.prev_pose = self.original_pose
                    self.x_hat = self.kf.update(self.pose)

                if abs(self.x_hat[2][0] - self.pose[2][0]) > np.pi/4: 
                    self.x_hat = self.pose
                    

                pose_msg = self.get_pose_msg(self.x_hat)
                self.filter_pub.publish(pose_msg)
                prev_time = time.time() 
                self.rate.sleep()
                
if __name__ == "__main__":
    filter = FilterNode()
    filter.apply_kalman_filter()