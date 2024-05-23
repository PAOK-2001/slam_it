#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from cv_bridge import CvBridge

DEBUG = False

class ObjectDetector:
    def __init__(self, engine_path):
        self.slam_namespace = "rtabmap"
        rospy.init_node('landmark_detector')
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher("/landmarks", MarkerArray, queue_size=10 )
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.camera_params = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cam_info_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber(f"/{self.slam_namespace}/localization_pose", PoseWithCovarianceStamped , self.pose_callback)
        # Camera params
        self.cv_image = None
        self.depth_image = None
        # Model params
        self.model = YOLO(engine_path, task='detect')
        self.confidence_thresh = 0.75

        self.class_dict = {0: 'Extinguisher', 
                           1: 'Fire', 
                           2: 'Person', 
                           3: 'Pipe'}
        
        self.id_dict  = {'Extinguisher': 0, 
                         'Fire': 0, 
                         'Person': 0, 
                         'Pipe': 0}
        
        self.display_colors = [(0, 0, 255), (18, 88, 219), (29, 120, 24), (224, 103, 22)]
        self.min_dist = 0
        
        self.robot_position = None
        self.camera_k = None #fx,fy,cx,cy
        self.max_dist = 2

        # TF attributes
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        self.robot_position = pose

    def debug_show(self, detection_info, frame):
        local_frame = frame.copy()
        detected_objects, bounding_boxes = detection_info
        for (det_class, box) in zip(detected_objects, bounding_boxes):
            box = box.astype(int)
            class_id = int(det_class)
            color = self.display_colors[int(class_id)]
            p1, p2 = ((int(box[0]-box[2]/2), int(box[1]-box[3]/2)), 
                      (int(box[0] + box[2]/2), int(box[1]+box[3]/2)))            
            cv2.rectangle(local_frame, p1, p2, color, 2)
            cv2.circle(local_frame, (box[0], box[1]), 1, (255,0,0),9)
            cv2.putText(local_frame, self.class_dict[det_class], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        cv2.imshow("Landmark Detector", local_frame)
        cv2.waitKey(1)

    def create_marker(self, id, x, y, z, ns, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.a = 1.0
        marker.color.r = color[2]/255.0
        marker.color.g = color[1]/255.0
        marker.color.b = color[0]/255.0
        return marker

    def transform_point(self, point, target_frame = 'map'):

        try:
            transformed_point = self.tfBuffer.transform(point, target_frame)
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to transform point to map")
        return None
            
    def publish_results(self, detection_info):
        markers = MarkerArray()
        detected_objects, bounding_boxes = detection_info

        for (det_class, box) in zip(detected_objects, bounding_boxes):
            # Unpack id and get local id
            class_id = int(det_class)
            color = self.display_colors[class_id]
            class_id = self.class_dict[class_id]
            local_id = self.id_dict[class_id]
            self.id_dict[class_id] += 1
            # Get pose
            box = box.astype(int)
            # Construct marker
            pos = self.get_pos_from_pixel(box[0], box[1])
            if pos is not None:
                x,y,z = pos
                if x <= self.max_dist:
                    marker = self.create_marker(id= local_id, 
                                                x=x, 
                                                y=y, 
                                                z=z, 
                                                ns= class_id, 
                                                color= color)
                    markers.markers.append(marker)
        self.marker_pub.publish(markers)
    
    def camera_callback(self, frame):
        self.cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")

    def depth_callback(self, frame):
        self.depth_image = self.bridge.imgmsg_to_cv2(frame, "passthrough")

    def cam_info_callback(self, params):
        consts = params.K
        self.camera_k = [consts[0],consts[4],consts[2],consts[5]]

    def detect(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None and self.depth_image is not None and self.camera_k:
                results = self.model.predict(self.cv_image, conf = self.confidence_thresh, verbose = False)
                if len(results)>0: 
                    results = results[0]
                else:
                    pass
                
                detected_objects = results.boxes.cls.cpu().numpy()
                bounding_boxes = results.boxes.xywh.cpu().numpy()

                detection_info = (detected_objects, bounding_boxes)
                
                if DEBUG:
                    self.debug_show(detection_info, self.cv_image)
                self.publish_results(detection_info)
            self.rate.sleep()
        
    def get_pos_from_pixel(self, x,y):
        depth_array = np.array(self.depth_image, dtype=np.float32)
        pos_x = depth_array[y,x]/1000
        pos_y = (-x + self.camera_k[2]) * pos_x / self.camera_k[0]
        pos_z = (-y + self.camera_k[3]) * pos_x / self.camera_k[1]
        # Construct point
        point = PointStamped()
        point.header.frame_id = "camera_link"
        point.point.x = pos_x 
        point.point.y = pos_y
        point.point.z = pos_z
        tf_point = self.transform_point(point)
        if tf_point is not None:
            x, y, z = tf_point
            return x, y, z
        return None


if __name__ == "__main__":
    detector = ObjectDetector(engine_path='/home/it/slam_it/src/landmark_detector/nets/weights/best.engine')
    detector.detect()
