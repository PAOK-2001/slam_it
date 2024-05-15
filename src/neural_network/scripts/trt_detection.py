#!/usr/bin/env python
import random
import rospy
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEBUG = True

class ObjectDetector:
    def __init__(self, engine_path):
        rospy.init_node('landmark_detector')
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # Camera params
        self.cv_image = None
        self.depth_image = None
        # Model params
        self.model = YOLO(engine_path, task='detect')
        self.confidence_thresh = 0.6
        self.class_dict = {0: 'Extinguisher', 1: 'Fire', 2: 'Person', 3: 'Pipe'}
        self.display_colors = [(128, 255, 128), (255, 128, 255), (85, 170, 255), (255, 255, 255)]

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
            cv2.putText(local_frame, self.class_dict[det_class], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        cv2.imshow("Landmark Detector", local_frame)
        cv2.waitKey(1)


    def publish_results(self, results):
        pass
    
    def camera_callback(self, frame):
        self.cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        
    def depth_callback(self, frame):
        self.depth_image = self.bridge.imgmsg_to_cv2(frame, "passthrough")

    def detect(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None and self.depth_image is not None:
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
            self.rate.sleep()
        
    def get_distance_to_pixel(self, x,y):
        depth_array = np.array(self.depth_image, dtype=np.float32)
        return depth_array[y,x]/1000


if __name__ == "__main__":
    detector = ObjectDetector(engine_path='/home/it/slam_it/src/neural_network/nets/weights/best.engine')
    detector.detect()
