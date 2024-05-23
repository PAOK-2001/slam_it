import cv2
from ultralytics import YOLO

if __name__ == "__main__":
    camera_port = '/dev/video4'
    engine_path = '/home/it/slam_it/src/neural_network/nets/weights/best.engine'

    model = YOLO(engine_path, task='detect')
    
    vid = cv2.VideoCapture(camera_port) 
  
    while(True): 
        ret, frame = vid.read() 
        results = model.predict(frame)
        breakpoint()
        print(results[0])
        