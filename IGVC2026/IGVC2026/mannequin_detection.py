import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String, Bool
from collections import deque
import cv2
import time
import easyocr
import re
import tkinter as tk
from threading import Thread
import numpy as np
from ultralytics import YOLO
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from my_msgs.action import StopFlag

class HumanDetection(Node):
    def __init__(self):
        super().__init__('human_detection')

        # camera
        self.cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        # YOLO
        self.model = YOLO("yolov8n.pt")

        # mannequin status publisher
        self.publisher = self.create_publisher(String, 'mannequin_status', 10)

        # action
        #self.action_client = ActionClient(self, StopFlag, 'stop_flag')
        #self.action_client.wait_for_server()
        #self.goal_in_progress = False

        # main loop
        detect_timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        # status
        self.current_state = "Go"
        self.last_stop_time = 0.0
        self.previous_status = "Go"
        self.stop = False
        self.human_detected_queue = deque(maxlen=10)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        # resultsに検出結果が入る
        results = self.model(frame, verbose=False)

        human_status = self.detect_human(frame, results)
        #detected = human_status
        
        self.publisher.publish(String(data=human_status))
        
        #final_status = "Stop" if (self.current_state == "Stop") else "Go"
        #final_status = "Stop" if (human_status == "Stop") else "Go"


        #if final_status != self.previous_status:
            #self.get_logger().info(f'status: {final_status}')


        #self.previous_status = final_status
                # 画像表示
        #cv2.imshow("frame", frame)
        #cv2.waitKey(1)




    def detect_human(self, frame, results):
        human_detected = False
        #K = 950 # キャリブレーション K = h * distance
        # K = 800で人からの停止位置1.9m前後 K > 800 good

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # person
                    human_detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1


                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    #cv2.rectangle(frame, (x1, top), (x2, bottom), (0,255,0),2)
                        
        
        self.human_detected_queue.append(human_detected)
        status = "Stop" if any(self.human_detected_queue) else "Go"

        return status

    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetection()
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
