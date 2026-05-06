import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String, Header
from collections import deque
import cv2
import numpy as np
import math
import struct
from collections import deque
from ultralytics import YOLO
import easyocr
import re
import tkinter as tk
from threading import Thread
from cv_bridge import CvBridge
from collections import deque

class HumanDisplay(Node):
    def __init__(self):
        super().__init__('human_display')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.image_publish = self.create_publisher(CompressedImage, 'camera/image/compressed', qos)
        

        self.cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        #self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        self.model = YOLO("yolov8n.pt")
        self.create_timer(1.0/30.0, self.timer_callback)
        self.vest_history = deque(maxlen=5)


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # human detection and vest display
    
        results = self.model(frame, verbose=False)
        
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1

                # size and priority
                # w > 50 h > 470 -> stop 5feet , w > 50 h > 240 -> change lane 10feet
                if cls_id == 0 and conf >= 0.5 and (w > 50 and h > 200): # width , height -> pixel
                    cropped_img = result.orig_img[y1:y2, x1:x2]
                    h_cropped, w_cropped = cropped_img.shape[:2]
                    y_start = h_cropped // 7
                    y_end = int(h_cropped - (h_cropped / 1.8))
                    trimmed_img = cropped_img[y_start:y_end, :]

                    top = y1 + y_start
                    bottom = y1 + y_end
                    cv2.rectangle(frame, (x1, top), (x2, bottom), (0,255,0),2)

                    label = "vest"
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 10),  # バウンディングボックスの上
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA
                    )

        

        cv2.imshow("camera", frame)
        cv2.waitKey(1)





def main(args=None):
    rclpy.init(args=args)
    human_display = HumanDisplay()
    try:
        rclpy.spin(human_display)
    finally:
        human_display.cap.release()
        cv2.destroyAllWindows()
        human_display.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

                    


