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
import time


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
        start_time = time.time()

    
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
                    label = f"human vest"
                    cv2.rectangle(frame, (x1, top), (x2, bottom), (0,255,0),2)
                    cv2.putText(
                        frame,
                        label,
                        (x1,y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0,255,0),
                        2
                    )

        
        """
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                if cls_id == 0 and conf > 0.5:
                    # 人領域の切り取り
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(frame.shape[1], x2)
                    y2 = min(frame.shape[0], y2)
                    #
                    person_h = y2 - y1
                    h_start = int(person_h * 0.2)
                    h_end = int(person_h * 0.7)
                    crop_y1 = y1 + h_start
                    crop_y2 = y1 + h_end
                    person_crop = frame[crop_y1:crop_y2, x1:x2]
                    center_x = person_crop.shape[1] // 2
                    center_y = person_crop.shape[0] // 2

                    detected = False

                    
                    
                    # hsv変換
                    hsv = cv2.cvtColor(person_crop, cv2.COLOR_BGR2HSV)

                    lower_yellow = np.array([20, 100, 100])
                    upper_yellow = np.array([35, 255, 255])
                    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
                    lower_orange = np.array([5, 100, 100])
                    upper_orange = np.array([15, 255, 255])
                    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
                    mask = cv2.bitwise_or(mask_yellow, mask_orange)

                    kernel = np.ones((5,5), np.uint8)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            if abs(cx - center_x) > person_crop.shape[1] * 0.3:
                                continue

                        if cv2.contourArea(largest_contour) > 500:
                            x, y, w, h = cv2.boundingRect(largest_contour)
                            aspect_ratio = h / w
                            if aspect_ratio < 0.8:
                                continue
                            global_x1 = x1 + x1
                            global_y1 = crop_y1 + y1
                            global_x2 = global_x1 + w
                            global_y2 = global_y1 + h
                            detected = True
                            self.vest_history.append(detected)
                            if self.vest_history.count(True) >= 3:
                                cv2.rectangle(frame, (global_x1, global_y1), (global_x2, global_y2), (0,255,0),2)

            """
        end_time = time.time()
        elapsed = (end_time - start_time)*1000
        print(f"Inference +  drawing time: {elapsed:.2f} ms")

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

                    


