import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String, Bool, Int32
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

class StopsignDetection(Node):
    def __init__(self):
        super().__init__('stop_sign_detection')

        # camera
        # my pc
        #self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
        # robot
        self.cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        self.reader = easyocr.Reader(['en'])

        # YOLO
        self.model = YOLO("yolov8n.pt")

        # main loop
        detect_timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        #sim_timer = self.create_timer(1.0 / 30.0, self.simulate)
        # publisher
        self.publisher = self.create_publisher(String, 'stop_sign_flag', 10)

        # action
        #self.action_client = ActionClient(self, StopFlag, 'stop_flag')
        #self.action_client.wait_for_server()

        # first status
        self.previous_stop_sign_status = None
        self.stop_sign_detected_queue = deque(maxlen=5)
        self.current_stop_sign_state = "Go"
        self.previous_status = "Go"


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        #results = self.model(frame, verbose=False)
        #stop_sign_status = self.detect_stop_sign(frame, results)
        stop_sign_status = self.detect_stop_sign(frame)
        self.publisher.publish(String(data=stop_sign_status))
        self.get_logger().info(f'Published stopsign status: {stop_sign_status}')
        

        """
        now = time.time()

        if stop_sign_status == "Detected":
            self.current_stop_sign_state = "Stop"
            self.last_stop_time = now

        else:
            if self.current_stop_sign_state == "Stop":
                if now - self.last_stop_time > 5.0:
                    self.current_stop_sign_state = "Go"

                else:
                    pass

            else:
                self.current_stop_sign_state = "Go"
        

        final_status = "Stop" if (self.current_stop_sign_state == "Stop") else "Go"

        if final_status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_stop_sign_action()

        if final_status == "Go" and self.previous_status == "Stop":
            self.stop = False
            self.send_stop_sign_action()


        self.previous_status = final_status
        """

               # 画像表示
        #cv2.imshow("frame", frame)
        #cv2.waitKey(1)    


        




    def detect_stop_sign(self, frame):
        stop_sign_detected = False
        results = self.model(frame, verbose=False)
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 11 and conf > 0.3:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    crop = frame[y1:y2, x1:x2]
                    if w * h > 2000:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    if frame is not None:
                        ocr_results = self.reader.readtext(crop)

                        for (bbox, text, confidence) in ocr_results:
                            normalized_text = re.sub(r'[^a-z]', '', text.strip().lower())
                            if len(normalized_text) != 4:
                                continue
                            self.get_logger().info(f'OCR検出結果: {text} → 正規化: {normalized_text} (信頼度: {confidence:.2f})')

                            if normalized_text == "stop" and confidence > 0.5:
                                stop_sign_detected = True
                    else:
                        stop_sign_detected = False
        
        self.stop_sign_detected_queue.append(stop_sign_detected)
        status = "Detected" if any(self.stop_sign_detected_queue) else "Go"
    
        return status

    
    """
    # action
    def delayed_action_send(self):
        self.send_action_request()


    def send_stop_sign_action(self):
        goal_msg = StopFlag.Goal()
        if self.stop:
            goal_msg.a = 2   # stop sign special code

        else:
            goal_msg.a = 0
        #self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback.feedback.rate}")


        # 結果を受け取るコールバック関数
    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sum}")
        #if self.stop_sign_latched and (time.time() - self.last_stop_time > 5.0):
        if result.sum == 999:
            self.stop_sign_latched = False
            self.get_logger().info("Stop sign released")
    """
    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = StopsignDetection()
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
