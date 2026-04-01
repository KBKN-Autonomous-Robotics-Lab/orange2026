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

        # action
        self.action_client = ActionClient(self, StopFlag, 'stop_flag')
        self.action_client.wait_for_server()
        self.goal_in_progress = False

        # main loop
        detect_timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        # status
        self.current_state = "Go"
        self.last_stop_time = 0.0
        self.previous_status = "Go"
        self.stop = False
        self.human_detected_queue = deque(maxlen=5)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        # resultsに検出結果が入る
        results = self.model(frame, verbose=False)

        human_status = self.detect_human(frame, results)
        #detected = human_status
        
        
        now = time.time()

        if human_status == "Stop":
            self.current_state = "Stop"
            self.last_stop_time = now
            
        else:
            if self.current_state == "Stop":
                if now - self.last_stop_time > 2.0:
                    self.current_state = "Go"

                else:
                    pass

            else:
                self.current_state = "Go"
        
        final_status = "Stop" if (self.current_state == "Stop") else "Go"
        #final_status = "Stop" if (human_status == "Stop") else "Go"


        if final_status != self.previous_status:
            self.get_logger().info(f'status: {final_status}')

        if final_status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_action_request()

        elif final_status == "Go" and self.previous_status == "Stop":
            self.stop = False
            self.send_action_request()

        self.previous_status = final_status
                # 画像表示
        cv2.imshow("frame", frame)
        cv2.waitKey(1)




    def detect_human(self, frame, results):
        human_detected = False
        K = 900 # キャリブレーション K = h * distance
        # K = 800で人からの停止位置1.9m前後 K > 800 good

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                   
                    if h > 0:
                        cropped_img = result.orig_img[y1:y2, x1:x2]
                        h_cropped, w_cropped = cropped_img.shape[:2]
                        y_start = h_cropped // 6
                        y_end = int(h_cropped - (h_cropped / 3.6))
                        trimmed_img = cropped_img[y_start:y_end, :]

                        top = y1 + y_start
                        bottom = y1 + y_end
                        human_h = h - 10
                        if h >= 475:
                            human_h = h + 5
                            distance = K / human_h
                            #self.get_logger().info(f'h={human_h}, distance={distance}')
                            if 1.5 <= distance <= 2.0:
                                human_detected = True

                        elif h < 475:
                            human_h = h - 10
                            distance = K / human_h
                            #self.get_logger().info(f'h={human_h}, distance={distance}')
                            if 1.5 <= distance <= 2.0:
                                human_detected = True


                    #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.rectangle(frame, (x1, top), (x2, bottom), (0,255,0),2)
                        
        
        self.human_detected_queue.append(human_detected)
        status = "Stop" if any(self.human_detected_queue) else "Go"

        return status          

    def delayed_action_send(self):
        self.send_action_request()

    # stop_reasonがNot stop_signサーバーにアクションを送信する関数
    def send_action_request(self):
        """
        if self.goal_in_progress:
            return
        
        self.goal_in_progress = True
        """
        goal_msg = StopFlag.Goal()
        
        if self.stop:
            goal_msg.a = 1

        else:# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。



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
        
    # 結果のコールバック
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sum}")
        #if self.stop_sign_latched and (time.time() - self.last_stop_time > 5.0):
        if result.sum == 999:
            self.stop_sign_latched = False
            self.get_logger().info("Stop sign released")
    
    
    

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
