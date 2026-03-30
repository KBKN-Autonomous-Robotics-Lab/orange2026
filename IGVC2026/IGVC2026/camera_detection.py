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

class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detection')

        # qos
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # publihser
        self.image_publihser = self.create_publisher(CompressedImage, 'camera/image/compressed', qos)
        self.status_publisher = self.create_publisher(String, 'now_status', 10)
        # test publisher
        self.pub = self.create_publisher(Bool, '/simulate_white_line', 10)
    
        # camera
        self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        self.reader = easyocr.Reader(['en'])

        # YOLO (human and stop sign)
        self.model = YOLO("yolov8n.pt")

        # first status
        self.tire_detected_queue = deque(maxlen=10)
        self.current_human_state = "Go"
        self.human_detected_queue = deque(maxlen=10)
        self.stop_sign_detected_queue = deque(maxlen=5)
        self.yellow_line_detected_queue = deque(maxlen=5)
        self.stop_sign_detected = False
        self.previous_status = "Go"
        self.stop = True
        self.stop_sign_latched = False
        self.stopsign_stop = False
        self.previous_stop_sign_status = "Not_detected"
        self.previous_line_status = "None"
        self.count = 0
        self.last_stop_time = 0.0

        


        # main loop
        detect_timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        #sim_timer = self.create_timer(1.0 / 30.0, self.simulate)

        # action
        self.action_client = ActionClient(self, StopFlag, 'stop_flag')
        self.action_client.wait_for_server()


    # test
    """
    def simulate(self):
        msg = Bool()
        if self.count % 100 == 0:
            msg.data = True
            self.pub.publish(msg)
            self.get_logger().info("Published True to /simulate_white_line")
            self.get_logger().info(f"/simulate_white_line: {msg}")
        #time.sleep(5)
        elif self.count % 100 == 50:
            msg.data = False
            self.pub.publish(msg)
            self.get_logger().info("Published False to /simulate_white_line")
            self.get_logger().info(f"/simulate_white_line: {msg}")
        
        self.count += 1
        #time.sleep(5)
    """
    

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, verbose=False)
        self.stop_reason = None

        # Call status function
        human_status = self.detect_human(frame, results)
        stop_sign_status = self.detect_stop_sign_status(frame, results)
        train_status = self.detect_train(frame, results)
        line_status = self.detect_yellow_line(frame)
        
        if human_status == "Stop":
            self.stop_reason = "human"

        elif stop_sign_status == "Detected":
            self.stop_reason = "stop_sign"

        elif train_status == "Stop":
            self.stop_reason = "train"

        now = time.time()
        

        if human_status == "Stop":
            self.current_human_state = "Stop"
            self.last_stop_time = now

        else:
            if self.current_human_state == "Stop":
                if now - self.last_stop_time > 2.0:
                    self.current_human_state = "Go"

                else:
                    pass

            else:
                self.current_human_state = "Go"
        


        


                

        final_status = "Stop" if (
            self.current_human_state == "Stop" or train_status == "Stop"
        ) else "Go"

        # publish status
        if final_status != self.previous_status:
            self.get_logger().info(f'Status: {final_status} stop_reason: {self.stop_reason}')

        # to action
        if stop_sign_status == "Detected" and self.previous_stop_sign_status != "Detected":
            self.get_logger().info("Stop sign detected")
            self.send_stop_sign_action()
        
        elif line_status == "Detected" and self.previous_line_status != "Detected":
            self.get_logger().info("yellow_line_detected")
            self.send_yellow_line_action()
            
        elif final_status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_action_request()

        elif final_status == "Stop" and self.previous_status == "Stop":
            pass

        elif final_status == "Go" and self.previous_status == "Stop":
            self.stop = False
            self.send_action_request()

        self.previous_status = final_status
        self.previous_stop_sign_status = stop_sign_status
        self.previous_line_status = line_status

        # image display
        #cv2.imshow("camera", frame)
        #cv2.waitKey(1)

    def detect_human(self, frame, results):
        #results = self.model(frame, verbose=False)
        human_detected = False

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    if w > 50 and h > 200:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        human_detected = True
        
        self.human_detected_queue.append(human_detected)
        status = "Stop" if any(self.human_detected_queue) else "Go"

        return status
    """
    def detect_tire(self, frame):
        tire_status = "Go"
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 70]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tire_detected = False
        frame_h, frame_w = frame.shape[:2]
        ground_line = int(frame_h * 0.75)  # 下部25%を地面と仮定

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if y + h < ground_line:
                continue  # 地面近くにない輪郭は除外

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.7:
                continue

            (xc, yc), radius = cv2.minEnclosingCircle(cnt)
            if radius < 40:
                continue

            cv2.circle(frame, (int(xc), int(yc)), int(radius), (0, 255, 0), 2)
            tire_detected = True

        self.tire_detected_queue.append(tire_detected)
        if any(self.tire_detected_queue):
            tire_status = "Stop"
        return tire_status
    """
    """
    def detect_stop_sign_status(self, frame, results):
        #results = self.model(frame, verbose=False)
        stop_sign_detected = False
        

        for result in results:
            classes = result.boxes.cls.cpu().numpy()
            if any(int(cls) == 11 for cls in classes):
                stop_sign_detected = True
        
        if stop_sign_detected and not self.stop_sign_latched:
            self.stop_sign_latched = True

        self.stop_sign_detected_queue.append(stop_sign_detected)
        status = "Detected" if any(self.stop_sign_detected_queue) else "Not_detected"
    
        return status
    """
    def detect_stop_sign_status(self, frame, results):
        stop_sign_detected = False
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 11 and conf > 0.3:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    if w * h > 2000:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    if frame is not None:
                        ocr_results = self.reader.readtext(frame)

                        for (bbox, text, confidence) in ocr_results:
                            normalized_text = re.sub(r'[^a-z]', '', text.strip().lower())
                            if len(normalized_text) != 4:
                                continue
                            self.get_logger().info(f'OCR検出結果: {text} → 正規化: {normalized_text} (信頼度: {confidence:.2f})')

                            if normalized_text == "stop" and confidence > 0.8:
                                stop_sign_detected = True
                    else:
                        stop_sign_detected = False
        
        self.stop_sign_detected_queue.append(stop_sign_detected)
        status = "Detected" if any(self.stop_sign_detected_queue) else "Not_detected"
    
        return status
        

    
    def detect_train(self, frame, results):
        train_status = "Go"
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 6 and conf > 0.5:  # train
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    if w > 50 and h > 200:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        train_status = "Stop"
        return train_status

    # stopsign test
    def detect_yellow_line(self, frame):
        line_status = False
        yellow_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        lower_yellow = np.array([0, 100, 100])
        upper_yellow = np.array([80, 255, 255])
        yellow_mask = cv2.inRange(yellow_blur, lower_yellow, upper_yellow)
        yellow_edges = cv2.Canny(yellow_mask, 50, 150) 
        h, w = yellow_edges.shape 
        mask_yellow = np.zeros_like(yellow_edges) 
        cv2.rectangle(mask_yellow, (0, int(h*0.6)), (w, h), 255, -1) 
        yellow_edges = cv2.bitwise_and(yellow_edges, mask_yellow)

        yellow_lines = cv2.HoughLinesP(
            yellow_edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=30
        )

        if yellow_lines is not None and len(yellow_lines) > 0:
            line_status = True
            
        else:
            line_status = False

        self.yellow_line_detected_queue.append(line_status)    
        status = "Detected" if any(self.stop_sign_detected_queue) else "Not_detected"
 
        
        
        
        return 
    
    def delayed_action_send(self):
        self.send_action_request()

    # stop_reasonがNot stop_signサーバーにアクションを送信する関数
    def send_action_request(self):
        goal_msg = StopFlag.Goal()
        
        if self.stop:
            goal_msg.a = 1

        else:
            goal_msg.a = 0

        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def send_stop_sign_action(self):
        goal_msg = StopFlag.Goal()
        goal_msg.a = 2   # stop sign special code
        #self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.response_callback)
    
    # stopsign test
    def send_yellow_line_action(self):
        goal_msg = StopFlag.Goal()
        goal_msg.a = 3
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.response_callback)


    # フィードバックを受け取るコールバック関数
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
    node = CameraDetection()
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