import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
import easyocr
import re
import tkinter as tk
from threading import Thread
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from my_msgs.action import StopFlag



# グローバル変数（GUIで表示するため）
recognized_text  = "Initializing..."

# GUI
def start_gui():
    global recognized_text
    root = tk.Tk()
    root.title("Detected Text on Stop Sign")
    label = tk.Label(root, text=recognized_text, font=("Helvetica", 32))
    label.pack(padx=20, pady=20)

    def update_label():
        label.config(text=recognized_text)
        root.after(200, update_label)

    update_label()
    root.mainloop()


class SignDetection(Node):

    def __init__(self):
        super().__init__('sign_detection')

        # publisher
        self.publisher_ = self.create_publisher(String, 'sign_text', 10)
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['en'])

        # action 
        #self.action_client = ActionClient(self, StopFlag, 'stop_flag')
        #self.action_client.wait_for_server()

        # camera
        # robot
        self.cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        # my pc
        #self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
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

        # timer
        self.timer = self.create_timer(1.0, self.image_callback)

        # status 
        self.previous_status = None
        self.stop = False
        self.stop_buffer = []
        #self.buffer_size = 3


    def image_callback(self):
        global recognized_text
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Frame capture failed")
            return
        
        results = self.model(frame, verbose=False)

        status, raw_text, stop_sign_img = self.detect_stop_sign(frame, results)
        """
        if normalized_text == "stop" and confidence > 0.5:
            self.stop_buffer.append(True)
        else:
            self.stop_buffer.append(False)

        if len(self.stop_buffer) > 5:
            self.stop_buffer.pop(0)
        """

        if self.stop_buffer.count(True) > len(self.stop_buffer) / 2:
            status = "Stop"
        else:
            status = "Go"

        if status != self.previous_status:
            self.get_logger().info(f'Now status: {status}')
        """
        if status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_action_request()

        if status == "Go" and self.previous_status == "Stop":
            self.stop = False
            self.send_action_request()
        """
        self.previous_status = status

            

        recognized_text = raw_text if raw_text else "No sign"

        # 映像表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

        
    def detect_stop_sign(self, frame, results):
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
            #margin = 10
            #cropped = frame[y1-margin:y2+margin, x1-margin:x2+margin]
            #ocr_results = self.reader.readtext(cropped)
            for (bbox, text, confidence) in ocr_results:
                normalized_text = re.sub(r'[^a-z]', '', text.strip().lower())
                # 4文字でないものは無視
                if len(normalized_text) != 4:
                    continue

                self.get_logger().info(f'OCR検出結果: {text} → 正規化: {normalized_text} (信頼度: {confidence:.2f})')

                if normalized_text == "stop" and confidence > 0.8:
                    return "Stop", normalized_text, frame  # topic stop GUI words

                if normalized_text in ["soup", "igvc"]:
                    return "Go", normalized_text, frame

            return "Go", "", None
        else:
            return "Go", "", None  
    """
    def delayed_action_send(self):
        self.send_action_request()

    def send_action_request(self):
        goal_msg = StopFlag.Goal()

        if self.stop:
            goal_msg.a = 1

        else:
            goal_msg.a = 0

        self.action_client.wait_for_server()
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
    """

def main(args=None):
    rclpy.init(args=args)

    # GUI
    gui_thread = Thread(target=start_gui, daemon=True)
    gui_thread.start()

    stop_sign_detector = SignDetection()
    rclpy.spin(stop_sign_detector)

    stop_sign_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
 
