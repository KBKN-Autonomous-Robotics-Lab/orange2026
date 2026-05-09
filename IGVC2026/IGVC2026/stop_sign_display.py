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
        
        # YOLO
        self.model = YOLO("yolov8n.pt")

        # timer
        self.timer = self.create_timer(1.0, self.image_callback)



    def image_callback(self):
        global recognized_text
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Frame capture failed")
            return
        
        results = self.model(frame, verbose=False)

        raw_text, stop_sign_img = self.detect_stop_sign(frame, results)

            

        recognized_text = raw_text if raw_text else "No sign"

        # 映像表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

    def detect_stop_sign(self, frame, results):
        crop = None

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                if cls_id == 11 and conf > 0.3:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1

                    if w * h > 2000:
                        crop = frame[y1:y2, x1:x2]
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        break

    
        if crop is None:
            return "", None

        ocr_results = self.reader.readtext(crop)

        for (bbox, text, confidence) in ocr_results:
            normalized_text = re.sub(r'[^a-z]', '', text.strip().lower())

            if len(normalized_text) != 4:
                continue

            self.get_logger().info(
                f'OCR検出結果: {text} → 正規化: {normalized_text} (信頼度: {confidence:.2f})'
            )

            return normalized_text, frame

        return "", None

        
    
 

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
 
