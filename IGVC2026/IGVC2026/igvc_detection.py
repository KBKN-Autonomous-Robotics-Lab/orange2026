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



class HumanDetectionCamera(Node):

    def __init__(self):
        super().__init__('human_detection_camera')

        # ================= QoS =================
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ================= Publisher =================
        self.image_pub = self.create_publisher(
            CompressedImage,
            'camera/image/compressed',
            qos
        )
        self.status_pub = self.create_publisher(String, 'now_status', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/tire_points', 1)
        self.status_publisher = self.create_publisher(String, 'all_status',10)

        # ================= Camera =================
        self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        # ================= YOLO =================
        self.model = YOLO("yolov8n.pt")  # human and stop_sigm
        #self.sign_model = YOLO('yolov8x.pt')
        #self.sign_model.classes = [11]


        # ================= Robot State =================
        self.robot_pose = (0.0, 0.0, 0.0)

        # ================= histerisys =================
        self.tire_detected_queue = deque(maxlen=5)
        self.stop_sign_detected_queue = deque(maxlen=5)

        # ================= Main Loop =================
        self.create_timer(1.0 / 30.0, self.timer_callback)

        self.previous_status = "Go"

    # ==========================================================
    # Main Loop
    # ==========================================================
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        human_status = "Go"
       

        # ================= Human Detection =================
        results = self.model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                if cls_id == 0 and conf > 0.5:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w = x2 - x1
                    h = y2 - y1

                    if w > 50 and h > 200:
                        cv2.rectangle(frame, (x1, y1), (x2, y2),
                                      (0, 255, 0), 2)
                        human_status = "Stop"

        # ================= Tire Detection =================
        tire_status = "Go"

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 70])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        tire_detected = False

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # =======================円形度フィルタ============================
            if area < 3000:
                continue
            
            # ========== 高さフィルタ =============
            """
            x, y, w, h = cv2.boundingRect(cnt)
            frame_h, frame_w = frame.shape[:2]

            ground_line = int(frame_h * 0.75)
            if y + h < ground_line:
                continue
            """
            # ===========周囲長の計算=============
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue
            
            # ===================真円度の計算=========================
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.7:
                continue

            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius < 40:
                continue
            """
            aspect_ratio = h / w
            if aspect_ratio < 0.8 or aspect_ratio >1.2:
                continue
            """
            cv2.circle(frame, (int(x), int(y)),
                       int(radius), (0, 255, 0), 2)

            tire_detected = True
        
        self.tire_detected_queue.append(tire_detected)
        tire_status = "Stop" if any(self.tire_detected_queue) else "Go"
            
        # ============== stop_sign_detection ===============
        stop_sign_status = "Go"
        stop_sign_detected = False
        results = self.model(frame, verbose=False)
        
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()

            for cls in classes:
                if int(cls) == 11:
                    stop_sign_detected = True
                    break
        self.stop_sign_detected_queue.append(stop_sign_detected)
        stop_sign_status = "Stop" if any(self.stop_sign_detected_queue) else "Go"

        # ================= Final Decision =================
        final_status = "Stop" if (
            human_status == "Stop" or tire_status == "Stop" or stop_sign_status == "Stop"
        ) else "Go"  

        # 画像表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)
        
        self.status_pub.publish(String(data=final_status))
        if self.previous_status is not None and final_status != self.previous_status:
            self.get_logger().info(f'Status: {final_status}')

        self.previous_status = final_status
   
    

def main(args=None):
    rclpy.init(args=args)
    #gui_thread = Thread(target=start_gui, daemon=True)
    #gui_thread.start()
    node = HumanDetectionCamera()

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