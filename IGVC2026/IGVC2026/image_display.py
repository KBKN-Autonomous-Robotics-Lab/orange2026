import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String
from collections import deque
import cv2
import time
import numpy as np
from ultralytics import YOLO
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from my_msgs.action import StopFlag

class HumanDetectionCamera(Node):
    def __init__(self):
        super().__init__('detection_camera')

        # ================= QoS =================
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ================= Publishers =================
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', qos)
        self.status_pub = self.create_publisher(String, 'now_status', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/tire_points', 1)
        self.status_publisher = self.create_publisher(String, 'all_status', 10)

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
        self.model = YOLO("yolov8n.pt")  # 人・停止標識など検出可能

        # ================= State =================
        self.tire_detected_queue = deque(maxlen=5)
        self.stop_sign_detected_queue = deque(maxlen=5)
        self.stop_sign_detected = False
        self.previous_status = "Go"
        self.stop = True
        self.stop_sign_latched = False
        self.last_stop_time = 0
        

        # ================= Main Loop =================
        self.create_timer(1.0 / 30.0, self.timer_callback)

        # ================= Action =================
        self.action_client = ActionClient(self, StopFlag, 'stop_flag')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return


        self.stop_reason = None
        results = self.model(frame, verbose=False)
        # ================= Human Detection =================
        human_status = self.detect_human(frame, results)
        tire_status = self.detect_tire(frame)
        stop_sign_status = self.detect_stop_sign(frame, results)

         # 画像表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)



    # 人検知
    def detect_human(self, frame, results):
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
                    y_start = h_cropped // 6
                    y_end = int(h_cropped - (h_cropped / 3.6))
                    trimmed_img = cropped_img[y_start:y_end, :]

                    top = y1 + y_start
                    bottom = y1 + y_end
                    cv2.rectangle(frame, (x1, top), (x2, bottom), (0,255,0),2)

        

        
    # タイヤ検知
    def detect_tire(self, frame):
        tire_status = "Go"
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 70]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
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
            
            ratio = w / h
            if ratio < 0.5 or ratio > 1.5:
                continue

            

            cv2.circle(frame, (int(xc), int(yc)), int(radius), (0, 255, 0), 2)
            

    
    # stop sign検知
    def detect_stop_sign(self, frame, results):
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 11 and conf > 0.3:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    if w * h > 2000:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)




    


def main(args=None):
    rclpy.init(args=args)
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