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
        if human_status == "Stop":
            self.stop_reason = "human"
        # ================= Tire Detection ================
        elif tire_status == "Stop":
            self.stop_reason = "tire"

        # ================= Stop Sign Detection =================
        if stop_sign_status == "Stop":
            if time.time() - self.last_stop_time > 5:
                self.get_logger().info("Stop sign detected")
                self.send_stop_sign_action()
                self.last_stop_time = time.time()

        # ================= Final Decision =================
        final_status = "Stop" if (
            human_status == "Stop" or tire_status == "Stop") else "Go"
        
       

        # Publish status
        self.status_pub.publish(String(data=final_status))
        if final_status != self.previous_status:
            self.get_logger().info(f'Status: {final_status}')
        
        
        
        """
        # stopsignを分けnai場合
        if final_status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_action_request()
        """
        # stopsignを分ける場合
        if final_status == "Stop" and self.previous_status != "Stop":
            self.stop = True
            self.send_action_request()

        if final_status == "Stop" and self.previous_status == "Stop":
            pass
            

        if final_status == "Go" and self.previous_status == "Stop":
            self.stop = False
            self.send_action_request()
        
        self.previous_status = final_status

         # 画像表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

    # ==========================================================
    # Individual Detection Methods
    # ==========================================================
    def detect_human(self, frame, results):
        #results = self.model(frame, verbose=False)
        status = "Go"

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    if w > 50 and h > 200:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        status = "Stop"
        return status

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

    def detect_stop_sign(self, frame, results):
        #stop_sign_status = "Go"
        #results = self.model(frame, verbose=False)
        stop_sign_detected = False

        for result in results:
            classes = result.boxes.cls.cpu().numpy()
            if any(int(cls) == 11 for cls in classes):
                stop_sign_detected = True
                break
        # ラッチsyori
        # 一度検出したらラッチ
        if stop_sign_detected and not self.stop_sign_latched:
            self.stop_sign_latched = True
            #self.last_stop_time = time.time()
            return "Stop"
        # ラッチされていたら常にストップ
        #if self.stop_sign_latched:
            #stop_sign_status = "Stop"
        
        """
        self.stop_sign_detected_queue.append(stop_sign_detected)
        if any(self.stop_sign_detected_queue):
            stop_sign_status = "Stop"
        """
        return "Go"



    def delayed_action_send(self):
        self.send_action_request()
    
    # stop_reasonがNot stop_signサーバーにアクションを送信する関数
    def send_action_request(self):
        goal_msg = StopFlag.Goal()
        
        # stop変数の状態でaの値を決定
        if self.stop: # True
            goal_msg.a = 1  # stop (stop_reason = human and tire)
        else: # False
            goal_msg.a = 0  # go
            
        # traffic_action変数の状態でbの値を決定
        #if self.traffic_action:
        #    goal_msg.b = 0  # start judge
        #else:
        #    goal_msg.b = 1  # 

        # アクションサーバーが利用可能になるまで待機
        self.action_client.wait_for_server()

        # アクションを非同期で送信
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    # stop_reasonがstop_signのときのサーバーにアクションを送信する関数
    def send_stop_sign_action(self):
        goal_msg = StopFlag.Goal()
        goal_msg.a = 2   # stop sign special code
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)
    


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