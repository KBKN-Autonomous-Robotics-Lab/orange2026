import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from collections import deque


# ================= RealSense設定 =================

class TireDetect(Node):
    def __init__(self):
        super().__init__('tire_detect')


        self.create_timer(1.0 / 30.0, self.timer_callback)

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

        # status
        self.tire_detected_queue = deque(maxlen=5)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        # roi
        frame_h , frame_w = frame.shape[:2]
        roi = frame[int(frame_h*0.5):, :]
        # ROI が存在するかチェック
        if roi is None or roi.size == 0:
            return

        # HSV に変換（BGR 3チャンネル前提）
        if roi.shape[2] != 3:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0,0,0], dtype=np.uint8)
        upper_black = np.array([180,255,50], dtype=np.uint8)

        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        mask_black_inv = 255 - mask_black

        kernel = np.ones((5,5), np.uint8)
        mask_clean = cv2.morphologyEx(mask_black_inv, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)
        # グレースケール
        #gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # ブラー（ノイズ除去）
        #blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # エッジ検出
        #edges = cv2.Canny(blur, 50, 150)

        # mask
        #mask = edges.copy()
        
        #_, mask = cv2.threshold(hsv, 50, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            mask_clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 4. 面積・円形度・アスペクト比でフィルタ
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)


            if area < 1000:
                continue
            
            # 周囲長の計算
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue
            
            # 真円度の計算
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.4:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w == 0:
                continue
            aspect_ratio = float(h) / w

            if aspect_ratio < 0.5 or aspect_ratio > 2.0:
                continue
            
            if hierarchy[0][i][2] == -1:
                continue

            (xc, yc), radius = cv2.minEnclosingCircle(cnt)
            xc_global = int(xc)
            yc_global = int(yc + int(frame_h*0.5))
            cv2.circle(frame, (int(xc), yc_global), int(radius), (0, 255, 0), 2)


        

        # 画像表示
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TireDetect()

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