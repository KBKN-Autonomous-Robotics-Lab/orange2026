import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class PotholeDisplay(Node):
    def __init__(self):
        super().__init__('pothole_display')

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


        main_loop = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # pothole color white
        #white_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # whiteの抽出
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 60, 255])
        #white_mask = cv2.inRange(white_blur, lower_white, upper_white)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 高さ93cmで約300cm先から地面が見える。視線の下端が地面よ交わる角度は約17.2度
        max_short_axis = 95
        min_short_axis = 35
        max_long_axis = 150
        min_long_axis = 35

        for cnt in contours:
            #max_ellipse_area = (np.pi/4) * (max_long_axis) * (max_short_axis)
            #min_ellipse_area = (np.pi/4) * (min_long_axis) * (min_short_axis)
            if len(cnt) < 5:
                continue
            
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue
            
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.65:
                continue
            
            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (w, h) , angle = ellipse
            major = max(w, h)
            minor = min(w, h)
            
            ratio = minor / major
            if ratio > 2.0:
                continue
            
            
            cv2.ellipse(frame, ellipse, (0, 255, 0), 10)

    
        cv2.imshow("pothole", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDisplay()
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


