import rclpy
from rclpy.node import Node
import numpy as np
import cv2

class PotholeDisplay(Node):
    def __init__(self):
        super().__init__('pothole_display')
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


        self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0,0,200])
        upper_white = np.array([180,50,255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            x,y,w,h = cv2.boundingRect(cnt)
            frame_h, frame_w = frame.shape[:2]
            touching_edge = x <= 0 or y <= 0 or (x+w) >= frame_w or (y + h) >= frame_h
            if area < 3000:
                continue
            
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.6:
                continue

            (xc, yc), radius = cv2.minEnclosingCircle(cnt)
            if radius < 20:
                continue
            
            aspect_ratio = h / w
            if aspect_ratio < 0.8 or aspect_ratio > 1.2:
                continue

            x1, y1 = x, y
            x2, y2 = x + w, y + h

            center = (int(xc), int(yc))
            radius = int(radius)

            label = f"pothole"
            cv2.circle(frame, (x1,y1), (x2,y2), (0,255,0), 2)

            cv2.putText(
                frame,
                label,
                (x1,y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0,255,0),
                2
            )



        # display 
        cv2.imshow("tire display", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDisplay()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()      

