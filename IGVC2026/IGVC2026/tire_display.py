import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from ultralytics import YOLO

class TireDisplay(Node):
    def __init__(self):
        super().__init__('tire_display')
        # camera
        # robot
        #self.cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        # my pc
        self.cap = cv2.VideoCapture('/dev/sensors/camera', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            rclpy.shutdown()

        self.model = YOLO("yolov8x-oiv7.pt")
        self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        results = self.model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                if cls_id == 536 and conf >= 0.5:
                    label = f"Tire"
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)

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
    tire_display = TireDisplay()
    try:
        rclpy.spin(tire_display)
    finally:
        tire_display.cap.release()
        cv2.destroyAllWindows()
        tire_display.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()      

