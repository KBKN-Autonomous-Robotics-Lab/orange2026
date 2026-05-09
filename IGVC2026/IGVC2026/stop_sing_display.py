import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import pytesseract


class SignDetection(Node):

    def __init__(self):
        super().__init__('sign_detection')

        # publisher
        self.publisher_ = self.create_publisher(String, 'sign_text', 10)

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

        # timer
        self.timer = self.create_timer(1/10.0, self.timer_callback)


    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Frame capture failed")
            return

        # grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # OCR
        text = pytesseract.image_to_string(gray, lang="eng")

        # publish
        msg = String()
        msg.data = text.strip()
        self.publisher_.publish(msg)

        # debug表示
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

        self.get_logger().info(f"OCR: {msg.data}")


def main(args=None):

    rclpy.init(args=args)

    node = SignDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()