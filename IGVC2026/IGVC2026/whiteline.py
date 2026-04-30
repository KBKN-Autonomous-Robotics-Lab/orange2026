import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class WhiteLineDetection(Node):
    def __init__(self):
        super().__init__('whiteline_detection')

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


        main_loop = self.create_timer(1.0/30.0, self.timer_callback)


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # =============== yellow line ===================
        # ノイズ除去
        yellow_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # yellowの抽出
        lower_yellow = np.array([0, 100, 100])
        upper_yellow = np.array([80, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #lower_yellow = np.array([0, 100, 100])
        #upper_yellow = np.array([80, 255, 255])
        #yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5,5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        # エッジ検出 
        yellow_edges = cv2.Canny(yellow_mask, 50, 150) 
        h, w = yellow_edges.shape 
        mask_yellow = np.zeros_like(yellow_edges) 
        cv2.rectangle(mask_yellow, (0, int(h*0.6)), (w, h), 255, -1) 
        yellow_edges = cv2.bitwise_and(yellow_edges, mask_yellow)

        yellow_lines = cv2.HoughLinesP(
            yellow_edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=30
        )

        # =============== white line ==================
        
        # グレースケール
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # ノイズ除去
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # エッジ検出 
        edges = cv2.Canny(blur, 50, 150) 
        h, w = edges.shape 
        mask_white = np.zeros_like(edges) 
        cv2.rectangle(mask_white, (0, int(h*0.6)), (w, h), 255, -1) 
        edges = cv2.bitwise_and(edges, mask_white)
        #edges = cv2.bitwise_and(edges, cv2.bitwise_not(yellow_mask))
        white_contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in white_contours:
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (bw, bh), angle = rect

            if cy < h /2:
                continue
            
            if max(bw, bh) < 50:
                continue

            cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 5)

        """
        
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        # 白色の抽出
        lower_white = np.array([100, 100, 100])
        upper_white = np.array([255, 255, 255])
        white_mask = cv2.inRange(blur, lower_white, upper_white)
        edges = cv2.Canny(white_mask, 50, 150)
        h, w = edges.shape 
        mask_white = np.zeros_like(edges) 
        cv2.rectangle(mask_white, (0, int(h*0.6)), (w, h), 255, -1) 
        edges = cv2.bitwise_and(edges, mask_white)
        """
        
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=30
        )

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                num_samples = 20
                is_yellow = False
                for t in np.linspace(0, 1, num_samples):
                    x = int(x1 + t*(x2 - x1))
                    y = int(y1 + t*(y2 - y1))

                    if yellow_mask[y, x] > 0:
                        is_yellow = True
                        break

                if is_yellow:
                    continue
                    
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        if yellow_lines is not None:
            for yellow_line in yellow_lines:
                x1, y1, x2, y2 = yellow_line[0]
                cv2.line(frame, (x1+3, y1-3), (x2+3, y2+3), (0, 0, 255), 40)

        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_h, img_w = frame.shape[:2]
        for cnt in yellow_contours:
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect
            
            if cy < img_h / 2:
                continue

            cv2.drawContours(frame, [cnt], 0, (0, 0, 255), cv2.FILLED)
 
    
             

        cv2.imshow("Detected Lane Lines", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = WhiteLineDetection()
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