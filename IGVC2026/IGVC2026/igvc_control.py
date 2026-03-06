import rclpy
import time 
from rclpy.node import Node
from std_msgs.msg import String

class IgvcControl(Node):
    def __init__(self):
        super().__init__('igvc_control')

        self.subscription = self.create_subscription(
            String,
            'now_status',
            self.status_callback,
            10
        )

        self.stop = True
        self.previous_status = None

    def status_callback(self, msg):
        if msg.data == "Stop" and self.previous_status != "Stop":
            self.get_logger().info("something_detected")
            self.stop = True
            self.send_action_request()

        if msg.data == "Go" and self.previous_status == "Stop":
            self.get_logger().info("Not detected")
            self.stop = False
            self.send_action_request()

        self.previous_status = msg.data


    def delayed_action_send(self):
        self.send_action_request()
    
    # サーバーにアクションを送信する関数
    def send_action_request(self):
        goal_msg = StopFlag.Goal()
        
        # stop変数の状態でaの値を決定
        if self.stop: # True
            goal_msg.a = 1  # stop
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

def main(args=None):
    rclpy.init(args=args)
    igvc_control = IgvcControl()
    rclpy.spin(igvc_control)
    igvc_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()