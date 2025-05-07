#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.dogname = "dog"
        self.subscription = self.create_subscription(
            Image,
            '/image_rgb',
            self.listener_callback,
            10)
        self.publiction = self.create_publisher(Float64 ,f"/{self.dogname}/max_area_pub",10)

        self.timer = self.create_timer(0.05,self.timer_callback)
        self.current_frame = None
        self.max_area = 0.0
        
        # 初始化滑动条窗口
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 35, 179, lambda x: None)
        cv2.createTrackbar("L - S", "Trackbars", 50, 255, lambda x: None)
        cv2.createTrackbar("L - V", "Trackbars", 50, 255, lambda x: None)
        cv2.createTrackbar("U - H", "Trackbars", 85, 179, lambda x: None)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, lambda x: None)

    def listener_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

        frame = self.current_frame.copy()
        
        # 转换到HSV并获取掩膜
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 检测轮廓
        image,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0.0
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            max_area = cv2.contourArea(max_contour)
            # 绘制轮廓和中心
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {max_area}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self.max_area = max_area
        self.get_logger().info(f"GreenBall area: {self.max_area:.5f} meters")
        # 显示图像
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        return 

    def timer_callback(self):#发布小狗rgb相机中绿色球的面积
        area_msg =  Float64()
        area_msg.data = self.max_area
        self.publiction.publish(area_msg)
        return

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()