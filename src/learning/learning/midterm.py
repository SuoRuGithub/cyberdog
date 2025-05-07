#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
from sensor_msgs.msg import Range
import cv2
import numpy as np


class DogAvoidanceController(Node):
    def __init__(self, name):
        super().__init__(name)
        # 参数声明
        self.declare_parameters(namespace='',
                               parameters=[
                                   ('dog_name', 'dog'),
                                   ('safe_distance', 0.5),
                                   ('forward_speed', 0.3),
                                   ('turn_speed', 0.5),
                                   ('green_area_threshold', 30000),  # 新增：绿色球停止阈值
                                   ('image_path', '/home/mi/workplace/latest_image.jpg')  # 图像路径
                               ])

        # 获取参数
        self.dog_name = self.get_parameter('dog_name').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.green_area_threshold = self.get_parameter('green_area_threshold').value
        self.image_path = self.get_parameter('image_path').value

        # 运动控制变量
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.distance = float('inf')

        # 视觉检测参数 (绿色HSV默认值)
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])

        # ROS2 发布/订阅
        self.pub = self.create_publisher(MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10)
        self.sub = self.create_subscription(
            Range, f"/{self.dog_name}/ultrasonic_payload", self.sensor_callback, 10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

        

    def detect_green_ball(self):
        """检测绿色球并返回最大轮廓面积"""
        try:
            frame = cv2.imread(self.image_path)
            if frame is None:
                self.get_logger().warn(f"无法读取图像文件: {self.image_path}")
                return 0
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # 兼容所有OpenCV版本的写法
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 2:  # OpenCV 4.x
                contours = contours[0]
            elif len(contours) == 3:  # OpenCV 3.x
                contours = contours[1]
            else:
                return 0
            
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                self.get_logger().debug(f"绿色区域面积: {area:.2f}")
                return area
                
        except Exception as e:
            self.get_logger().error(f"视觉检测错误: {str(e)}")
        return 0

    def execute_forward(self):
        self.speed_x = self.forward_speed
        self.speed_y = 0.0
        self.speed_z = 0.0

    def execute_turn(self):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = self.turn_speed

    def stop(self):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

    def sensor_callback(self, msg: Range):
        self.distance = msg.range
        self.get_logger().info(f"Distance to obstacle: {self.distance:.2f} meters")

    def timer_callback(self):
        # 检测绿色球
        green_area = self.detect_green_ball()
        
        # 优先处理绿色球检测
        if green_area > self.green_area_threshold:
            self.get_logger().info(f"Green ball detected (Area: {green_area:.0f}), stopping!")
            self.stop()
        # 其次处理超声避障
        elif self.distance < self.safe_distance:
            self.get_logger().info("Obstacle detected! Turning...")
            self.execute_turn()
        else:
            self.get_logger().info("Path clear, moving forward...")
            self.execute_forward()

        # 发布运动指令
        msg = MotionServoCmd()
        msg.motion_id = 303
        msg.cmd_type = 1
        msg.value = 2
        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        msg.step_height = [0.05, 0.05]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DogAvoidanceController("dog_avoidance_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()