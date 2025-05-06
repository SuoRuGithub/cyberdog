# ###########################
# avoid_controller.py
# Created by ZZ, 2025.05.06
# ###########################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from protocol.msg import MotionServoCmd
import numpy as np

# 自动避障控制
class AvoidanceController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("dog_name", "dog")
        self.declare_parameter("safe_distance", 0.5)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("turn_speed", 0.5)

        self.dog_name = self.get_parameter('dog_name').get_parameter_value().string_value
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.obstacle_distance = float('inf')
        self.obstacle_position = None
        self.image_width       = None

        self.stage = "detect"  # 时间分段：detect -> plan -> avoid -> recover
        self.stage_timer = 0.0
        self.stage_duration = 2.0  # 每个阶段持续时间（秒）
        self.turn_direction = "left"  # 默认转向方向

        self.bridge = CvBridge()
        self.pub = self.create_publisher(MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10)
        self.depth_sub = self.create_subscription(Image, f"/{self.dog_name}/depth_image", self.depth_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.kp = 0.5  # PID参数
        self.previous_error = 0.0

    def depth_callback(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        height, width = depth_image.shape
        self.image_width = width

        # 计算前方区域的最小深度
        center_x, center_y = width // 2, height // 2
        region_size = min(width, height) // 4
        region = depth_image[center_y - region_size:center_y + region_size,
                           center_x - region_size:center_x + region_size]
        self.obstacle_distance = np.nanmin(region) if np.any(region) else float('inf')

        # 确定障碍物位置（大致估计在图像中的x坐标）
        if self.obstacle_distance < self.safe_distance:
            left_half = depth_image[center_y - region_size:center_y + region_size,
                                  center_x - region_size:center_x]
            right_half = depth_image[center_y - region_size:center_y + region_size,
                                   center_x:center_x + region_size]
            left_dist = np.nanmin(left_half) if np.any(left_half) else float('inf')
            right_dist = np.nanmin(right_half) if np.any(right_half) else float('inf')
            self.obstacle_position = center_x - region_size if left_dist < right_dist else center_x + region_size
        else:
            self.obstacle_position = None

        self.get_logger().info(f"Obstacle distance: {self.obstacle_distance:.2f} meters, position: {self.obstacle_position}")

    def execute_forward(self):
        self.speed_x = self.forward_speed
        self.speed_y = 0.0
        self.speed_z = 0.0

    def execute_turn(self, direction="left"):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = self.turn_speed if direction == "left" else -self.turn_speed

    def stop(self):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

    def timer_callback(self):
        msg = MotionServoCmd()
        msg.motion_id = 303
        msg.cmd_type = 1
        msg.value = 2
        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        msg.step_height = [0.05, 0.05]

        # 静态分段：时间上的阶段切换
        self.stage_timer += 0.1

        if self.stage == "detect":
            # 阶段1：检测障碍物
            if self.obstacle_distance < self.safe_distance and self.obstacle_position:
                # detect => plan
                self.get_logger().info("Obstacle detected, moving to planning stage...")
                self.stage = "plan"
                self.stage_timer = 0.0
            else:
                # detect => detect
                self.get_logger().info("No obstacle, moving forward...")
                self.execute_forward()

        elif self.stage == "plan":
            # 阶段2：物理建模与路径规划
            if self.stage_timer < self.stage_duration:
                # plan => plan
                center_x = self.image_width / 2
                error = (self.obstacle_position - center_x) / center_x
                self.turn_direction = "right" if error > 0 else "left"  # 障碍物在右边则左转，反之右转
                self.get_logger().info(f"Planning: Turn {self.turn_direction}, error: {error:.2f}")
                self.stop()  # 规划时暂停
            else:
                # plan => avoid
                self.stage = "avoid"
                self.stage_timer = 0.0

        elif self.stage == "avoid":
            # 阶段3：动态决策与执行避障
            if self.stage_timer < self.stage_duration:
                # avoid => avoid
                self.execute_turn(direction=self.turn_direction)
                self.get_logger().info(f"Avoiding: Turning {self.turn_direction}...")
            else:
                # avoid => recover
                self.stage = "recover"
                self.stage_timer = 0.0

        elif self.stage == "recover":
            # 阶段4：恢复前进
            if self.stage_timer < self.stage_duration:
                # recover => recover
                self.execute_forward()
                self.get_logger().info("Recovering: Moving forward...")
            else:
                # recover => detect
                self.stage = "detect"
                self.stage_timer = 0.0

        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceController("obstacle_avoidance_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()