import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
import time

AREA_THRESHOLD = 15000
SAFE_DISTANCE = 0.25
MAX_DISTANCE = 100000
MIN_GREENBALL_AREA=0.0

class basic_move(Node):
    def __init__(self,name):
        super().__init__(name)
                # 参数声明
        self.declare_parameters(namespace='',
                               parameters=[
                                   ('dog_name', 'dog'),
                                   ('forward_speed', 0.3),
                                   ('turn_speed', 0.5),
                               ])

        # 获取参数
        self.dog_name = self.get_parameter('dog_name').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.speed_x, self.speed_y, self.speed_z = 0.0, 0.0, 0.0
        self.motion_id = 303
        self.greenball_area = MIN_GREENBALL_AREA
        self.distance_to_greenball = MAX_DISTANCE

        self.pub = self.create_publisher(MotionServoCmd, f'{self.dog_name}/motion_servo_cmd',10)
        self.subscribe_greenball_area = self.create_subscription(Float64,f"/{self.dog_name}/max_area_pub",self.rgb_callback,10)#订阅视觉中绿色球的面积
        self.subscribe_distance_message = self.create_subscription(Range,f'/{self.dog_name}/ultrasonic_payload',self.sensor_callback,10)
        self.timer=self.create_timer(0.01, self.timer_callback)


    def change_speed(self, speed_x, speed_y, speed_z):
        self.speed_x = speed_x
        self.speed_y = speed_y
        self.speed_z = speed_z
        self.get_logger().info(f"Speed changed to: {self.speed_x}, {self.speed_y}, {self.speed_z}")

    def change_motion_id(self, motion_id):
        self.motion_id = motion_id
        self.get_logger().info(f"Motion ID changed to: {self.motion_id}")

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
        self.motion_id = 111
        rclpy.shutdown() 

    def rgb_callback(self,msg:Float64):
        self.greenball_area = msg.data
        # self.get_logger().info(f"GreenBall area in rgb camera: {self.greenball_area:.5f} meters")
    
    def sensor_callback(self,msg:Range):
        self.distance_to_greenball = msg.range
        # self.get_logger().info(f"Distance to obstacle: {self.distance_to_greenball:.5f} meters")

    def timer_callback(self):

        if self.greenball_area > AREA_THRESHOLD:
            self.get_logger().info(f"Green ball detected (Area: {self.greenball_area:.5f}), stopping!")
            self.stop()
        elif self.distance_to_greenball < SAFE_DISTANCE:
            self.get_logger().info(f"distance:{self.distance_to_greenball}, Obstacle detected! Turning...")
            self.execute_turn()
        else:
            self.get_logger().info(f"distance:{self.distance_to_greenball}, moving forward...")
            self.execute_forward()

        msg = MotionServoCmd()
        msg.motion_id = self.motion_id
        msg.cmd_type=1
        msg.value = 2
        msg.vel_des=[self.speed_x, self.speed_y, self.speed_z]
        msg.step_height=[0.05, 0.05]
        self.pub.publish(msg)
        # self.get_logger().info(f"Publishing: {msg}")

def main(args=None):
    rclpy.init(args=args)

    move_node = basic_move('avoid_greenball')

    move_node.change_speed(1.0, 0.0, 0.0)
    rclpy.spin(move_node)
    move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
