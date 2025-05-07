#!/usr/bin/env python3
# coding: utf-8
import cv2
import numpy as np

# 模拟机器狗运动控制
class RobotDog:
    def __init__(self):
        self.is_moving = False
    
    def stop(self):
        self.is_moving = False
        print("STOP: 检测到绿色球靠近，停止运动！")
    
    def move_forward(self):
        self.is_moving = True
        print("Running前进中...")

def main():
    # 初始化摄像头和机器狗
    dog = RobotDog()
    dog.move_forward()  # 初始状态前进
    
    # 创建HSV滑动条窗口
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L - H", "Trackbars", 35, 179, lambda x: None)  # 绿色默认值
    cv2.createTrackbar("L - S", "Trackbars", 50, 255, lambda x: None)
    cv2.createTrackbar("L - V", "Trackbars", 50, 255, lambda x: None)
    cv2.createTrackbar("U - H", "Trackbars", 85, 179, lambda x: None)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, lambda x: None)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, lambda x: None)

    # 停止阈值（根据实际测试调整）
    AREA_THRESHOLD = 2000  # 示例值，需根据摄像头高度、球大小调整

    while True:
        frame = cv2.imread("/home/mi/workplace/latest_image.jpg")
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
        max_area = 0
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            max_area = cv2.contourArea(max_contour)
            # 绘制轮廓和中心
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {max_area}", (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 根据面积控制运动
        if max_area > AREA_THRESHOLD:
            dog.stop()
        else:
            if not dog.is_moving:
                dog.move_forward()

        # 显示图像
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        # 按ESC退出
        if cv2.waitKey(30) == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
