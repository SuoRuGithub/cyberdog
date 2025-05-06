清华大学第四届机器狗大赛第15组 进度记录与备忘

写在前面

本文档引自 @清华大学第三届机器狗大赛第7组 进度记录与备忘

This README file records some of our commonly used command lines, as well as logs and parts of our thoughts during debugging.

信息
配置
狗
 15-1
 IP: 10.0.0.24
 Password: 123
 Namespace: dog
 15-2
 IP: 10.0.0.192
 Password: 123
 Namespace:cyberdog 
 TightVNC:
 Remote Host: 10.0.0.123:5901
 Password: 123321

参数信息

x 直行（向前为正方向），y 平移（向左为正方向），z 旋转（逆时针为正方向）

提醒

 同时写代码时注意保存
 Build之前要先在终端输入 `cd workplace`, 否则无法更新所做的修改!

参考资料

二代机器狗的开源信息：

1. [文档博客](https://miroboticslab.github.io/blogs/#/)
2. [源码地址](https://github.com/MiRoboticsLab/cyberdog_ws)

ROS学习参考

1. [发布订阅节点](https://blog.csdn.net/qq_38649880/article/details/104423203)

其它资料

1[第二次培训的PPT和录屏](https://cloud.tsinghua.edu.cn/d/9aefef66ac9542a6944d/)
2. [代码托管](https://git.tsinghua.edu.cn/cyberdog_competition/2024)
3. [whf的github仓库](https://github.com/HeFeiW/cyberdog_az)

进度

进度记录

04.29 zhz cyf zwk zbs zz

 完成了stand.py,ultrasonic & step的代码和复现并耗费了大量时间调试（虽然可能用处并不很大？）
 补上一条：实际上对于中期验收可以ultrasonic测距来控制停下的位置，不过精度还得再调一下。

 尝试理解ros2的工作机制: subscribe & publish
 皓哲学长成功打开了摄像头并完成了拍摄和停止控制任务
05.07: bs hz wk yf zz  
打开RGB摄像机：
窗口1:ros2 launch camera_test stereo_camera.py
窗口2:# 启动lifecycle的接口
ros2 lifecycle set /dog/camera/camera configure 
ros2 lifecycle set /dog/camera/camera activate (不清楚这两条的作用，跑了为主）
ros2 lifecycle set /stereo_camera configure 
ros2 lifecycle set /stereo_camera activate

workplace/src/learning/learning/greenball_tracker.py:
实时获取rgb相机的图片，并标出绿色球的bounding box，可以获得面积大小

明日目标：利用红外相机实现避障功能

