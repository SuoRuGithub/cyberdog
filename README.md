清华大学第四届机器狗大赛第15组 进度记录与信息共享

信息
代码共享:https://github.com/SuoRuGithub/cyberdog


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


ROS学习参考

1. [发布订阅节点](https://blog.csdn.net/qq_38649880/article/details/104423203)


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
05.06: bs hz wk yf zz  
打开RGB摄像机：
窗口1:ros2 launch camera_test stereo_camera.py
窗口2:# 启动lifecycle的接口
ros2 lifecycle set /dog/camera/camera configure 
ros2 lifecycle set /dog/camera/camera activate (不清楚这两条的作用，跑了为主）
ros2 lifecycle set /stereo_camera configure 
ros2 lifecycle set /stereo_camera activate
workplace/src/learning/learning/camera.py：
调用rgb相机，获得图片

workplace/src/learning/learning/greenball_tracker.py:
将camera.py和opencv结合，实时获取rgb相机的图片，并标出绿色球的bounding box，publish面积大小

明日目标：利用红外相机实现避障功能

05.07下午：cyf
troubles:
1.狗子1连不上远程桌面tightvnc了：记得连实验室的cyberdog网！！！警钟长鸣！！！
2.尝试复现相机代码，好像出现了问题，发给了学长晚上看下解决s:
3.发现好像tightvnc与命令行中并不兼容？需要重新colcon build,source，总之多build几遍总没错……
4.Traceback (most recent call last):
  File "/home/mi/workplace/install/learning/lib/learning/greenball_stop", line 33, in <module>
    sys.exit(load_entry_point('learning==0.0.0', 'console_scripts', 'greenball_stop')())
  File "/home/mi/workplace/install/learning/lib/learning/greenball_stop", line 25, in importlib_load_entry_point
    return next(matches).load()
StopIteration的问题是setup.py中没加entry——point中，注意区分setup.bash!!!
5.开始尝试整合，但是视觉部分相机一直没连上（或者连上了）？不知道为什么呜呜呜
6.连上了，需要保证四条语句都成功，相机能够正常持续不断输出
7.注意我setup里写的是
8.'ball_tracker = learning.greenball_tracker:main',没有green
9.在ball——tracker中，为什么狗子不走呢？试了改
10.AREA_THRESHOLD没有用……
11.
achievements：
1.超声避障成功实现，写在ultrasonic_barrier.py中,下一步在midterm.py中第65行左右的if else中添加“见到绿色球停下”的逻辑判断即可
2.桌子上放了绿色小方块，晚上做视觉的时候方便用一下
3.哭了呀现在是五点37，摄像头正常打开识别了。开始进一步整合
4.现在是六点半，现在midterm程序不报错了，不过还没能像ball——tracker一样识别到球，晚上再继续。
5.7晚上：zhz zbs cyf
成功完成了整合部分，完整操作流程如下：
1.连接tightvnc
2.运行三件套，正确激活相机
3.在tightvnc命令行运行ball_tracker,注意先source
4.在vscode中运行avoid，同样要source

workplace/src/learning/learning/avoid.py:
subcribe狗子的红外相机和greenball_tracker.py发布的绿球面积

注意:
①publish任何messgae的步骤：
1.先设置timer
self.timer=self.create_timer(频率,self.timer_callback,数据量)

2.设置publish函数：
self.pub = self.create_publisher(DATA_TYPE,"topic name"，数据量)

3.在timer_callback函数中：
def timer_callback(self):
    msg = DATA_TYPE()
    msg.data = self.data
    ......
    （设置msg中的其他数据）
    ......
    self.pub.publish(msg)

②subcribe某一个话题的步骤：
1.设置subscribe函数
self.sub = self.create_subscription(DATA_TYPE,"topic name",self.sub_callback,数据量)

2.在sub_callback函数中：
def sub_callback(self,msg:DATA_TYPE):
    data = msg.data#即可订阅topic中传输的数据

③保存数据：
最好将数据都存储在“类的元素”中

所有代码均上传github，链接放在本文档开头了，大家辛苦了！！！
5.8晚上：cyf
按照第二次培训内容完成了两只狗的动捕捉配置，应该后续可以直接用了

