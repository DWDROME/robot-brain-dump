# **手持树莓派建图**

环境是：

- SLAMTEC A1M8  、 ubuntu 22.04lts 、 ros2 *Iron*
- xubuntu-desktop 、 lightdm 、 X11
- VNC 、 SSH

一共有两个功能包：

- #### **雷达驱动包**==a1m8_cartographer==

- #### 雷达驱动包==**sllidar_ros2**==

  - sllidar_a1_launch.py
  - view_sllidar_a1_launch.py



## 雷达

雷达包来源于：[sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)

在插入雷达之后，在终端中记得先

```bash
sudo chmod 777 /dev/ttyUSB0
```

或者使用附赠的sh文件永久添加(里面的路径需要手动修改)

```bash
cd src/rpldiar_ros/
source scripts/create_udev_rules.sh
```

## 建图

- 建图代码来源于[Cartographer手持雷达SLAM建图配置指南 ](https://blog.csdn.net/qq_27865227/article/details/127000582)

- 直接/map->/scan的方案[Cartographer 我也试一把只用雷达建图](https://fishros.org.cn/forum/topic/720/cartographer-%E6%88%91%E4%B9%9F%E8%AF%95%E4%B8%80%E6%8A%8A%E5%8F%AA%E7%94%A8%E9%9B%B7%E8%BE%BE%E5%BB%BA%E5%9B%BE)



其中，如果在树莓派上运行rviz的话，会发生报错，并且可预见的会非常卡顿。

```bash
dwmini@dwmini-mini:~$ rviz2
qt.qpa.xcb: could not connect to display 
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms/libqxcb.so" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.

Aborted (core dumped)
```

我尝试过指定特定的display（网络方案），但是因为无屏幕（vnc无须屏幕），所以我放弃了继续更改。

(似乎是因为vnc无法支持openGL？而导致缺少qt库？)

 > [!IMPORTANT]
 >
 > 我希望有人能够帮助我解决这个问题

## 笔记本与树莓派共同工作

于是，我决定在树莓派上运行雷达驱动和建图，而在我的笔记本上运行rviz和最后的保存建图。

我是用手机热点确保它们在同一个网段下面。



1. 首先我们需要两个设备的ip地址。

```bash
ip addr show	#本机详细地址
hostname -I		#本机局域网地址（inet）
```

```bash
nmap -sn 192.168.0.0/24	#搜索同一网段下所有设备
```

当然更方便的是在手机中查看连接设备ip地址。



2. 参考下列文章完成配置。

- [ROS远程通信配置，rviz远程显示及命令控制](https://blog.csdn.net/liu3612162/article/details/115231604)



3. 在树莓派的功能包中，禁用rviz部分。



4. 接着树莓派成功运行后，在笔记本运行原来的rviz文件就可以启动了。

```bash
rviz -d $path_to_your_rviz_file
```



5. 最后保存图片
```bash
ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
```



## 附记： 如何更加平稳地建图？

我是在宿舍里建图的。

蹲着，将雷达举到头顶，慢慢滑行。

切忌雷达旋转和加速移动。

保持直线



或者，用手直直地架住雷达。 



## 现象

![odom建图](/home/dw/note/树莓派配置/cartographer建图.assets/截图 2025-01-21 13-01-42.png)

![rviz](/home/dw/note/树莓派配置/cartographer建图.assets/截图 2025-01-21 12-22-39.png)