### 2. 地图转换（PCD to 二维栅格地图）



[![img](https://github.com/66Lau/NEXTE_Sentry_Nav/raw/main/doc/sentry_navigation.png)](https://github.com/66Lau/NEXTE_Sentry_Nav/blob/main/doc/sentry_navigation.png)

这里面红色的点云就是PCD文件显示的三维点云地图，白色的就是熟知的二维栅格地图

地图转换主要是因为move_base是基于2d的栅格地图进行路径规划，而fast_lio默认的输出地图是三维点云的PCD文件，我们需要用一些方法获取2d的栅格地图，有以下几种方式：

1. 用fast_lio构建好PCD地图后，将PCD地图转换为栅格地图
   方式一：使用[pcd_package](https://github.com/Hinson-A/pcd2pgm_package)开源功能包，参考[离线将PCD地图转换为pgm栅格地图](https://blog.csdn.net/Draonly/article/details/124537069?ops_request_misc=%7B%22request%5Fid%22%3A%22165207936116781435426048%22%2C%22scm%22%3A%2220140713.130102334.pc%5Fall.%22%7D&request_id=165207936116781435426048&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-6-124537069-null-null.142^v9^control,157^v4^control&utm_term=pcd地图转换为栅格地图&spm=1018.2226.3001.4187)
   方式二：使用`octomap_server`功能包,离线将pcd转换成栅格地图，参考[octomap_server使用－－生成二维占据栅格地图和三维概率地图](https://blog.csdn.net/sru_alo/article/details/85083030?ops_request_misc=%7B%22request%5Fid%22%3A%22169804282616800213031883%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=169804282616800213031883&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-85083030-null-null.142^v96^pc_search_result_base9&utm_term=点云地图生成栅格地图&spm=1018.2226.3001.4187)
2. 在fast_lio构建三维点云地图的同时，也实时构建2d的栅格地图

本文的代码仓库里两种方式都有，常用的是第二种，如果你是打比赛之类的用的话，建议你建图的时候可以使用rosbag录制一下你的雷达的话题（imu+pcd）,这样就算建图效果一般，也能离线重新rosbag play进行建图

## **1. Octomap_server 简介**

`octomap_server` 是 ROS 中一个功能包，基于 **Octomap** 框架，主要用于处理点云数据，将其转化为基于八叉树的地图表示 (**Octree-based OctoMap**)。这个功能包既可以生成三维概率地图，也支持生成二维占据栅格地图。

------

### **2. Octomap_server 的安装**

以下是安装命令（以 ROS Kinetic 为例）：

```bash
sudo apt-get install ros-kinetic-octomap-ros
sudo apt-get install ros-kinetic-octomap-msgs
sudo apt-get install ros-kinetic-octomap-server
sudo apt-get install ros-kinetic-octomap-rviz-plugins
```

安装完成后，可以在 RViz 中添加 `octomap_rviz_plugins` 模组，用于可视化地图数据。

------

### **3. 启动 Octomap_server**

#### 3.1 启动方式

通过 launch 文件启动 `octomap_server` 节点。例如：

```xml
<launch>
  <node name="octomap_server" pkg="octomap_server" type="octomap_server_node" output="screen">
    <param name="frame_id" value="world"/>
    <param name="resolution" value="0.05"/>
    <remap from="/cloud_in" to="/your_pointcloud_topic"/>
  </node>
</launch>
```

**关键点：**

- `frame_id` 要和 RViz 的 `Fixed Frame` 保持一致。
- `/cloud_in` 需要映射为实际的 `PointCloud2` 数据话题。

#### 3.2 相关话题

运行后可产生以下话题：

- `/octomap_binary`
- `/octomap_full`
- `/occupied_cells_vis_array`

------

### **4. 可视化数据**

在 RViz 中：

- 添加 **OccupancyGrid** 或 **OccupancyMap**。
- 对应话题设置为 `/octomap_full` 或 `/octomap_binary`。

可视化效果如下：

- **Map:** 显示概率地图。
- **OccupancyMap:** 显示二维占据栅格地图。
- **OccupancyGrid:** 显示三维点云数据。

------

### **5. 扩展功能**

#### 5.1 三维到二维地图

`octomap_server` 除了生成三维概率地图，还可以转换为二维占据栅格地图，用于导航等场景。

#### 5.2 参数调整

可根据需求调整 `frame_id`、`resolution` 等参数以适配不同应用场景。

------

### **6. 学习与代码资源**

- 北达科他大学的学习资源：
   [cloud_to_map 学习代码](https://download.csdn.net/download/sru_alo/12277545)。

更多信息可参考 [Octomap_server 的 ROS Wiki](http://wiki.ros.org/octomap_server)。

如果你有其他具体问题，欢迎进一步探讨！