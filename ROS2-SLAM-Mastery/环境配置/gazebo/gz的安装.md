



# 安装



资料来源:



- [Gazebo Sim 开源项目快速指南](https://blog.csdn.net/gitblog_00323/article/details/141211006)
- [gazebosim.org](https://gazebosim.org/api/sim/7/tutorials.html)

==注意安装版本==

在Ubuntu24.04lts中，ros2-jazzy中。我们需要安装的是gazebo-**Harmonic**



==请务必注意版本的选择==

我因为选错了版本导致非常多的错误，由此耽搁了一个星期。

因为全英文的环境，直到上周我才意识到版本可能出错了。

##  安装gazebo

### ** 添加 Gazebo 软件源**

运行以下命令添加 Gazebo 官方源：

```bash
sudo apt update && sudo apt install -y wget gnupg
wget -O - https://packages.osrfoundation.org/gazebo.key | sudo tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

------

### **更新包列表**

```bash
sudo apt update
```

------

### **安装 Gazebo**

安装最新版本的 Gazebo Sim（如 `gazebo11` 或 `gz-sim`）：

```bash
sudo apt install -y gz-sim
```

## 安装ros_gz



**与 ROS 2 的集成：**

在新的架构中，Gazebo Sim 与 ROS 2 的集成通过 `ros_gz`（以前称为 `ros_ign`）桥接包实现。该桥接包允许 ROS 2 和 Gazebo Sim 之间的消息传递。

**安装 `ros_gz`：**

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```



# 初步使用

在新版的 Gazebo Sim中，启动命令和插件加载方式已发生变化。之前使用的命令 `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so` 已被新的命令和插件机制取代。





这是gazebo的启动命令。



```bash
gz sim -v 4
```

- **`-v`**: 指定日志详细级别，`4` 表示最高详细程度。
- 插件的加载方式已被更改，需要在模型文件（SDF/URDF）或 Gazebo 的配置中添加。

接着你会在一个GUI中选择要加载的**SDF模型**（gazebo自带）



或者你可以直接用这个来启动一个空模板

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

------

## **新版插件配置方式**

1. **模型加载方式** 在 SDF 文件中直接定义插件。例如：

   ```xml
   <plugin
       name="ros2_control"
       filename="libgazebo_ros2_control.so">
       <!-- 插件参数 -->
   </plugin>
   ```

2. **ROS 2 集成** 使用 `ros_gz_bridge` 代替 `libgazebo_ros_init.so` 和 `libgazebo_ros_factory.so` 的功能：

   ```bash
   ros2 run ros_gz_bridge parameter_bridge /example_topic@std_msgs/msg/String@gz.msgs.StringMsg
   ```

3. **启动文件方式** 使用 ROS 2 的 launch 文件启动 Gazebo 和桥接插件。例如：

   ```bash
   ros2 launch ros_gz_sim gz_sim.launch.py
   ```

   

   ==不懂？==在下一章。我会补充关于链接和启动相关的东西。叫【通过ros2启动gazebo】

------



