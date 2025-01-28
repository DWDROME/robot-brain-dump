- [ros2_integration](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [3.使用gazebo加载URDF](https://fishros.com/d2lros2/#/humble/chapt9/get_started/3.%E5%9C%A8Gazebo%E5%8A%A0%E8%BD%BD%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%A8%A1%E5%9E%8B)





在老版本中，我们使用

```bash
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

来启动gazebo和ros2与gazebo的桥。



但在新版本中，`libazebo_ros_init.so`和`libazebo_ros_factory.so`不再被支持

==你当然可以通过链接的方式，强行建立联系==

# ros2启动gazebo

ros2内置一个ros_gz_sim功能包，用于启动gz.

**1. 带 GUI 的仿真模式**

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```



---

**2. 无 GUI 的服务器模式**

```bash
ros2 launch ros_gz_sim gz_server.launch.py world_sdf_file:=empty.sdf
```

| **功能**     | **带 GUI 模式** | **服务器模式**   |
| ------------ | --------------- | ---------------- |
| 是否加载 GUI | 是              | 否               |
| 文件加载参数 | `gz_args`       | `world_sdf_file` |
| 资源消耗     | 高              | 低               |

==我们使用第一个,相当于使用了`gz sim`

---

#  建立桥梁

## 手动桥接



```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```
意思是：
- **`/scan`** 是话题名。
- **`sensor_msgs/msg/LaserScan`** 是 ROS 2 的消息类型。
- **`gz.msgs.LaserScan`** 是 Gazebo 的消息类型。
- 这将会桥接 Gazebo 的 `/scan` 和 ROS 2 的 `/scan`，实现激光扫描数据的通信。

---

## 配置桥接



- 你需要提供一个 YAML 配置文件（比如 `bridge_config.yaml`），里面列出所有需要桥接的话题。
- 比如一个 YAML 文件可能这样写：
  ```yaml
  - ros_topic_name: "scan"
    gz_topic_name: "/scan"
    ros_type_name: "sensor_msgs/msg/LaserScan"
    gz_type_name: "gz.msgs.LaserScan"
    direction: GZ_TO_ROS  # BIDIRECTIONAL or ROS_TO_GZ
  ```
  这就定义了 **一次性桥接多个话题** 的规则。

## **总结：两种桥接方式的区别**

| **桥接方式**                           | **特点**                                 |
| -------------------------------------- | ---------------------------------------- |
| **`parameter_bridge`（手动桥接）**     | 手动运行单个桥接，直接指定话题和消息类型 |
| **`ros_gz_sim.launch.py`（自动桥接）** | 用配置文件一次性桥接多个话题，启动更方便 |

---

- **手动桥接**就像点菜，你每次都要单独点一份，比如“来一份激光扫描数据”。
- 自动桥接就像自助餐，你提前设置好需要的所有话题，然后一启动就“全包了”。



------

# launch文件

有两种实现方式，`xml`和`.launch.py`

e，我决定使用python的方式，因为之前一直都是这么用的，即使xml要方便很多。



这是我自己写的桥接launch

**`RosGzBridge`**



```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='src/my_bridge/config/scan_bridge_config.yaml',
            description='Path to the scan bridge configuration file.'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scan_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        )
    ])
```



这是官方的节点代码(通用)

通用 `Node`



```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)

    return ld
```



| **方式**          | **适用场景**                                                 | **优缺点**         |
| ----------------- | ------------------------------------------------------------ | ------------------ |
| **`RosGzBridge`** | 仅需要桥接功能的场景，专注于 `ros_gz_bridge`。               | 简洁、专用。       |
| **通用 `Node`**   | 需要运行其他 ROS 2 节点，或者对节点参数有更多定制化需求的场景。（传感器处理、控制算法） | 灵活，可扩展性强。 |

------

# 实现模板



接下来说说一个稍微完整的ros-gz包的基本结构是怎么样的。

## **目录结构**

```plaintext
ros2_ws/
├── src/
│   ├── my_bridge/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── bridge.launch.py
│   │   │   ├── scan_bridge.launch.py
│   │   │   └── cmd_vel_bridge.launch.py
│   │   ├── config/
│   │   │   ├── bridge_config.yaml
│   │   │   ├── scan_bridge_config.yaml
│   │   │   └── cmd_vel_bridge_config.yaml
│   │   ├── scripts/
│   │   │   ├── custom_node.py
│   │   │   ├── scan_processor.py
│   │   │   └── cmd_vel_processor.py
```

------

## **各文件功能说明**

### **1. `package.xml` 和 `CMakeLists.txt`**

- 负责声明依赖（如 `ros_gz_bridge`）并设置包的构建规则。
- 不用做特殊改动，只需保证包的名字和依赖声明正确。

### **2. `launch` 文件夹**

- **`bridge.launch.py`**：主桥接入口，加载通用的 `bridge_config.yaml`。
- **`scan_bridge.launch.py`**：专门用于激光雷达的桥接，加载 `scan_bridge_config.yaml`。
- **`cmd_vel_bridge.launch.py`**：专门用于机器人速度控制的桥接，加载 `cmd_vel_bridge_config.yaml`。

### **3. `config` 文件夹**

- **`bridge_config.yaml`**：包含所有需要桥接的 ROS 2 和 Gazebo 话题规则，适合综合桥接。
- **`scan_bridge_config.yaml`**：仅桥接激光雷达数据，便于独立调试。
- **`cmd_vel_bridge_config.yaml`**：仅桥接控制指令，便于独立调试。

### **4. `scripts` 文件夹**

- **`custom_node.py`**：可以放置自定义的 ROS 2 节点逻辑，用于处理 Gazebo 数据或实现控制。
- **`scan_processor.py`**：专门处理激光雷达数据，比如过滤、可视化等。
- **`cmd_vel_processor.py`**：处理速度指令，比如平滑控制或加入算法逻辑。

------

## **执行步骤**

1. **构建工作空间**

   - 在 `ros2_ws`下运行：

     ```bash
     colcon build
     source install/setup.zsh
     ```

2. **运行通用桥接**

   - 执行：

     ```bash
     ros2 launch my_bridge bridge.launch.py config_file:=<路径>/bridge_config.yaml
     ```

3. **运行单独桥接**

   - 激光雷达:

     ```bash
     ros2 launch my_bridge scan_bridge.launch.py config_file:=<路径>/scan_bridge_config.yaml
     ```

   - 控制指令：

     ```bash
     ros2 launch my_bridge cmd_vel_bridge.launch.py config_file:=<路径>/cmd_vel_bridge_config.yaml
     ```



------



