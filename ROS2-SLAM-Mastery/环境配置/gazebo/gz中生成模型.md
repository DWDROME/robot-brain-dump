# 生成模型



## 通过服务调用生成

还记得`parameter_bridge` 吗？

我们在生成桥接的时候调用了这个cpp文件。



- 一个 `parameter_bridge` 实例用于消息传递（传感器数据）。==之前的例子==

- 另一个 `parameter_bridge` 实例用于服务桥接（动态生成模型）。==现在的例子==

------

### **实现多开的方法**

#### **1. 启动消息桥接**

用第一个 `parameter_bridge` 实例桥接消息话题，例如传递传感器数据：

```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

- **功能**：桥接 Gazebo 和 ROS 2 的激光雷达数据。

------

#### **2. 启动服务桥接**

启动第二个 `parameter_bridge` 实例，用于桥接服务，例如生成模型：

```bash
ros2 run ros_gz_bridge parameter_bridge /world/empty/create@ros_gz_interfaces/srv/SpawnEntity
```

- **功能**：桥接 Gazebo 和 ROS 2 的服务，允许通过 ROS 2 服务生成模型。

------

#### **3. 发送请求生成模型**

在服务桥接启动后，使用以下命令发送生成模型的请求：

```bash
ros2 service call /world/empty/create ros_gz_interfaces/srv/SpawnEntity \
  "{name: 'my_robot', sdf: '<完整的SDF内容>', pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```



------

### **总结两种调用的区别**

| **功能**     | **话题桥接**                                            | **服务桥接**                                            |
| ------------ | ------------------------------------------------------- | ------------------------------------------------------- |
| **调用命令** | `/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan`     | `/world/empty/create@ros_gz_interfaces/srv/SpawnEntity` |
| **桥接对象** | ROS 2 和 Gazebo 的话题                                  | ROS 2 和 Gazebo 的服务                                  |
| **桥接方向** | 支持单向或双向（GZ_TO_ROS、ROS_TO_GZ 或 BIDIRECTIONAL） | 服务请求从 ROS 2 转发到 Gazebo，返回响应                |
| **使用场景** | 数据流桥接，如传感器数据                                | 动作桥接，如动态生成模型                                |

------

### **同时运行的效果**

- 两个 `parameter_bridge` 实例可以独立运行，并不会互相干扰。
- 一个处理话题消息，另一个处理服务请求。



---

### **`parameter_bridge` 中服务桥接的实现**

在这个调用服务的过程中，我十分好奇，到底是哪一个地方起了作用。

#### **关键实现代码**

1. **解析服务参数**：

   ```cpp
   if (config.ros_type_name.find("/srv/") != std::string::npos) {
      std::string gz_req_type_name;
      std::string gz_rep_type_name;
      if (config.direction == BridgeDirection::ROS_TO_GZ ||
          config.direction == BridgeDirection::GZ_TO_ROS) {
        usage();
        return -1;
      }
      if (config.direction == BridgeDirection::BIDIRECTIONAL) {
        delimPos = arg.find(delim);
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        }
        gz_req_type_name = arg.substr(0, delimPos);
        arg.erase(0, delimPos + delim.size());
        gz_rep_type_name = std::move(arg);
      }
      try {
        bridge_node->add_service_bridge(
          config.ros_type_name,
          gz_req_type_name,
          gz_rep_type_name,
          config.ros_topic_name);
      } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
      }
      continue;
   }
   ```

   - 解析传入参数，判断是否为服务桥接（`ros_type_name` 包含 `/srv/`）。
   - 处理服务的请求类型和响应类型（`gz_req_type_name` 和 `gz_rep_type_name`）。

2. **调用服务桥接方法**：

   ```cpp
   bridge_node->add_service_bridge(
      config.ros_type_name,
      gz_req_type_name,
      gz_rep_type_name,
      config.ros_topic_name);
   ```

   - 使用 `RosGzBridge::add_service_bridge` 方法注册服务桥接。
   - 桥接逻辑会将 ROS 2 的服务请求转发给 Gazebo 服务，并将响应返回给 ROS 2 客户端。

3. **服务桥接的核心逻辑**：

   - Gazebo 服务通过内部的 Gazebo Transport 提供功能。
   - ROS 2 服务通过 `rclcpp` 提供功能。
   - 桥接会订阅 Gazebo 服务，并将其暴露为一个 ROS 2 服务。
   - 响应流向：
     1. ROS 2 客户端 -> ROS 2 服务。
     2. ROS 2 服务 -> Gazebo 服务。
     3. Gazebo 服务 -> ROS 2 服务 -> ROS 2 客户端。

---

##  通过自带包生成

在`ros_gz_sim`里面有一个`gz_spawn_model`的东西，它可以帮助我们在一个已经建成的gz中添加模型。



```bash
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5
```



你可以打开这个文件，看看哪个没有默认值，没有默认值的需要你指定参数。一般是 `world`和`file`需要指定参数。

## 自定义配置

在功能包里面创建`xml`或者`launch.py`就行了



例如

```xml
<launch>
  <arg name="world" default="" />
  <arg name="file" default="" />
  <arg name="model_string" default="" />
  <arg name="topic" default="" />
  <arg name="entity_name" default="" />
  <arg name="allow_renaming" default="False" />
  <arg name="x" default="" />
  <arg name="y" default="" />
  <arg name="z" default="" />
  <arg name="roll" default="" />
  <arg name="pitch" default="" />
  <arg name="yaw" default="" />
  <gz_spawn_model 
    world="$(var world)"
    file="$(var file)"
    model_string="$(var model_string)"
    topic="$(var topic)"
    entity_name="$(var entity_name)"
    allow_renaming="$(var allow_renaming)"
    x="$(var x)"
    y="$(var y)"
    z="$(var z)"
    roll="$(var roll)"
    pitch="$(var pitch)"
    yaw="$(var yaw)">
  </gz_spawn_model>
</launch>
```



```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='', description='Name of the Gazebo world'),
        DeclareLaunchArgument('file', default_value='', description='Path to the model file'),
        DeclareLaunchArgument('model_string', default_value='', description='Model description as a string'),
        DeclareLaunchArgument('topic', default_value='', description='Gazebo topic to use'),
        DeclareLaunchArgument('entity_name', default_value='', description='Name of the entity to spawn'),
        DeclareLaunchArgument('allow_renaming', default_value='False', description='Allow renaming of the entity'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position of the model'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position of the model'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z position of the model'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll rotation of the model'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch rotation of the model'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw rotation of the model'),

        # Gazebo spawn model node
        Node(
            package='ros_gz_sim',
            executable='gz_spawn_model',
            output='screen',
            name='spawn_model',
            parameters=[
                {'world': LaunchConfiguration('world')},
                {'file': LaunchConfiguration('file')},
                {'model_string': LaunchConfiguration('model_string')},
                {'topic': LaunchConfiguration('topic')},
                {'entity_name': LaunchConfiguration('entity_name')},
                {'allow_renaming': LaunchConfiguration('allow_renaming')},
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')},
                {'roll': LaunchConfiguration('roll')},
                {'pitch': LaunchConfiguration('pitch')},
                {'yaw': LaunchConfiguration('yaw')}
            ]
        ),
    ])
```

当然我们也可以简化一些

我们调用了`parameter_bridge`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 创建 LaunchDescription
    ld = LaunchDescription()

    # 声明参数
    ld.add_action(DeclareLaunchArgument('entity_name', default_value='my_robot', description='Name of the entity to spawn'))
    ld.add_action(DeclareLaunchArgument('file', default_value='/path/to/your/model.sdf', description='Path to the model file'))
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0', description='X position of the model'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0', description='Y position of the model'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.0', description='Z position of the model'))

    # 配置 parameter_bridge，用于桥接 Gazebo 和 ROS 服务
    parameter_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )

    # 添加到 LaunchDescription
    ld.add_action(parameter_bridge_cmd)

    return ld
```







| **XML 元素**             | **Python 替代**                        |
| ------------------------ | -------------------------------------- |
| `<arg name="..." />`     | `DeclareLaunchArgument`                |
| `<gz_spawn_model ... />` | `Node`，通过 `parameters` 字段传递参数 |

------

# **同时启动？**



在官方教程中，我找到了这一个代码

```bash
ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5 bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file>  
```





他说，可以同时桥接gz-ros和生成模型



------

1. **关键参数的功能**

- **`world:=empty`**
  - 指定要加载的 Gazebo 世界为 `empty`（空白世界）。
  - Gazebo 会在这个世界中加载模型。
- **`file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf`**
  - 指定要加载的模型文件路径。
  - 在这里，加载的是 `vehicle` 模型的 SDF 文件。
- **`entity_name:=my_vehicle`**
  - 模型的名称，生成后的模型会被命名为 `my_vehicle`。
- **`x:=5.0 y:=5.0 z:=0.5`**
  - 指定模型的初始位置：
    - `x`：模型在 Gazebo 世界中的 X 坐标。
    - `y`：模型在 Gazebo 世界中的 Y 坐标。
    - `z`：模型在 Gazebo 世界中的 Z 坐标。
- **`bridge_name:=ros_gz_bridge`**
  - 定义桥接节点的名称为 `ros_gz_bridge`。
  - 这是 `ros_gz_bridge` 在 ROS 2 网络中的唯一标识。
- **`config_file:=<path_to_your_YAML_file>`**
  - 指定一个 YAML 配置文件，用于定义要桥接的话题和服务。

------

# **通用的桥接 YAML 文件**

以下是一个桥接 YAML 配置文件实例。

网络收集，可能会有错误。

`bridge_config.yaml`



```yaml
# 桥接 Gazebo 服务到 ROS 2 服务
services:
  # 模型生成服务
  - ros_service_name: "/world/empty/create"
    gz_service_name: "/gazebo/spawn_entity"
    ros_service_type: "ros_gz_interfaces/srv/SpawnEntity"
    gz_request_type: "gz.msgs.EntityFactory"
    gz_response_type: "gz.msgs.Boolean"

  # 模型删除服务
  - ros_service_name: "/world/empty/delete"
    gz_service_name: "/gazebo/delete_entity"
    ros_service_type: "ros_gz_interfaces/srv/DeleteEntity"
    gz_request_type: "gz.msgs.Entity"
    gz_response_type: "gz.msgs.Boolean"

  # 世界控制服务
  - ros_service_name: "/world/empty/control"
    gz_service_name: "/gazebo/control_world"
    ros_service_type: "ros_gz_interfaces/srv/ControlWorld"
    gz_request_type: "gz.msgs.WorldControl"
    gz_response_type: "gz.msgs.Boolean"

# 桥接 Gazebo 和 ROS 2 的话题
topics:
  # 激光雷达数据
  - ros_topic_name: "/scan"
    gz_topic_name: "/gazebo/laser_scan"
    ros_type_name: "sensor_msgs/msg/LaserScan"
    gz_type_name: "gz.msgs.LaserScan"
    direction: BIDIRECTIONAL

  # 摄像头图像数据
  - ros_topic_name: "/camera/image"
    gz_topic_name: "/gazebo/camera/image"
    ros_type_name: "sensor_msgs/msg/Image"
    gz_type_name: "gz.msgs.Image"
    direction: GZ_TO_ROS

  # 深度图像数据
  - ros_topic_name: "/camera/depth"
    gz_topic_name: "/gazebo/camera/depth_image"
    ros_type_name: "sensor_msgs/msg/Image"
    gz_type_name: "gz.msgs.Image"
    direction: GZ_TO_ROS

  # 点云数据
  - ros_topic_name: "/points"
    gz_topic_name: "/gazebo/point_cloud"
    ros_type_name: "sensor_msgs/msg/PointCloud2"
    gz_type_name: "gz.msgs.PointCloudPacked"
    direction: GZ_TO_ROS

  # 机器人位置数据
  - ros_topic_name: "/robot_pose"
    gz_topic_name: "/gazebo/pose/info"
    ros_type_name: "geometry_msgs/msg/Pose"
    gz_type_name: "gz.msgs.Pose"
    direction: GZ_TO_ROS

  # 速度命令
  - ros_topic_name: "/cmd_vel"
    gz_topic_name: "/gazebo/cmd_vel"
    ros_type_name: "geometry_msgs/msg/Twist"
    gz_type_name: "gz.msgs.Twist"
    direction: ROS_TO_GZ

  # 关节状态数据
  - ros_topic_name: "/joint_states"
    gz_topic_name: "/gazebo/joint_states"
    ros_type_name: "sensor_msgs/msg/JointState"
    gz_type_name: "gz.msgs.Model"
    direction: GZ_TO_ROS

  # TF 数据（用于坐标变换）
  - ros_topic_name: "/tf"
    gz_topic_name: "/gazebo/tf"
    ros_type_name: "tf2_msgs/msg/TFMessage"
    gz_type_name: "gz.msgs.Pose_V"
    direction: GZ_TO_ROS

  # 世界状态数据
  - ros_topic_name: "/world_state"
    gz_topic_name: "/gazebo/world_state"
    ros_type_name: "ros_gz_interfaces/msg/WorldState"
    gz_type_name: "gz.msgs.WorldStatistics"
    direction: GZ_TO_ROS
```



------





