

---

# urdf的构建



## 功能包的构建

- ```zsh
  ros2 pkg create fishbot_description --build-type ament_python
  ```

## 创建urdf文件和launch文件

- ```zsh
  cd fishbot_description && mkdir urdf && cd urdf 
  touch fishbot_base.urdf
  cd .. && mkdir launch && cd launch
  touch display_rviz2.launch.py
  ```

## fishbot_base.urdf

- ```xml
  <?xml version="1.0"?>
  <robot name="fishbot">
      
    <!-- base link -->
    <link name="base_link">
        <visual>
        <origin xyz="0 0 0.0" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.12" radius="0.10"/>
        </geometry>
      </visual>
    </link>
      
    <!-- laser link -->
    <link name="laser_link">
        <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.02" radius="0.02"/>
        </geometry>
        <material name="black">
            <color rgba="0.0 0.0 0.0 0.8" /> 
        </material>
      </visual>
    </link>
      
    <!-- laser joint -->
      <joint name="laser_joint" type="fixed">
          <parent link="base_link" />
          <child link="laser_link" />
          <origin xyz="0 0 0.075" />
      </joint>
  
  </robot>
  ```

## display_rviz2.launch.py

- ```python
  import os
  from launch import LaunchDescription
  from launch.substitutions import LaunchConfiguration 
  from launch_ros.actions import Node
  from launch_ros.substitutions import FindPackageShare
  
  def generate_launch_description():
      package_name = 'fishbot_description'
      urdf_name = 'fishbot_base.urdf'
  
      ld = LaunchDescription()
      pkg_share = FindPackageShare(package=package_name).find(package_name)
      urdf_model_path = os.path.join(pkg_share , f'urdf/{urdf_name}')
  
      robot_state_publisher_node = Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          arguments=[urdf_model_path]
      )
  
      joint_state_publisher_node = Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          name='joint_state_publisher_gui',
          arguments=[urdf_model_path]
      )
  
      rviz2_node = Node(
          package='rviz2',
          executable='rviz2',
          name = 'rviz2',
          output='screen'
      )
  
      ld.add_action(robot_state_publisher_node)
      ld.add_action(joint_state_publisher_node)
      ld.add_action(rviz2_node)
  
      return ld
  ```

  ###  **导入必要的模块**

  ```python
  import os
  from launch import LaunchDescription
  from launch.substitutions import LaunchConfiguration
  from launch_ros.actions import Node
  from launch_ros.substitutions import FindPackageShare
  ```

  - **`LaunchDescription`**: 用于定义和管理要启动的节点。
  - **`LaunchConfiguration`**: 可用于在启动时传递配置参数（虽然在此例中未使用）。
  - **`Node`**: 用于定义一个 ROS 2 节点，指定它的包和可执行文件。
  - **`FindPackageShare`**: 查找一个包的共享目录，通常用来查找 URDF 等文件。

  ###  **定义启动描述**

  ```python
  def generate_launch_description():
      package_name = 'fishbot_description'
      urdf_name = 'fishbot_base.urdf'
  
      ld = LaunchDescription()
  ```

  - **`generate_launch_description`**: 启动文件的核心函数，负责返回启动的所有描述信息。
  - **`package_name`** 和 **`urdf_name`**: 定义了要加载的包名和 URDF 文件名。

  ###  **查找 URDF 文件路径**

  ```python
      pkg_share = FindPackageShare(package=package_name).find(package=package_name)
      urdf_model_path = os.path.join(pkg_share , f'urdf/{urdf_name}')
  ```

  - **`FindPackageShare`**: 查找指定包的共享目录。这里通过该方法找到 `fishbot_description` 包的目录，并拼接出 URDF 文件的路径。
  - **路径拼接**: 使用 `os.path.join()` 来确保跨平台的路径兼容性。

  ###  **定义并启动节点**

  ```python
      robot_state_publisher_node = Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          arguments=[urdf_model_path]
      )
  
      joint_state_publisher_node = Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          name='joint_state_publisher_gui',
          arguments=[urdf_model_path]
      )
  
      rviz2_node = Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          output='screen'
      )
  ```

  - 每个 

    ```
    Node
    ```

     对象代表一个 ROS 2 节点。你需要指定：

    - **`package`**: 包名
    - **`executable`**: 可执行文件名
    - **`arguments`**: 节点需要的参数或文件路径（这里是 URDF 模型路径）

  ###  **将节点添加到启动描述中**

  ```python
      ld.add_action(robot_state_publisher_node)
      ld.add_action(joint_state_publisher_node)
      ld.add_action(rviz2_node)
  
      return ld
  ```

  - **`add_action`**: 将节点添加到 `LaunchDescription` 中，指示 ROS 2 启动时运行这些节点。

  ### **难点解析**

  - **`LaunchConfiguration` 的使用**：用于动态地从启动时传递的参数中读取配置。虽然在该示例中未使用，但通常用于需要在启动时指定的参数，如 URDF 文件路径等。
  - **URDF 文件路径拼接**：使用 `os.path.join()` 而不是直接字符串拼接，可以确保在不同操作系统上都能正确处理路径分隔符。

## setup.py

- ```python
  import os
  from glob import glob
  from setuptools import find_packages, setup
  
  package_name = 'fishbot_description'
  
  setup(
      name=package_name,
      version='0.0.0',
      packages=find_packages(exclude=['test']),
     #packages=[package_name]
      data_files=[
          ('share/ament_index/resource_index/packages',
              ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
          (os.path.join('share',package_name,'urdf'),glob('urdf/**')),
      ],
      install_requires=['setuptools'],
      zip_safe=True,
      maintainer='dw',
      maintainer_email='dw@todo.todo',
      description='TODO: Package description',
      license='TODO: License declaration',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [
          ],
      },
  )
  ```

  主要是在这里修改(date_files)

  - 原来

    - ```python
      data_files=[
              ('share/ament_index/resource_index/packages',
                  ['resource/' + package_name]),
              ('share/' + package_name, ['package.xml']),
      ],
      ```

  - 后来

    - ```python
      data_files=[
              ('share/ament_index/resource_index/packages',
                  ['resource/' + package_name]),
              ('share/' + package_name, ['package.xml']),
              (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
              (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
          ],
      ```

  同时注意一下packages

  - 在humble中是`packages=[package_name],`

## 编译运行

- 在`~/.zshrc`中添加`source ~/ros2_ws/install/setup.zsh`

- 然后在工作空间中（不是功能包）执行

  - ```zsh
    colcon build
    source ~/.zshrc
    ros2 launch fishbot_description display_rviz2.launch.py
    ```





##  为什么在joint_state_publisher_gui中出现了可控选项?

你可以观察到在 `joint_state_publisher_gui` 中出现了角度操作选项

- `left_wheel_joint` 
-  `right_wheel_joint` 



这是因为 **`joint_state_publisher_gui` 会自动根据 URDF 文件中的关节定义生成操作界面**。具体原因和机制如下：

------

### **1. 为什么会生成角度操作选项？**

**URDF 文件中关节类型的影响**

在你的 URDF 文件中，`left_wheel_joint` 和 `right_wheel_joint` 的关节类型被定义为 **`continuous`**：

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link" />
  <child link="left_wheel_link" />
  <origin xyz="-0.02 0.10 -0.06" />
  <axis xyz="0 1 0" />
</joint>

<joint name="right_wheel_joint" type="continuous">
  <parent link="base_link" />
  <child link="right_wheel_link" />
  <origin xyz="-0.02 -0.10 -0.06" />
  <axis xyz="0 1 0" />
</joint>
```

- **`continuous` 类型关节**：
  - 表示关节可以无限旋转，通常用于轮子或转台等部件。
  - 需要关节的角度值（即位置）来定义状态。
- **`fixed` 类型关节**：
  - 表示关节是固定的，没有运动自由度，例如 `laser_joint` 和 `imu_joint`。
  - 不需要角度值，`joint_state_publisher_gui` 不会为其生成可操作选项。

------

#### **`joint_state_publisher_gui` 的工作原理**

1. **解析 URDF 文件：**
   - `joint_state_publisher_gui` 会读取 URDF 文件中所有关节。
   - 对于每个非 `fixed` 类型的关节，它会生成一个操作滑块，用于用户调整关节的角度值。
2. **生成角度控制选项：**
   - 如果关节类型是 `revolute` 或 `continuous`，它会为这些关节生成一个滑块，允许用户设置角度值。
   - `continuous` 关节没有范围限制，但可以显示当前角度。
   - `revolute` 关节会受到 URDF 中 `<limit>` 定义的角度范围限制。





