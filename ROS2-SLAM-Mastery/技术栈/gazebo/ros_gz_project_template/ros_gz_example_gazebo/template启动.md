1. - ### **笔记：启动 Gazebo Harmonic 并配置 SLAM 环境**

     ------

     0. 

     ```zsh
     colcon build --cmake-args -DBUILD_TESTING=ON
     ```
     
     #### **1. 启动 Gazebo 和 ROS 2**

     - 使用以下命令启动 Gazebo 和 ROS 2 环境：

       ```bash
       ros2 launch ros_gz_example_bringup slam.launch.py
       ```
     
     ------

     #### **2. 在 Gazebo 中添加键盘控制**

     - 在 Gazebo 仿真窗口中：
       1. 点击右上角工具栏。
       2. 添加 `Key Publisher` 组件，用于发布键盘输入消息。

     ------
     
     #### **3. 配置 Gazebo 与 ROS 2 的桥接**
     
     - 使用以下命令桥接必要话题：

       ```bash
       ros2 run ros_gz_bridge parameter_bridge /diff_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
       ros2 run ros_gz_bridge parameter_bridge 
       /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
       ```

     ------
     
     #### **4. 验证 ROS 2 环境**

     - **检查 ROS 2 节点是否启动：**

       ```bash
       ros2 node list
       ```

     - **检查话题是否存在：**

       ```bash
       ros2 topic list
       ```

     ------

     #### **5. 手动测试 Gazebo 和 ROS 2**

     - **测试 Gazebo 中的键盘输入：**
     
       1. 在 Gazebo 中，按上下左右键，验证小车是否响应。

       2. 检查键盘话题：

          ```bash
          gz topic -e -t /keyboard/keypress
          ```

       3. 检查 

          ```
          cmd_vel
          ```

           数据是否生成：

          ```bash
          gz topic -e -t /diff_drive/cmd_vel
          ```
     
     - **测试 ROS 2 的手动消息发布：**

       1. 前进：

          ```bash
          ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
          ```

       2. 左转：

          ```bash
          ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
          ```

       3. 后退：

          ```bash
          ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
          ```

       4. 右转：

          ```bash
          ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"
          ```

     ------
     
     #### **6. 检查传感器数据**

     - 验证激光雷达话题：

       ```bash
       ros2 topic echo /diff_drive/scan
       ```

     ------
     
     #### **7. 检查 Gazebo 日志**

     - 查看 Gazebo 启动时的日志：

       ```bash
       gz sim -v 4 slam_demo.sdf
       ```
     
     ------
     
     ### **完整检查步骤**
     
     1. 启动 Gazebo 和 ROS 2，加载环境。
     2. 添加 `Key Publisher`，检查是否响应键盘输入。
     3. 配置并启动 `parameter_bridge`，确保桥接成功。
     4. 验证节点和话题列表，确保 `/diff_drive/cmd_vel` 和 `/diff_drive/scan` 存在。
     5. 手动发布控制消息，确认小车的运动功能。
     6. 检查传感器话题，确认激光雷达输出是否正常。
     7. 检查日志，确保没有插件加载错误。