我们不应该花太多时间在gazebo身上。



fishros给出的gazebo教程是给classic版本的。

而我是harmonic版本的，



我尝试将教程中的代码一一复现，确实实现了一部分，但是距离落地遥遥无期。

而同时我的大一上学期快结束了，根本没有这么多时间。我需要马上进入正题--建图、导航。





中间有很多都是使用ai生成的内容

# 官方资源集合

- [ros_gz](https://github.com/gazebosim/ros_gz)

#  template包

直接使用这个包，然后去构建就好了。

- [Guide to `ros_gz_project_template` for ROS 2 and Gazebo Development](https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/)
- [`ros_gz_project_template` template](https://github.com/gazebosim/ros_gz_project_template) 

# 提供一些集成的虚拟空间.

1. **[turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)**
   - 提供多个 Gazebo 仿真场景，适用于学习导航、路径规划等。
   - 场景中包含简单的障碍物和地形，支持 RViz 和 Gazebo。
2. **[ros-gz-sim-demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)**
   - 官方提供的 ROS 2 和 Gazebo 集成的演示。
   - 包含车辆模型、简单地形场景等，支持你直接使用或修改。
3. **[navigation2_tutorials](https://github.com/ros-planning/navigation2_tutorials)**
   - 提供与 Gazebo 集成的导航教程，包括如何在虚拟场景中生成物体。



1. **[Gazebo-exercise](https://github.com/zcl2016/Gazebo-exercise)**
    该项目在 Gazebo 7.14 的基础上，采用异步机制的事件调度与消息管理，以及 OpenMP、TBB、MPI 等并行加速技术，优化重构了大型 3D 室外高保真实时仿真平台，支持群体智能机器人协同仿真。
2. **[gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection)**
    此仓库收集了多个 Gazebo 的模型和世界文件，供用户直接使用或参考。
3. **[Autonomous-mobile-robot](https://github.com/TJUUAVLaboratory/Autonomous-mobile-robot)**
    该项目使用 ROS 和 Gazebo 搭建了室内/室外的轮式机器人仿真场景，并集成了 ROS Navigation 机器人感知导航算法，适合用于移动机器人功能的学习和测试。
4. **[sunday](https://github.com/Lord-Z/sunday)**
    该功能包主要用于配置机械臂操作的 Gazebo 仿真环境，包含多个用于发布机械臂关节状态以及与 MoveIt 对接的 launch 文件。
5. **[ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)**
    该项目提供了多个机器人在 3D 环境中的 Gazebo 仿真，支持丰富的动态交互，适用于多种机器人平台的仿真测试。

------



