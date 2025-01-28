设置自动化脚本的目的是简化您在每个新的工作空间中启动开发环境的流程。通过这个脚本，您可以在一个步骤中完成环境加载、生成编译命令文件和启动VS Code的工作，避免每次都手动执行这些命令。

### 具体步骤

#### 1. 创建脚本文件

在您的主目录（或其他方便的位置）中创建一个脚本文件，比如 `start_ros2_ws.sh`：

```bash
nano ~/start_ros2_ws.sh
```

#### 2. 添加脚本内容

在脚本文件中输入以下内容：

```bash
#!/bin/bash

# 加载ROS 2 Humble环境
source /opt/ros/humble/setup.bash

# 加载您的自定义工作空间环境（如果有）
source ~/your_custom_ws/install/setup.bash  # 请替换为您的自定义工作空间路径

# 在当前目录执行构建，并生成 compile_commands.json 文件
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 启动VS Code，并将当前目录作为工作空间
code .
```

这段脚本做了以下几件事情：

1. **加载ROS 2环境**：使用 `source /opt/ros/humble/setup.bash` 加载ROS 2 Humble的环境变量，使得终端和VS Code能够识别ROS 2相关路径。
2. **加载自定义工作空间**（可选）：如果您有自定义的ROS工作空间（比如接口包、第三方依赖），可以使用类似 `source ~/your_custom_ws/install/setup.bash` 的命令加载该工作空间的环境。将 `your_custom_ws` 替换为您的实际工作空间路径。
3. **构建并生成 `compile_commands.json`**：运行 `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 来构建工作空间中的包，并生成 `compile_commands.json` 文件，这样VS Code可以自动解析头文件路径和编译选项。
4. **启动VS Code**：运行 `code .` 将当前目录作为工作空间打开VS Code。

#### 3. 赋予脚本执行权限

在终端中为脚本赋予执行权限：

```bash
chmod +x ~/start_ros2_ws.sh
```

#### 4. 使用脚本

当您进入新的ROS 2工作空间目录后，可以运行以下命令来启动VS Code并自动设置好所有环境：

```bash
~/start_ros2_ws.sh
```

### 优点

- **自动化加载环境**：每次运行脚本时自动加载ROS 2和自定义工作空间环境，确保依赖路径正确。
- **自动生成`compile_commands.json`**：无需手动配置`c_cpp_properties.json`，VS Code会从 `compile_commands.json` 获取所有包含路径和编译选项。
- **一键启动**：只需运行一个命令，便可以进入完整的开发环境，大幅减少重复的配置步骤。

这种方法尤其适用于多个工作空间的情况，因为它避免了重复的手动配置，确保每个工作空间的一致性。