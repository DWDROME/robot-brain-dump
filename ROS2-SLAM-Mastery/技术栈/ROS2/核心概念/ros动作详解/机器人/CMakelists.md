## CMakelists

```bash
// robot
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# action_robot节点

add_executable(action_robot_01 
    src/robot.cpp
    src/action_robot_01.cpp
)
target_include_directories(action_robot_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(action_robot_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  action_robot_01
  "rclcpp"
  "rclcpp_action"
  "robot_control_interfaces"
  "example_interfaces"
)

install(TARGETS action_robot_01
  DESTINATION lib/${PROJECT_NAME})

# action_control节点

add_executable(action_control_01 
  src/action_control_01.cpp
)
target_include_directories(action_control_01 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(action_control_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  action_control_01
  "rclcpp"
  "rclcpp_action"
  "robot_control_interfaces"
  "example_interfaces"
)

install(TARGETS action_control_01
DESTINATION lib/${PROJECT_NAME})
```

## finds_packages()

**提供名字以找到整个程序所需要依赖的核心包**

### 使用 `find_package`

`find_package` 命令的基本语法如下：

```cmake
find_package(<PackageName> [version] [REQUIRED] [COMPONENTS components...])
```

- `<PackageName>`：要查找的包名称（例如 `rclcpp`、`ament_cmake`、`Boost` 等）。
- `[version]`：指定的包的最低版本号（可选）。
- `[REQUIRED]`：如果指定，表示这个包是必须的，找不到则停止配置过程并报错。
- `[COMPONENTS components...]`：指定包中的特定组件（可选）。

### 本例中的依赖包与核心包

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
```

### 解释

- `find_package(ament_cmake REQUIRED)`：查找 `ament_cmake` 包。
- `find_package(rclcpp REQUIRED)`：查找 ROS 2 的 C++ 库 `rclcpp`，用于编写 ROS 2 节点。
- `find_package(robot_control_interfaces REQUIRED)`：查找 `robot_control_interfaces` 包，这是我们的自定义接口，包含了 `MoveRobot.action`。
- `find_package(example_interfaces REQUIRED)`：查找 ROS 2 中的 `example_interfaces` 包，它提供了一些基本的接口。
- `find_package(rclcpp_action REQUIRED)`：查找 `rclcpp_action` 包，用于支持 ROS 2 中的 action 功能。

### `find_package` 的工作原理

- CMake 会在系统的默认路径和 `CMAKE_PREFIX_PATH` 中搜索所需的包。如果找到符合条件的包，会加载其配置文件（例如，`<PackageName>Config.cmake` 或 `<package>.cmake`）。
- 一旦找到并加载了包的配置文件，CMake 就知道如何包含和链接该包的头文件和库文件。



### `add_executable`

在 CMake 构建系统中，`add_executable` 用于定义一个可执行文件，并指定它的源文件。

```cmake
# action_robot_01 节点
add_executable(action_robot_01 
    src/robot.cpp
    src/action_robot_01.cpp
)
```

#### 解释每一部分

1. **`add_executable`**：
   
   - `add_executable` 是 CMake 的一个指令，用于定义一个可执行目标（即最终生成的可执行文件）。
   - 语法：`add_executable(<name> [sources...])`，其中 `<name>` 是可执行文件的名称，`[sources...]` 是该可执行文件所需的源文件列表。
   
2. **`action_robot_01`**：
   - 这是要生成的可执行文件的名称。在编译后，会生成一个名为 `action_robot_01` 的可执行文件。
   - 在执行 `make` 编译后，生成的可执行文件路径通常位于 `build` 文件夹中的指定位置（例如 `build/<project_name>/action_robot_01`）。

3. **`src/robot.cpp`** 和 **`src/action_robot_01.cpp`**：
   
   - 这些是组成 `action_robot_01` 可执行文件的源文件。
   
   - `src/robot.cpp` 和 `src/action_robot_01.cpp` 会被一起编译并链接，以生成 `action_robot_01`。
   
   - CMake 会将这些源文件一起编译，然后将编译出的目标对象文件链接成一个单一的可执行文件 `action_robot_01`。
   
     

==也就是说，`action_robot_01`需要`src/robot.cpp` 和 `src/action_robot_01.cpp`这两个依赖。==

在生成`action_robot_01`节点时，会同时执行`src/robot.cpp` 



## target_include_directories()

在 CMake 中，`target_include_directories` 用于为指定的目标（例如可执行文件或库）设置头文件的包含路径。通过 `target_include_directories` 命令，就可以确保在编译目标时，编译器能够找到它所依赖的头文件哩。

### 基本语法

```cmake
target_include_directories(<target> <INTERFACE|PUBLIC|PRIVATE> [directories...])
```

- `<target>`：目标名称，例如可执行文件或库的名称。
- `<INTERFACE|PUBLIC|PRIVATE>`：可见性修饰符，决定包含目录的可见性：
  - **PRIVATE**：只有当前目标能用这些头文件，别人不能用。
  - **INTERFACE**：当前目标不用这些头文件，但它的依赖目标可以用。
  - **PUBLIC**：当前目标和依赖它的其他目标都可以用这些头文件。
- `[directories...]`：一个或多个包含目录的路径。

### 示例

```cmake
target_include_directories(action_robot_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
```

在该示例中：

- `action_robot_01` 是目标名。
- `PUBLIC` 关键字表示包含目录在 `action_robot_01` 目标和依赖该目标的其他目标中都可见。
- `$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>`：在构建时使用 `${CMAKE_CURRENT_SOURCE_DIR}/include` 作为包含路径。

  1. `CMAKE_SOURCE_DIR`：始终指向项目的根目录 `chapt4_ws`，在任何子目录中都不变。

  2. `CMAKE_CURRENT_SOURCE_DIR`：指向当前处理的 `CMakeLists.txt` 所在的目录。例如在 `example_action_rclcpp` 的 `CMakeLists.txt` 中，它指向`chapt4/src/example_action_rclcpp`。

  3. `CMAKE_CURRENT_BINARY_DIR`：指向当前目录在构建目录下的对应路径。例如在 `example_action_rclcpp` 的 `CMakeLists.txt` 中，它指向 `chapt4/build/src/example_action_rclcpp`。
- `$<INSTALL_INTERFACE:include>`：在安装后使用 `include` 目录作为包含路径。

## target_compile_features()

指定 C++ 或 C 的特定标准版本

## ament_target_dependencies()

```bash
ament_target_dependencies(
  action_robot_01
  "rclcpp"
  
  "rclcpp_action"
  "robot_control_interfaces"
  "example_interfaces"
)
```

此命令将 `"rclcpp"  "rclcpp_action"  "robot_control_interfaces" "example_interfaces"`的包含路径、库链接等自动添加到 `action_robot_01`，确保正确编译和链接。

## install()

在 CMake 中，`install` 命令用于指定安装目标文件的位置，将编译后的文件放入指定的安装目录。

```bash
install(TARGETS action_robot_01
  DESTINATION lib/${PROJECT_NAME})
```

- `action_robot_01`：要安装的目标，例如可执行文件或库。
- `DESTINATION <path>`：指定目标文件的安装路径，通常是相对于安装根目录的路径。
- `${PROJECT_NAME}` 是项目名称变量。
