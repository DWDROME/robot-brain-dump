# robot.h

```cpp
/*
copyright
*/
#ifndef EXAMPLE_ACTION_RCLCPP_ROBOT_H_
#define EXAMPLE_ACTION_RCLCPP_ROBOT_H_
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
   public:
    using MoveRobot = robot_control_interfaces::action::MoveRobot;
    Robot() = default;
    ~Robot() = default;
    float move_step();              // 移动一小步，间隔500ms一次
    bool set_goal(float distance);  // 移动一段举例
    float get_current_pose();
    int get_status();
    bool close_goal();  //是否接近目标
    void stop_move();   //停止移动

   private:
    float current_pose_ = 0.0;              //声明当前位置
    float target_pose_ = 0.0;               //目标位置
    float move_distance_ = 0.0;             //目标距离
    std::atomic<bool> cancel_flag_{false};  //取消标志
    int status_ = MoveRobot::Feedback::STATUS_STOP;
};
#endif  // EXAMPLE_ACTION_RCLCPP_ROBOT_H_
```

## 数据成员和变量

### 公有成员函数

这些成员函数提供了 `Robot` 类的主要功能接口：

1. **`move_step()`**：移动一小步，间隔 500ms。
   - 类型：`float`
   - 功能：根据 `move_distance_` 设置的方向和目标，更新当前位置 `current_pose_` 并返回新位置。

2. **`set_goal(float distance)`**：设置目标位置。
   - 类型：`bool`
   - 参数：`float distance`（要移动的目标距离）
   - 功能：将目标距离设置为 `move_distance_`，更新 `target_pose_`。如果接近目标位置，则设置 `status_` 为 `STATUS_STOP`，否则为 `STATUS_MOVING`。

3. **`get_current_pose()`**：获取当前的机器人位置。
   - 类型：`float`
   - 功能：返回 `current_pose_`，即机器人当前位置。

4. **`get_status()`**：获取当前移动状态。
   - 类型：`int`
   - 功能：返回 `status_`，表示当前的机器人状态（`STATUS_MOVING` 或 `STATUS_STOP`）。

5. **`close_goal()`**：判断是否接近目标。
   - 类型：`bool`
   - 功能：检查当前位置 `current_pose_` 与目标位置 `target_pose_` 之间的距离是否小于 0.01，用于判断是否接近目标位置。

6. **`stop_move()`**：停止机器人移动。
   - 类型：`void`
   - 功能：将 `status_` 设置为 `STATUS_STOP`。

### 私有数据成员

这些数据成员用于存储机器人的内部状态：

1. **`float current_pose_`**：当前位置。

   - 初始值：`0.0`
   - 功能：用于记录机器人当前的位置信息。

2. **`float target_pose_`**：目标位置。

   - 初始值：`0.0`
   - 功能：表示机器人需要到达的目标位置。

3. **`float move_distance_`**：目标距离。

   - 初始值：`0.0`
   - 功能：表示机器人从当前位置到目标位置的总距离。

4. **`std::atomic<bool> cancel_flag_`**：取消标志。

   - 初始值：`false`
   - 功能：用于标记是否取消当前移动操作，以支持线程安全的取消操作。

5. **`int status_`**：移动状态。

   - 初始值：`MoveRobot::Feedback::STATUS_STOP`
   - 功能：表示机器人的当前状态，可能的值包括 `STATUS_MOVING`（正在移动）和 `STATUS_STOP`（停止）。

   ```mermaid
   flowchart LR
       A["Robot"]:::main
   
       A --> Z["私有数据成员"]:::subgroup
       Z --> B["current_pose_ : float (当前位置)"]:::data
       Z --> C["target_pose_ : float (目标位置)"]:::data
       Z --> D["move_distance_ : float (移动距离)"]:::data
       Z --> E["cancel_flag_ : std::atomic<bool> (取消标志)"]:::data
       Z --> F["status_ : int (状态)"]:::data
   
       A --> Y["公有方法"]:::subgroup
       Y --> G["move_step() : float - 移动一小步"]:::method
       Y --> H["set_goal(distance : float) : bool - 设置目标"]:::method
       Y --> I["get_current_pose() : float - 获取当前位置"]:::method
       Y --> J["get_status() : int - 获取状态"]:::method
       Y --> K["close_goal() : bool - 检查接近目标"]:::method
       Y --> L["stop_move() : void - 停止移动"]:::method
       
       %% MoveRobot 状态别名
       subgraph MoveRobot ["MoveRobot::Feedback 状态"]
           M["STATUS_STOP"]
           N["STATUS_MOVING"]
           M -->|开始移动| N  
           N -->|停止| M      
       end
   
       %% 使用 style 指令为每个节点单独设置样式
       style MoveRobot fill:#E6E6FA,stroke:#333,stroke-width:1.5px,color:#333;
       style M fill:#FFFACD,stroke:#333,stroke-width:1px,color:#333;
       style N fill:#FFFACD,stroke:#333,stroke-width:1px,color:#333;
   
       F -->|状态常量| MoveRobot
       H -->|设置状态| F
       L -->|设置为停止状态| F
   
       classDef main fill:#FFD700,stroke:#333,stroke-width:2px,rx:5px,ry:5px,color:#333;
       classDef subgroup fill:#ADD8E6,stroke:#333,stroke-width:1.5px,rx:5px,ry:5px,color:#333;
       classDef data fill:#FFA07A,stroke:#333,stroke-width:1px,color:#333;
       classDef method fill:#98FB98,stroke:#333,stroke-width:1px,color:#333;
       classDef statusVal fill:#FFFACD,stroke:#333,stroke-width:1px,color:#333;
   
   ```
   



## `#ifndef` 和 `#define` ?

在 C++ 中，`#ifndef` 和 `#define` 宏通常用于**防止头文件被多次包含**。这种写法被称为“**包含保护**”（Include Guard），可以确保头文件的内容在一个编译单元中只被包含一次，避免重复定义带来的编译错误。

### 解释

1. **`#ifndef EXAMPLE_ACTION_RCLCPP_ROBOT_H_`**：
   - `#ifndef` 是“if not defined”的缩写，意思是“如果未定义”。
   - `EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 是一个预处理器宏的名称.
   - 如果该宏尚未定义（即此文件是第一次被包含），则执行下面的内容。

2. **`#define EXAMPLE_ACTION_RCLCPP_ROBOT_H_`**：
   - `#define` 指令将 `EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 定义为一个宏。
   - 一旦这个宏被定义，后续任何对这个头文件的包含都会发现 `EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 已经定义了，因此会跳过整个头文件的内容，不再重复包含。

3. **`#endif`**：
   - `#endif` 与 `#ifndef` 相对应，用于结束这个条件编译的块。

### 执行流程

- **第一次包含**：
  - 如果这个文件第一次被包含，`EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 尚未定义。
  - `#ifndef EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 的条件为==TRUE==，因此会定义 `EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 并执行文件中的内容。

- **再次包含**：
  - 如果该文件再次被包含，`EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 已经定义。
  - `#ifndef EXAMPLE_ACTION_RCLCPP_ROBOT_H_` 的条件为==FALSE==，文件内容会被跳过，不会被再次包含。

## std::atomic

**==官方文档==**:

- ["std::atomic"](https://en.cppreference.com/w/cpp/atomic/atomic)

```cpp
std::atomic<bool> cancel_flag_{false};  // 取消标志
```

`std::atomic<bool>` 是 C++ 标准库中的一种**原子类型**，用于在**多线程**环境中安全地操作布尔值 `cancel_flag_`。

### 解释

1. **`std::atomic<bool>`**：
   - `std::atomic` 是一个模板类，它可以包装一个数据类型（在这里是 `bool`），使其在多线程环境中操作时是“原子性”的。
   - **原子性**指的是，变量的操作不会被其他线程中断。对原子变量的读写操作在硬件级别是不可分割的，因此不会出现多个线程同时访问时的数据竞争问题。
   - 使用 `std::atomic<bool>` 确保了 `cancel_flag_` 的读写操作在多线程环境中是安全的，不会因为竞争条件而出现错误的状态。

2. **`cancel_flag_`**：
   
   - 这是一个标志变量，通常用于**通知某个操作或线程应该停止**。
   - 例如，机器人正在执行某个动作，如果外部请求取消操作，`cancel_flag_` 可以被设置为 `true`，其他线程会检测到这个标志并停止动作。
   
   

### 典型

在机器人控制系统或多线程应用中，`cancel_flag_` 通常用于实现**安全的取消操作**。例如，假设有一个机器人正在执行一个移动任务，当任务开始时，`cancel_flag_` 为 `false`，表示正常执行。某一时刻，主线程或外部控制器可能会发出取消指令，这时可以将 `cancel_flag_` 设置为 `true`，执行任务的线程会检测到这个标志并停止任务。

### 总结

- `std::atomic<bool> cancel_flag_{false};` 是一个线程安全的布尔标志，初始为 `false`。
- 在需要取消操作时，可以将其设置为 `true`，其他线程会检测到这一状态并终止相应操作。
- 这是多线程编程中的一种常用方法，用于实现线程间的安全状态传递。

## MoveRobot::Feedback::STATUS_STOP

在这行代码中：

```cpp
int status_ = MoveRobot::Feedback::STATUS_STOP;
```

### 解释

在接口中定义的==(这里推荐你保存，之后会有很多相互调用的地方)==

```cpp
#MoveRobot.action   robot_control_interfaces/action
# Feedback: 中间反馈的位置和状态
float32 pose
uint32 status
uint32 STATUS_MOVING = 3
uint32 STATUS_STOP = 4
```



这里的 `MoveRobot::Feedback::STATUS_STOP` 表示一个枚举值或常量，用于指示机器人当前的**状态**。具体含义如下：

1. **`MoveRobot`**：
   - `MoveRobot` 是一个定义在 `robot_control_interfaces::action` 命名空间中的 Action 接口（action 文件）
2. **`Feedback`**：
   - `Feedback` 是 `MoveRobot` 定义的一个嵌套类型（通常是一个结构体），用于传递动作执行过程中的反馈信息。（这是由rosidl自动生成的）
3. **`STATUS_STOP`**：
   - `STATUS_STOP` 是 `MoveRobot::Feedback` 中定义的一个**状态常量**或**枚举值**，用于表示机器人当前处于“停止”状态。

### 具体作用

将 `status_` 变量初始化为 `MoveRobot::Feedback::STATUS_STOP` 表示机器人一开始处于“停止”状态。

**状态转换包括**：

- 从 `STATUS_STOP` 转变为 `STATUS_MOVING`（机器人开始移动）。
- 从 `STATUS_RUNNING` 转变为 `STATUS_STOP`（机器人到达目标或收到停止命令）。

==具体实现的定义将在robot.h中被完善==
