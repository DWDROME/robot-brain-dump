# Robot.cpp

```cpp
// Robot.cpp
#include "example_action_rclcpp/robot.h"

//移动一小步，500ms一次
float Robot::move_step() {
    //每一步移动到 当前到目标举例的 1/10
    int direct = move_distance_ / fabs(move_distance_);
    float step = direct * fabs(target_pose_ - current_pose_) * 0.1;

    current_pose_ += step;
    std::cout << "移动: " << step << " 当前位置: " << current_pose_ << std::endl;
    return current_pose_;
}

//移动一段距离
bool Robot::set_goal(float distance) {
    move_distance_ = distance;
    target_pose_ += move_distance_;

    //当目标距离和当前距离大于0.1 ，同意想目标移动
    if (close_goal()) {
        status_ = MoveRobot::Feedback::STATUS_STOP;
        return false;
    }
    status_ = MoveRobot::Feedback::STATUS_MOVING;
    return true;
}

float Robot::get_current_pose() { return current_pose_; }
int Robot::get_status() { return status_; }

bool Robot::close_goal() { return fabs(target_pose_ - current_pose_) < 0.01; }
void Robot::stop_move() { status_ = MoveRobot::Feedback::STATUS_STOP; }

```

这是 `Robot` 类的实现代码，用于模拟机器人移动。

即定义了在`Robot.h`中声明的方法。

## 1. `move_step()`

```cpp
float Robot::move_step() {
    int direct = move_distance_ / fabs(move_distance_);
    float step = direct * fabs(target_pose_ - current_pose_) * 0.1;

    current_pose_ += step;
    std::cout << "移动: " << step << " 当前位置: " << current_pose_ << std::endl;
    return current_pose_;
}
```

- **功能**：让机器人每次向目标位置移动一小步。移动的步长是距离目标位置的 `1/10`，并且方向由 `move_distance_`==(Robot类的私有数据成员)== 确定。
- **返回值**：返回当前的 `current_pose_`。

## 2. `set_goal(float distance)`

```cpp
bool Robot::set_goal(float distance) {
    move_distance_ = distance;
    target_pose_ += move_distance_;

    if (close_goal()) {
        status_ = MoveRobot::Feedback::STATUS_STOP;
        return false;
    }
    status_ = MoveRobot::Feedback::STATUS_MOVING;
    return true;
}
```

- **功能**：设定机器人移动的目标位置。调用此函数时，`move_distance_` 被设为目标距离，`target_pose_` 被更新。
- **逻辑**：如果机器人已经接近目标（通过 `close_goal()` 检查），则设置状态为 `STATUS_STOP` 并返回 `false`；否则，设置状态为 `STATUS_MOVING` 并返回 `true`。

## 3. `get_current_pose()`

```cpp
float Robot::get_current_pose() { return current_pose_; }
```

- **功能**：返回当前的位置 `current_pose_`。

## 4. `get_status()`

```cpp
int Robot::get_status() { return status_; }
```

- **功能**：返回当前的状态 `status_`，状态为 `STATUS_MOVING` 或 `STATUS_STOP`。

## 5. `close_goal()`

```cpp
bool Robot::close_goal() { return fabs(target_pose_ - current_pose_) < 0.01; }
```

- **功能**：检查机器人是否接近目标位置（误差小于 `0.01`）。
- **返回值**：若接近目标位置，返回 `true`；否则返回 `false`。
- `fabs`是取**绝对值**

## 6. `stop_move()`

```cpp
void Robot::stop_move() { status_ = MoveRobot::Feedback::STATUS_STOP; }
```

- **功能**：停止机器人移动，将状态设置为 `STATUS_STOP`。

### 代码功能总结

- `Robot` 类模拟了一个简单的移动机器人。
- 支持设置目标位置、按步移动、检查是否到达目标等操作。
- 通过 `status_` 字段记录机器人的移动状态，反馈当前是否在移动或已停止。