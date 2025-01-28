在整一个绘制过程，为了能够实现一条代码实现。你**首先需要两样东西**

- launch 
  - 需要注意的是，应该在启动turtlesim_node之后暂停一段时间，然后再启动你的Client节点

- `colcon build && source install/setup.bash && ros2 launch turtle_rclcpp turtle_control_launch.py`

在节点的实现上，大致有两个方向

1. 手动赋予每一个笔画（建立每一个字符的笔画函数）
2. 根据已经存在的点，建立相应的圆滑曲线。

他们都需要关注线程的一致性，以防止线程不统一导致的弯折问题。



在实际的实现过程中，我原先是采用了第二中方案，但是因为点的数量不够，以及能力问题。我最终采取了第一种方案。

我在获取了师兄的solidworks的指导方案后，我很快就搭建起了`turtle_control`节点。



## 手动绘制

1. 一开始，我调用了kill和spawn对海龟进行了删除和重新更新的操作，使用`async_send_request`
2. 设立这个类的两个方法函数，起笔和落笔，一样是使用`async_send_request`
3. 定义笔画，并且将笔画函数放在初始化构造函数里面，他们能够顺序进行的原因是，我直接使用了`rclcpp::sleep_for(1s);`
   - 这种好处显而易见，能够更加精准的控制每一笔，不需要考虑各种回调函数带来的时间不统一问题。

具体是使用这种方式

```cpp
auto cmd_msg = geometry_msgs::msg::Twist();

        // 开始绘制
        start_draw(0, 162, 232, 6);

       // 第二步：直行 1.3 米
        cmd_msg.linear.x = 1.3;
        cmd_msg.angular.z = 0;
        cmd_pub_->publish(cmd_msg);
        rclcpp::sleep_for(1s);
	
	stop_draw();
```

然后说一下**旋转**

让机器人走一个圆弧，关键在于合理设置线速度（`linear.x`）和角速度（`angular.z`）的比值。圆弧的运动可以理解为机器人沿着一个固定的曲线轨迹行驶.

### 圆弧运动的原理
圆弧的运动实际上是在圆的路径上行驶，机器人前进的速度和旋转的角速度需要满足特定的比例关系，具体由下面的公式描述：

$$
\text{radius} = \frac{\text{linear speed}}{\text{angular speed}}
$$

- 通过调整机器人前进的线速度（`linear.x`）和角速度（`angular.z`），你可以控制机器人的圆弧半径（`radius`）。
- **半径公式**：  
  $$
  R = \frac{V_{\text{linear}}}{V_{\text{angular}}}
  $$

- **线速度公式**:

$$
\text{linear.x} = \text{angular.z} \times R
$$

比如说这个

```cpp
//转了一个半圆
        cmd_msg.linear.x = 3.14 * 1.2 / 2;
        cmd_msg.angular.z = 3.14;
        cmd_pub_->publish(cmd_msg);
        rclcpp::sleep_for(1s);
//转了一个四分之一圆
        cmd_msg.linear.x = 3.14 * 0.6 / 2 / 2;
        cmd_msg.angular.z = 1.57;
        cmd_pub_->publish(cmd_msg);
        rclcpp::sleep_for(1s);
```

在具体的实现中，使用/2来判断是半圆，/2/2来判断是四分之一圆。3.14这个线速度可以保证**1s**走完一个圆（单位圆）。1.2和0.6代表直径。

由r和angular可以推出linear(我们的半径和要旋转的度数是规定好的)。

## 使用复杂函数实现圆滑曲线

本人并未成功实现，期待有人能够完成。

失败的原因是：当时对于线程控制不熟悉，整个control节点达到200行还全是回调函数，搞得头皮发麻，时间紧迫于是赶紧放弃，只会简单的异步传输、线程捆绑和sleep，听说线程锁可以解决，但是不会用。

- 首先是要构建出一个自定义接口（struct也可以），然后是turtle类。它们能够帮助我实现target和current点的监管
- 使用csv保存路径（绝对路径）
- 使用turtle.cpp解释turtle方法，同时包含transform功能（将点转换为vector的path_，用于保存路径）以及更新target和currrent点。
- 在turtle_control里面实现turtle1的重新构造、turtle控制逻辑、文件的读取顺序。

### 妥协版本

在csv文件里添加类型--type

- 0 : 停笔直线

- 0 : 停笔直线
- 1 : 落笔直线 
- 2 : 落笔曲线
- 3 : 旋转方向（180）度

其中直线部分加入判断（第一次先旋转到指定度数，然后之后进行线性移动，在更新点后重新刷新）

在曲线部分，通过target点和current点的横纵坐标差，结合上述公式进行旋转。



在整个移动过程中，加入对linear速度的动态调控

```cpp
cmd_vel.linear.x = 0.5 * distance;
cmd_vel.angular.z = theta_ > 0 ? 0.3 : -0.3;
```

对于直线斜率的计算

```cpp
double angle_to_target =
                    atan2(target.y - current_pose_.y, target.x - current_pose_.x);
double angle_diff = angle_to_target - current_pose_.theta;
                if (std::abs(angle_diff) > 0.05) {
                    cmd_vel.angular.z = 0.5;
                } else {
                    is_rotating_ = false;
                }
/*你可以加入这个进行再次优化
if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
*/
```

对于曲线斜率的计算来源于theta =  (target_ y -current_y )/abs(target_ x -current_x )，它一定是1或者是-1，然后若是1则逆时针旋转标准圆弧，若是-1则顺时针.

我通过四个例子，似乎这个是成立的。然后整个的曲线只是圆弧，并不是一些特殊的平滑曲线。比如说整圆有4个点，半圆有3个点

## 严肃版本

需要大量的点，50hz的频率，越多越准确

csv不再需要type，只需要在control，里标定步一步不需要落笔即可

```cpp
double target_theta = atan2(target_.y - current_pose_.y, target_.x - current_pose_.x);
            double angle_diff = target_theta - current_pose_.theta;

            if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            geometry_msgs::msg::Twist cmd_vel;

            if (std::abs(angle_diff) < 0.1) {
                cmd_vel.linear.x = std::max(0.1, std::min(0.5 * distance, 0.5));
                cmd_vel.angular.z = 0.0;
            } else {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angle_diff;
            }
```

 ==以上的程序都没有正确实现，如果有机会，我会再尝试去实现==比如说通过仿真图像，得到大量的点之后再测试是否成立。



最后附上相关资料

## 接口

---

### `turtlesim/srv/SetPen.srv`

用于设置小海龟的笔属性。

**请求部分**：

```plaintext
uint8 r        # 红色通道值 (0-255)
uint8 g        # 绿色通道值 (0-255)
uint8 b        # 蓝色通道值 (0-255)
uint8 width    # 笔的宽度
uint8 off      # 是否关闭绘图 (0 表示开启, 1 表示关闭)
```

**响应部分**：
无返回内容（`---` 表示响应为空）

---

### `turtlesim/srv/Spawn.srv`

用于在指定位置生成一个新的小海龟。

**请求部分**：

```plaintext
float32 x      # x 位置坐标
float32 y      # y 位置坐标
float32 theta  # 朝向角度 (弧度)
string name    # 可选：小海龟的名字；若为空，将自动生成唯一名称
```

**响应部分**：

```plaintext
string name    # 生成的小海龟的名称（若未指定则返回自动生成的名称）
```

---

### `turtlesim/msg/Pose.msg`

包含小海龟的位置信息和速度信息。

```plaintext
float32 x                # x 位置坐标
float32 y                # y 位置坐标
float32 theta            # 方向角（弧度）

float32 linear_velocity  # 线速度
float32 angular_velocity # 角速度
```

---

### `geometry_msgs/msg/Twist.msg`

用于表示三维空间中的线速度和角速度。`cmd_vel` 话题

```plaintext
Vector3 linear           # 线速度向量
    float64 x            # x 方向的线速度
    float64 y            # y 方向的线速度
    float64 z            # z 方向的线速度

Vector3 angular          # 角速度向量
    float64 x            # 绕 x 轴的角速度
    float64 y            # 绕 y 轴的角速度
    float64 z            # 绕 z 轴的角速度
```

**用途**：  

- `linear` 部分设置线速度，控制机器人沿各轴的直线运动。对于小海龟，通常只用 `x` 方向的线速度（单位：m/s）。
- `angular` 部分设置角速度，控制机器人绕各轴的旋转。对于小海龟，通常只用 `z` 轴的角速度（单位：rad/s）。

**常用示例**：

- 将 `linear.x` 设置为正值可以让小海龟向前移动，负值让其向后移动。
- 将 `angular.z` 设置为正值可使小海龟顺时针旋转，负值让其逆时针旋转。

这样，`cmd_vel` 话题通过 `geometry_msgs/msg/Twist` 消息类型控制小海龟的运动。

---

