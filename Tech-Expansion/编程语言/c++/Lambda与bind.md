分别用 **Lambda 表达式** 和 **`std::bind()`** 来定义 `response_callback`

### 使用 Lambda 表达式

```cpp
// 使用 Lambda 表达式定义回调函数
using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
auto response_callback = [this](ServiceResponseFuture future) {
    RCLCPP_INFO(this->get_logger(), "响应: %ld", future.get()->sum);
};

// 发送请求
client_->async_send_request(request, response_callback);
```

在此示例中：
- `response_callback` 是一个捕获 `this` 指针的 Lambda 表达式，允许我们直接访问类的成员函数 `get_logger()`。
- 当请求完成时，它将调用 `response_callback`，从 `future` 中获取结果并输出。

---



### 使用 `std::bind()`

```cpp
// 使用 std::bind 定义回调函数
using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
auto response_callback = std::bind(&ClientNode::handle_response, this, std::placeholders::_1);

// 发送请求client_->async_send_request(request, response_callback);
```

在此示例中：
- `response_callback` 使用 `std::bind()` 绑定了类的成员函数 `handle_response`。
- `std::placeholders::_1` 用于占位 `ServiceResponseFuture` 参数，以便 `async_send_request` 在响应到达时传递给回调22函数。
### 回调函数 `handle_response`

### `当使用 `std::bind` 时，您需要定义回调函数 `handle_response`，例如：

```cpp
void handle_response(ServiceResponseFuture future) {
    RCLCPP_INFO(this->get_logger(), "响应: %ld", future.get()->sum);
}
```

### 总结

- **Lambda 表达式**：直接定义回调代码，较为简洁。
- **`std::bind()`**：将类成员函数绑定为回调，需要定义额外的回调函数，但对于复杂回调，便于代码重用和清晰组织。

