这篇教程基于（[小鱼ros](https://fishros.com/)）以及古月居ros2入门教程
是个人做的笔记，希望对你有所帮助
@[TOC]( 目录)

# 综述
## 服务通信介绍

```mermaid
graph LR
A[服务端]--发送响应-->B[客户端]
B--发送请求-->A
```
*这是一个==服务器-客户端模型==也可以被称之为请求-响应模型，（c/s模型）*

与**话题**（适用于单向或大量的数据传输）不同的是，**服务**是双向的，多次的，所以说就需要**回调函数**出场了。

 - [ ] 异步通信机制
 - [ ] 服务端唯一，客户端不唯一
 - [ ] srv文件定义请求和应答数据结构

## 体验service
声明一个服务端

```cpp
ros2 run examples_rclpy_minimal_service service
```
我们需要看看现在有什么service在启用
```cpp
ros2 service list //add_two_ints就是
```

手动调用这个服务端
```cpp
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```
## service 的一些基本操作
```cpp
ros2 service  //help
ros2 service list  //当前有哪些服务可被使用?
ros2 service type /service_name  //服务的接口类型(srv中的数据结构)
ros2 service find /interface_name  //接口所属的服务（与上一个相反）
ros2 service info /service_name  //服务的基本信息
ros2 service ow /service_name  //服务的带宽
ros2 service echo /service_name  //服务正在传输的数据
ros2 service call <service_name> <service_type> "{data}" //调用服务
ros2 interface show <service_type>  //服务类型的具体结构

//<>和/表示一个意思。
//ros2 interface show service_type：查看特定服务类型的结构。
//ros2 service find interface_name：查找使用特定服务类型的所有服务。
```
## 服务的程序流程
| 创建客户端的程序流程   | 创建服务端的程序流程     |
| ---------------------- | ------------------------ |
| 编程节点初始化         | 编程接口初始化           |
| 创建节点并初始化       | 创建节点并初始化         |
| 创建**客户端**对象     | 创建**服务端**对象       |
| 创建并发送请求数据     | 通过**回调函数**进行服务 |
| 等待**服务端**应答数据 | 等待**客户端**应答数据   |
| 销毁节点并关闭接口     | 销毁节点并关闭接口       |
--------------------------------------------------------------------------------------------------------------------------------------------
## 相关配置（RCLCPP）
>这里详细请看小鱼的教程。(那里有完整教学，我这里只做一些补充)

我们需要建立一个工作空间 然后通过 **pkg**来建立相关环境
这里我们有两个依赖 ==example_interfaces==以及==rclcpp==
```cpp
ros2 pkg create example_topic_rclcpp --build-type ament_cmake --dependencies rclcpp example_interfaces
```
代码末的 rclcpp example_interfaces，接在参数--dependencies后的依赖，可以帮助我们自动写好**CMakeLists.txt**以及**package**内的相关配置

==关于source== 你也可以通过添加到全局变量里面，这样每次启动就不需要再写一次了。

编译好的文件，不再src里面，而是在install里面，自己去找一下。

--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------
# 服务端实现
## 代码

```cpp
#include"rclcpp/rclcpp.hpp"
#include"example_interfaces/srv/add_two_ints.hpp"

class ServiceServer01 : public rclcpp::Node{
    public:
        ServiceServer01(const std::string name):Node(name){
            RCLCPP_INFO(this->get_logger(),"the server [%s] is executing successfully!",name.c_str());
            //create the service
            add_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                                "add_two_ints_srv",
                                std::bind(&ServiceServer01::handle_add_two_ints,this,
                                std::placeholders::_1,std::placeholders::_2)
            );
        }

    private:
    /// declare a server
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_ints_server_;

    //the callback funtion producing the record service
    void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, 
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
        ){
            RCLCPP_INFO(this->get_logger(),"received a:%ld  b:%ld",request->a,request->b);
            response->sum = request->a + request->b;
        };
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ServiceServer01>("service_server_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

--------------------------------------------------------------------------------------------------------------------------------------------
## server的建立
```cpp
add_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                                "add_two_ints_srv",
                                std::bind(&ServiceServer01::handle_add_two_ints,this,
                                std::placeholders::_1,std::placeholders::_2)
            );
```
很复杂对吗，我们来一步一步分析。
首先打开**ros2 reference**  ![create_serve()的类介绍](https://i-blog.csdnimg.cn/direct/e51f7001a5ea4a1cbd64b446ba9e3c7d.png#pic_center)很复杂不是么，我们重点观察两个部分，一个是参数部分，一个是前缀

 1. 参数部分
+ service_name  这个服务的名称是什么，要和client的保持相同
+ Callback  回调函数，用std::bind()进行替换，记得加占位符(回调函数有几个参数，占位符就几个)
+ qos_profile  要占用多少带宽(服务质量)的文件，这里有默认值
+ group 调用服务的回调组 ，有默认值
		
 2. 前缀部分 
+ ServiceT 消息接口 ```example_interfaces::srv::AddTwoInts```
+ 类型```rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr```
+ 所属方法 ```rclcpp::Node::create_service```

你肯定会疑惑，**add_ints_server_**难道不应该是由模板类**rclcpp::Service < ServiceT >**（继承自**ServiceBase**,为特殊的数据类型提供了特定的功能）
即：
```rclcpp::Service<example_interfaces::srv::AddTwoInts>```

那为什么在这里，前缀却是?```rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr```

### 智能指针
#### Client_到底是什么？
```cpp
//public
add_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                                "add_two_ints_srv",
                                std::bind(&ServiceServer01::handle_add_two_ints,this,
                                std::placeholders::_1,std::placeholders::_2)
            );
//private
  
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_ints_server_;
```
在最初，我认为，这是一个重载，在public中定义的是一个```rclcpp::Service<example_interfaces::srv::AddTwoInts>```类型的对象,在private中定义的是```rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr```类型的**传统指针**，指向同名的Client_对象

经过两天的思考后！事实上，这错的非常离谱！

原因是我不会阅读api，当我在反复（非常艰难啊）地阅读纯英文api之后，我终于看见了那文档下面的==Returns== 部分  ！！！

**“Spared pointer to the created service”**

就是说，**this**这个类里面的方法**rclcpp::create_service**，返回的是一个指向 **<example_interfaces::srv::AddTwoInts>** 类型的对象的指针
然后再通过  _=_  赋值给 **Client_**

==那么==  **Client_** 到底是什么？
一个**智能指针**，具体来说，一个**共享智能指针(shared_ptr)**
类型是
```rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr```
更加根本是（我们不通过这个来定义）
```std::shared_ptr<rclcpp::Client<example_interfaces::srv::AddTwoInts>>```这个类型的实例

在==public==中，我们声明了一个对象（不是client_）,然后将一个指向这个对象的指针**赋值**给了client_,完成了对client的基本声明
在==private==中，我们定义了一个**指向客户端对象的指针**


| 类型         | 特点                                                   |
| ------------ | ------------------------------------------------------ |
| 传统原生指针 | 直接指向某个对象的内存地址                             |
| 共享智能指针 | 一种智能指针，自动管理对象的生命周期，避免手动内存控制 |
|              | 同时具有**引用技术**和**自动释放内存**的优势，更为安全 |


详细请见[现代 C++：一文读懂智能指针](https://zhuanlan.zhihu.com/p/150555165)  //共包含三种智能指针的介绍

#### 为和要分public 和private?
+ ==信息隐藏==  
  * 在**public**部分提供类型声明， 让外部代码能够访问类的接口；
  * 在**private**部分实现，是的外部代码无法直接操作或修改它
+ ==生命周期管理== 
  * 在**private**内定义，确保生命周期由内部控制 
--------------------------------------------------------------------------------------------------------------------------------------------

## 回调函数
### 实现代码
```cpp
//the callback funtion producing the record service
    void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, 
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
        ){
            RCLCPP_INFO(this->get_logger(),"received a:%ld  b:%ld",request->a,request->b);
            response->sum = request->a + request->b;
        };
```
### 分析
#### 又长又臭的参数
定义request来接受参数，定义response 来赋值到sum之中
所以说这两个又长又臭的东西到底是什么？？？
```const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request```
```std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response```
首先**std::shared_ptr**说明这是一个智能指针
然后中间的```< example_interfaces::srv::AddTwoInts::Response > ```是我们的老朋友，等等，==Response?==

我们通过以下命令，去查找AddTwoInts包含的东西
```cpp
ros2 interface show example_interfaces/srv/AddTwoInts 
```
结果
```cpp
int64 a
int64 b
---
int64 sum
```
==Request 是上部分  Response 是下部分==
使用上面的 **interface show**所展示的是 .srv文件下的实际句段，而Request和Response这两个**结构体**是由 ROS 2 的生成工具 **rosidl** 自动创建的

--------------------------------------------------------------------------------------------------------------------------------------------
#### 服务端回调函数对异步传输的贡献
```cpp
std::bind(&ServiceServer01::handle_add_two_ints,this,
                                std::placeholders::_1,std::placeholders::_2)
```

* **std::bind 的作用：**

    + std::bind 用于将成员函数（handle_add_two_ints）绑定到当前对象（this），并创建一个可调用对象。
    + std::bind 将 handle_add_two_ints 的前两个参数绑定到占位符 _1 和 _2，表示当这个绑定函数被调用时，这两个占位符将被替换成实际的参数。
    

==具体的我放在最后讲==
			
--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------
# 客户端实现

*关于client_的智能指针实现，我不再赘述*
*本部分将着重讲  **异步传输实现**  以及  **recall_result处理***
## 代码
```cpp
//service_client_01.cpp
#include"rclcpp/rclcpp.hpp"
#include"example_interfaces/srv/add_two_ints.hpp"

class ServiceClient01 : public rclcpp::Node{
    public:
        ServiceClient01(const std::string name):Node(name){
            RCLCPP_INFO(this->get_logger(),"the client [%s] is executing successfully!",name.c_str());
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
                "add_two_ints_srv");
        }

        void send_request(int a ,int b){
            RCLCPP_INFO (this->get_logger(),"calculate %d and %d ",a,b);
            
            //1. Wait for the server to become available
            while(!client_->wait_for_service(std::chrono::seconds(1))){
                //scan the condition of rclcpp in waiting time
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(),"The waiting process for the service was interrupted...");
                    return ;
                }
                RCLCPP_INFO(this->get_logger(),"waiting for the service");
            }

            //2.Construct the request message
            auto request = 
                std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
            request->a = a;
            request->b = b;

            //3.Send an asynchronous request and set up the response callback.               
            client_->async_send_request(
                request,
                std::bind(&ServiceClient01::result_callback, 
                            this,
                            std::placeholders::_1)
            );
        };
    
   private:
        //declare the client
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

        void result_callback(
            rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future){
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(),"the calculate result is %ld",response->sum);
        }

};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ServiceClient01>("service_client_01");
    
    node->send_request(5,6);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
--------------------------------------------------------------------------------------------------------------------------------------------
## 分析
### client_的建立
```cpp
client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
                "add_two_ints_srv");
```
![create_client的api](https://i-blog.csdnimg.cn/direct/b9155ce867a2475aad58147937b2fac9.png#pic_center)

--------------------------------------------------------------------------------------------------------------------------------------------
###  等待数据传输（send_request）
```cpp
void send_request(int a ,int b){
            RCLCPP_INFO (this->get_logger(),"calculate %d and %d ",a,b);
            
            //1. Wait for the server to become available
            while(!client_->wait_for_service(std::chrono::seconds(1))){
                //scan the condition of rclcpp in waiting time
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(),"The waiting process for the service was interrupted...");
                    return ;
                }
                RCLCPP_INFO(this->get_logger(),"waiting for the service");
            }
```
![wait_for_service](https://i-blog.csdnimg.cn/direct/5209cb3b7a4d4818a0f9ca2e4208d303.png#pic_center)
+ **wait_for_service**是一个**bool类型**的 、**rclcpp::ClientBase** （*一个基类  提供通用的客户端功能* ） 的一个方法，用于阻塞进程，直到接收到服务端消息
  * ***std_chrono::duraton()* **是一个**c++11**中的时间对象，代表一个指定长度的持续时间
  		 	_ 超过1s,**wait_for_service()** 将返回**false**,表示服务不在线
  		 	_ 1s以内,**wait_for_service()** 将返回**true**,表示服务在线
+ **Client_** 是模板类 **rclcpp::Client< ServiceT >** 的**智能指针**,其继承自**ClientBase**,为特殊的数据类型提供了特定的功能，因继承，所以**Client_** 可以直接使用**wait_for_service**这个方法。
+ **rclcpp::ok()**
	* 在每次循环中，代码会调用 rclcpp::ok() 检查节点的状态。
	* 如果节点已被终止（例如用户在命令行按下 Ctrl+C ），rclcpp::ok() 将返回 false。
	* 如果节点不再有效，代码输出一条错误信息并退出 send_request 函数，停止等待。
---
### 异步传输请求
#### 请求的实例对象(request)
```cpp
auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
```
==作用==
为接下来的异步传输请求函数，提供一个可传输的实例对象

==类型==  (*只是猜测，需要以后再具体考究*)
[1] . ```<example_interfaces::srv::AddTwoInts_Request>```
心细的朋友应该不难发现，这和上一节 （server）里的有些区别
[2] . ```<example_interfaces::srv::AddTwoInts::Response>```
在这个小鱼的实例里面，他们代表的是同一个东西，但是有细节上的区别

**rosidl**基于**example_interfaces::srv::AddTwoInts**自动生成一个叫做 ```< example_interfaces::srv::AddTwoInts_Request >```的结构体（==这是实际生成的==）

为了 **表达的简洁** 和 **兼容性(防止命名重复)** 以及 **可读性**
采用**别名**  ```<example_interfaces::srv::AddTwoInts::Response>```

#### 本体
```cpp
client_->async_send_request(
                request,
                std::bind(&ServiceClient01::result_callback, 
                            this,
                            std::placeholders::_1)
            );
```
![async_send_request](https://i-blog.csdnimg.cn/direct/dfc4add0f7ac4ddaa401dec09d89b61d.png#pic_center)==这里我们用的是第二个== 也是最常用的版本
*async_send_requests是一个方法*
+ request  名称
+ callbackT  回调函数
+ 前缀类别 rclcpp::Client< ServiceT >

然后不知道你注意到没有，async_send_request()的第一个参数SharedFuture
这表明它返回的类型是	```SharedFuture```
在注册之后，将会触发回调函数
若我们需要处理关于**async_send_request()** 中的内容，就需要一个SharedFuture类的参数与**async_send_request()** 相对应。

==具体步骤是==
1. 返回的 future 对象：**async_send_request()** 返回一个**SharedFuture结构体对象**(本代码里并没有保存它)

2. 回调函数触发：当服务端响应后，ROS 2 的事件循环会自动调用 **result_callback**。

3. 回调函数处理响应： 在 **result_callback** 中，使用 **result_future.get()** 获取响应结果：

#### 为何 server 需要保存 create_service 的返回值?
```cpp
add_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints_srv",
    std::bind(&ServiceServer01::handle_add_two_ints, this, std::placeholders::_1, std::placeholders::_2)
);
```
在服务器的智能节点定义中，你可以发现这一点，我们保存了这一返回值(和客户端的处理方式不同)
那么想想**add_ints_server**的类别 ***对!*** **std::shared_ptr** 一种**共享智能指针**，它指向一个```rclcpp::Client<example_interfaces::srv::AddTwoInts>```的实际对象，如果不保存，那么这个对象在**create_service**返回值后，就会马上销毁。
于是乎 **add_ints_server** 有了另一个名字 ***服务的句柄***（即服务对象的共享指针）

##### 句柄？
- **句柄（Handle）** 是一种**引用**，用于管理和操作资源，比如文件、服务、内存等。在 ROS 2 中，句柄通常是一个**指针**（例如 std::shared_ptr），用于引用创建的服务或客户端对象。持有句柄可以确保资源的生命周期管理和有效访问。
#### 为何 client 不需要保存 async_send_request 的返回值？
```cpp
client_->async_send_request(
                request,
                std::bind(&ServiceClient01::result_callback, 
                            this,
                            std::placeholders::_1)
            );
```
1. **SharedFuture** 只是一个**异步结果的占位符**：它表示将来可以获取到的结果，所以不影响整个客户端程序的运行。
2. **回调函数自动处理结果**：我们已经绑定了回调函数 result_callback，当响应到达时，ROS 2 会自动调用回调函数来处理结果。
3. **无需持有 SharedFuture**：**SharedFuture** 只是一种**临时性**的对象，用于等待异步响应，而不像服务对象需要在整个节点生命周期内保持有效。只要在回调中访问 **SharedFuture** 的结果即可。

---
### result_callback()  --回调函数
```cpp
void result_callback(
            rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future){
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(),"the calculate result is %ld",response->sum);
        }
```
#### SharedFuture 是什么?
```rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture```

显然关于```rclcpp::Client<example_interfaces::srv::AddTwoInts>```我们已经很熟悉了


那**SharedFuture**是什么？
- 是由 ROS 2 的 ROSIDL 工具自动生成的 **服务请求结构体**
- **SharedFuture** 是 **rclcpp::Client** 类中的一个类型定义(一种便捷工具)，用于处理服务请求的异步响应。
- 当您调用 **async_send_request** 时，它会返回一个 **std::shared_future** 对象，这个对象可以用来在响应到达时获取结果。
- 具体来说，**rclcpp::Client< example_interfaces::srv::AddTwoInts >::SharedFuture** 是 **std::shared_future** 的别名，用于简化代码阅读和书写。

**result_future.get()** ：在回调函数中，**get()** 被调用来提取服务端的响应数据。

##### ==关于SharedFuture对象的一些操作==
```cpp
result_future.get()：获取异步结果（可以多次调用）。
result_future.valid()：检查 shared_future 是否有效。
result_future.wait()：等待异步操作完成，但不获取结果。
result_future.wait_for(duration)：等待指定的时间段，直到结果可用。
result_future.wait_until(time_point)：等待到指定的时间点，直到结果可用.
```
---
---
# 背后的机制
  ROS2这种回调触发方式 基于DDS(Data Distribution Service)的中间件
--
DDS 提供了发布-订阅模型 和服务调用支持
--

## 以服务端举例

1. ***服务的创建和注册***
	+ `add_ints_servers_`的赋值部分，创建了一个名为`"add_two_ints_srv"`的服务名
	+ `std::binds`将`handle_add_two_i`nts与`"add_two_ints_srv"`绑定。即：受到请求时(client端通过服务名)，调用回调函数
2. ***ROS2事件驱动机制***
	 **即事件驱动的异步通信机制**  **->** 意味着所有服务端、客户端、发布者、订阅者等都通过事件和回调来相应消息与请求
	+ **事件监听** : `spin(node)` 是服务端监听等待来的客户端的请求
	+ **事件触发** :  一旦客户端发布请求，**ROS2网络**会检测该事件，并将请求数据传递给相应的服务节点
3. ***回调函数的触发***
	+ **请求参数传递** : 将请求数据封装到 `request`对象中并将其作为参数 `_1` 传递到**回调函数**（`handle_add_two_ints`）中
	+ **响应处理** : 结果保存在 `response` 中
	+ **自动发送响应** : **回调函数**结束后，将 `response` 返回到 **客户端** 处
4. ***事件循环(spin)***

---
## 以客户端端举例


1. ***客户端的创建和请求发送***
   + `client_` 的赋值部分，创建了一个名为 `"add_two_ints_srv"` 的服务**客户端**。
   + `async_send_request` 发送异步请求到 `"add_two_ints_srv"` 服务，绑定回调函数 `result_callback`，即：**服务端**响应完成后自动调用**回调函数**。
 2. ***ROS 2 事件驱动机制***
      **事件驱动的异步通信机制**：ROS 2 使用事件驱动的异步通信机制，服务端、客户端、发布者、订阅者等都通过事件和回调来响应消息和请求。
   + **事件监听**：客户端进入事件循环，通过 `spin(node)` 等待服务端的响应。
   + **事件触发**：服务端完成计算并返回响应时，**ROS2网络**会检测到该事件，并自动将响应数据传递给**客户端**的回调函数。
3. **回调函数的触发**
   + **响应参数传递**：服务端返回的数据被封装到 `SharedFuture` 对象中，并传递给回调函数 `result_callback`。
   + **结果提取**：在 `result_callback` 中，通过 `result_future.get()` 获取服务端的响应结果。
   + **结果处理**：回调函数内处理或显示响应数据，例如，将结果打印到日志中。
4. **事件循环 (spin)**
   + `spin(node)` 保持客户端的事件循环运行，等待服务端的响应事件并处理。
   + **持续监听和响应**：`spin` 保持节点活跃，监听事件，并在响应到达时自动调用绑定的回调函数进行处理。
   
---
---
# 总结

确实，小鱼的这个服务节点实现是比较简单的。
但是不可否认的是，当我们将每一个细节扣下去，会发现非常多值得学习的内容
无论是基本的 依赖、方法、泛型算法的知识
还是 更深的 不同位置声明、封装，对程序的可移植性、安全性的影响。

一个小细节可以影响到整个程序的性能
--
比如说序列以及反序列的数据结构建立。