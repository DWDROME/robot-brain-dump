# 部署树莓派

[阅读理解：在树莓派上安装 ROS 2 是几个意思？](https://blog.csdn.net/qq_27865227/article/details/139264619)

需要去准备：

- 树莓派4b，128gb高性能sd卡、microhdmi-hdmi线、显卡欺骗器或实体显示器（可选）、读卡器、电池（充电宝）、鼠标键盘



使用官方Ubuntu 22.04lts镜像，使用**raspberry pi imager**刻录。

提前在custom中设定wifi密码以及ssh。



安装好系统之后。

使用`ping raspberrypi.local`找到同网段下树莓派的地址。

>如果不行可以尝试
>
>```bash
>ip addr show	#本机详细地址
>hostname -I		#本机局域网地址（inet）
>```
>
>```bash
>nmap -sn 192.168.0.0/24	#搜索同一网段下所有设备
>```
>
>当然更方便的是在手机中查看连接设备ip地址。



## 开启ssh

1. 在/boot中新建一个ssh文件（我造了也没用）

2. `sudo raspi-config`选择Interfacing Options再选择SSH

3. ```bash
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

我认为第三种最好用

#### **连接指令:**

输入【SSH 账号@IP地址】连接服务器,如：

```bash
ssh dwmini@192.168.246.101
```

在树莓派中运行图形程序，然后在本机中显示出图形程序（不好用）

```bash
ssh -X dwmini@192.168.246.101
```



## 开启vnc

使用Ubuntu无法直接用`sudo raspi-config`开启vnc服务

参考

- [Ubuntu 20.04 VNC 安装与设置](https://community.bwbot.org/topic/2917/ubuntu-20-04-vnc-%E5%AE%89%E8%A3%85%E4%B8%8E%E8%AE%BE%E7%BD%AE) 
- [todesk远程黑屏 todesk连接黑屏](https://blog.51cto.com/u_12897/9857473)



### 一些vnc操作

1. `ps -ef | grep vnc`

   - 显示出当前运行的vnc服务

     ```ini
     user     1234  5678  0 12:34 ?        00:00:05 Xtightvnc :1
     user     2345  6789  0 12:35 ?        00:00:04 Xtightvnc :2
     user     3456  7890  0 12:36 ?        00:00:03 x11vnc :0
     ```

     其中第一行是PID

     可以使用`kill`杀死（出现在多次注册vnc后发生冲突问题），因为我同时安装了vnc和todesk，当出现重启之后todesk反复重新连接网络的情况，使用``无法恢复，只能通过重装解决。后来，我杀死了所有vnc之后程序正常，所以则可能是因为占用冲突了。

     ```bash
     kill  1234		#正常杀死
     kill -9 1234	#强制杀死
     ```

2. `密码重置`

   ```bash
   sudo su
   rm /root/.vnc/passwd
   vncpasswd
   ```

   

## 我遇到的错误以及一些操作

1. **重复的vnc初始化**

   ![截图 2025-01-21 14-30-46](/home/dw/note/树莓派配置/vnc远程安装.assets/截图 2025-01-21 14-30-46.png)

   执行了`vncserver`之后，提示 X11 的显示器会话（`X1`, `X2`, `X3`）已经被占用。(它提示移除`/root/.vnc/passwd`可以解决)

   > `echo $?` 确认最后一个操作是否正确
   >
   > - **`0`**：表示命令成功执行。
   > - **`1`**：一般错误，命令未成功执行。
   > - **`127`**：命令未找到。

   

   接着我使用`vncserver -kill :1`删除显示器会话，但是显示找不到当前用户的对应显示器会话（root），这里应该是dwmini

2. **将复杂的启动文件修改简单，方便后续debug**

   `~/.vnc/xstartup`

   ```sh
   #!/bin/sh
   unset SESSION_MANAGER
   unset DBUS_SESSION_BUS观察输出发现服务启动失败，需要通过 systemctl status 和 journalctl 进一步分析具体错误。
   _ADDRESS
   exec startxfce4
   ```

   这是一个用于启动 XFCE 桌面环境的脚本。在这段代码中：

   - `#!/bin/sh` 指定使用的 shell 类型。
   - `unset SESSION_MANAGER` 和 `unset DBUS_SESSION_BUS_ADDRESS` 是为了清除当前会话的一些环境变量，确保新的会话不会受到先前会话设置的影响。
   - `exec startxfce4` 启动 XFCE4 桌面环境。

   

   ==记得赋予权限==

   ```bash
   chmod u+x xstartup
   # u 代表 user（文件的所有者），即文件的拥有者。
   # + 表示 添加 权限。
   # x 代表 execute 权限，即可执行权限。
   ```

   

3. **设置启动分辨率**

   ```text
   geometry=1920x1080
   dpi=96
   ```

   这是一个 `config` 配置文件，保存了 VNC 会话的显示设置：

   1. **`geometry=1920x1080`**:
      - 设置了 VNC 桌面的分辨率为 1920x1080（宽 x 高）。
   2. **`dpi=96`**:
      - 设置了显示的 DPI（每英寸像素数）。DPI 值影响屏幕上内容的清晰度和比例，通常默认使用 96 DPI。

   

4. **关闭防火墙**

   避免防火墙干扰，不确定是不是因为这个导致的。

   ```bash
   root@dwmini-mini:~/.vnc# systemctl stop firewalld
   Failed to stop firewalld.service: Unit firewalld.service not loaded.
   root@dwmini-mini:~/.vnc# ufw disable
   Firewall stopped and disabled on system startup
   root@dwmini-mini:~/.vnc# ufw status
   Status: inactive
   ```

   

5. **成功连接**

   **1. 启动 VNC 服务器**

   ```bash
   vncserver -localhost no && ss -ntlp
   ```

   - **`vncserver -localhost no`**：
     - 启动一个 VNC 服务。
     - `-localhost no` 参数允许远程客户端（非本地）访问 VNC 服务。
     - 系统分配一个显示编号（如 `:5`），并绑定到对应端口（5905）。
   - **`&& ss -ntlp`**：
     - 如果 VNC 启动成功，紧接着查看当前网络端口监听状态，确保 VNC 的服务端口正确开放。

   **a. 我出现的警告信息**

   ```text
   Warning: dwmini-mini:1 is taken because of /tmp/.X11-unix/X1
   Remove this file if there is no X server dwmini-mini:1
   ...
   Warning: dwmini-mini:4-lock
   Remove this file if there is no X server dwmini-mini:4
   ```

   ##### - **解释警告**

   - 每个 VNC 会话绑定一个显示编号（`:1`、`:2` 等），这些编号的会话信息存储在 `/tmp/.X11-unix/XN` 和 `/tmp/.X11-unix/XN-lock` 文件中。
   - 警告表明 `:1` 到 `:4` 的会话已被占用或其残留文件尚未清理。

   ##### **b. 解决方法**

   - 如果确认这些会话无效，可以删除对应的文件：

     ```bash
     rm -rf /tmp/.X11-unix/X* /tmp/.X11-unix/X*-lock
     ```

   **2. 启动成功**

   ![截图 2025-01-21 15-06-17](/home/dw/note/树莓派配置/vnc远程安装.assets/截图 2025-01-21 15-06-17.png)

   ```text
   New Xtigervnc server 'dwmini-mini:5 (root)' on port 5905 for display :5.
   Use xtigervncviewer -SecurityTypes VncAuth,TLSVnc -passwd /root/.vnc/passwd dwmini-mini:5 to connect to the VNC server.
   ```

   - 成功启动一个新的 VNC 服务，分配显示编号 `:5`。

   - 该会话绑定到端口 `5905`，这是 VNC 远程连接的端口。

   - 提示了连接命令：

     ```bash
     xtigervncviewer -SecurityTypes VncAuth,TLSVnc -passwd /root/.vnc/passwd dwmini-mini:5
     ```

   **3. 查看端口监听状态**

   ```bash
   ss -ntlp
   ```

   - 输出显示了系统当前监听的端口信息

6. 之后在vnc viewer中连接就行了。



# 图形化界面

在实际使用中，完整的Ubuntu界面太卡顿了，因而使用更轻的界面可能是更好的选择

==但我更推荐无图形==

1. **检查桌面环境安装状态**

```bash
echo $XDG_SESSION_DESKTOP
```

**2. 安装 Xfce 桌面环境**

运行以下命令安装轻量化桌面环境：

```bash
sudo apt update
sudo apt install xubuntu-desktop -y
```

**3. 切换默认显示管理器**

如果没有弹出提示，可以通过以下命令手动配置：

```bash
sudo dpkg-reconfigure lightdm
```

选择 `lightdm` 后，按 **Tab** 键切换到  并按下 **Enter** 确认。

**4. 重启系统并进入桌面**

完成安装后，重启系统：

```bash
sudo reboot
```

系统启动后，应该能直接进入轻量化的 Xfce 图形桌面环境。



# vnc开机自启

> 这是我觉得非常陌生的地方。
>
> 虽然最后能够成功，但是不知道是怎么实现的

[vnc安装和开机自启设置](https://blog.csdn.net/Sukura111/article/details/127982100)
