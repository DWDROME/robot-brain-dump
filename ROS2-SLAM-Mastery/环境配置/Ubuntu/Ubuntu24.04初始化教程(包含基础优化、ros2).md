csdn博客地址：

- [Ubuntu24.04初始化教程(包含基础优化、ros2)](https://blog.csdn.net/2402_87385120/article/details/144096726?spm=1001.2014.3001.5501)

将会不断更新。但是所有都是基础且必要的操作。

- 为重装系统之后的环境配置提供便捷信息来源。  
- 记录一些错误的解决方案。

[TOC]

# 构建系统

参照经典教程  
[Windows11 安装 Ubuntu 避坑指南](https://www.bilibili.com/video/BV1Cc41127B9/?spm_id_from=333.337.search-card.all.click&vd_source=70726c0c4e3073ec08e9566e626253af)  
[Windows 和 Ubuntu 双系统的安装和卸载](https://www.bilibili.com/video/BV1554y1n7zv?spm_id_from=333.788.recommend_more_video.0&vd_source=70726c0c4e3073ec08e9566e626253af)  

一些值得注意的点：  
如果是 GPT 的磁盘（我使用这个）  
同时使用双硬盘的笔记本  

那么在 Ubuntu 24.04 中，无需要创建一个 EFI 分区。  
当创建根挂载点之后，将自动生成一个 FAT32 格式的引导分区。

**推荐使用最简配置**  

其他的正常操作即可。非常简单。

---

# 建立系统备份

## **Timeshift: 系统快照和备份工具**

**Timeshift** 是一款用于 Linux 的强大工具，它可以创建系统快照并在需要时还原系统到指定的时间点，非常适合在进行高风险操作（如删除关键系统组件、更新系统等）之前使用。

---

## **安装 Timeshift**

```bash
sudo nala update
sudo nala install timeshift
```

---

## **使用 Timeshift 创建快照**

1. **启动 Timeshift**：

   - 使用图形界面：

     ```bash
     sudo timeshift-gtk
     ```

   - 使用终端界面：

     ```bash
     sudo timeshift
     ```

2. **选择快照类型**：

   - **RSYNC**（推荐）：适用于大多数场景，它使用文件同步的方式创建快照。
   - **BTRFS**：适用于 Btrfs 文件系统。

3. **选择快照存储位置**：

   - 选择存储快照的磁盘分区（建议使用外部磁盘或与系统分区不同的磁盘分区）。

4. **创建快照**：

   - 单击 **"Create"** 按钮即可开始创建快照。

   - 或者在终端中运行：

     ```bash
     sudo timeshift --create --comments "Before removing Snap"
     ```

---

## **还原快照**

如果删除关键组件后系统出现问题，可以通过以下步骤还原快照：

1. 启动 Timeshift：

   - 图形界面：

     ```bash
     sudo timeshift-gtk
     ```

   - 终端：

     ```bash
     sudo timeshift --restore
     ```

2. 选择要还原的快照。

3. 按照提示操作，完成系统还原。

---

## **自动创建快照**

为了避免忘记创建快照，您可以设置 Timeshift 自动创建快照：

1. 启动 Timeshift。
2. 转到 **Settings > Schedule**，启用定时快照。
3. 根据需求选择创建快照的频率（如每天、每周等）。



# 最基本配置

## 时间同步

1. **更新软件包列表**

   ```bash
   sudo apt update
   ```

2. **安装时间同步工具**

   ```bash
   sudo apt install ntpdate
   ```

3. **同步时间**

   ```bash
   sudo ntpdate time.windows.com
   ```

4. **设置硬件时钟为本地时间**

   ```bash
   sudo timedatectl set-local-rtc 1 --adjust-system-clock
   ```

5. **验证同步状态**

   ```bash
   timedatectl status
   ```

## 换源

- 你可以手动操作，但容易出错。

- 我曾使用 Fishros 脚本更换软件源，命令如下：

   ```zsh
  wget http://fishros.com/install -O fishros && . fishros 
  ```

  **注意**：Fishros 的一键换源可能会导致依赖问题且速度较慢，推荐手动更换源。

### 官方源

在软件更新中，选择 Ubuntu 官方源是更好的选择。比如说阿里源。

1. 在 Ubuntu 24.04 中，源地址配置文件发生了变化，不再使用以前的 `sources.list` 文件，而是以下文件：
    **`/etc/apt/sources.list.d/ubuntu.sources`**

2. 备份源配置文件：

    ```zsh
    sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
    ```

3. 编辑源配置文件：

   ```zsh
   sudo vim /etc/apt/sources.list.d/ubuntu.sources
   ```

4. 替换为阿里云源（以下配置中增加了 `suites` 部分）：

   ```zsh
   Types: deb
   URIs: http://mirrors.aliyun.com/ubuntu/
   Suites: noble noble-security noble-updates noble-proposed noble-backports
   Components: main restricted universe multiverse
   Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
   ```

5. 注释掉系统默认的官方源。

6. 更新系统：

    ```zsh
   sudo apt-get update && sudo apt-get upgrade
   ```

其他参考换源方法：

- [Ubuntu24.04换源方法（新版源更换方式，包含Arm64）](https://blog.csdn.net/qq_37344125/article/details/138841559)
- [Ubuntu 24 换国内源及原理（阿里源）](https://blog.csdn.net/qq_46444918/article/details/138772667)

### 软件源

目前尚未解决的源问题包括：

- `typora` 源的支持问题。

通常，在安装软件时会附带软件源的配置教程。

---

#### ROS 2 的软件源

使用官方文档中提供的源即可，虽然某些情况下可能无法连接。此时推荐使用清华源（阿里源已被废弃）。

帮助文档：[ROS2 软件仓库](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)

1. 下载 ROS 的 GPG Key：

    ```zsh
   sudo apt install curl gnupg2
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

   如果下载失败，将文件转移到 `/usr/share/keyrings` 并命名为 `ros-archive-keyring.gpg`：

   ```zsh
   sudo mv ~/Downloads/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
   ```

2. 替换源配置：
    在 `/etc/apt/sources.list.d/ros2.list` 中使用清华源代替官方源：

   ```zsh
   deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main
   ```

3. 然后更新系统：

   ```zsh
   sudo apt-get update && sudo apt-get upgrade
   ```

   ---

   # 软件配置

   ## 打开新世界大门

   无

   ---

   ## 谷歌浏览器

   建议优先安装谷歌浏览器，因为后续会删除所有 `snap` 应用。
    直接前往 [谷歌官网](https://www.google.com/chrome/) 下载适配的安装包。我个人是用谷歌官方的稳定版本，安装过程非常简单。

   ---

   ## 星火应用商城

   如果需要安装国产软件，星火应用商城是个不错的选择。我通过它安装了微信、QQ 音乐和网易邮箱等应用，体验很好，推荐使用。

   ---

   ## 更换输入法

   在 Ubuntu 系统中，输入法的选择和配置是一个常见的问题，推荐以下教程：

   - **废弃教程**：~~[在Ubuntu22.04上安装rime中文输入法的基本教程](https://blog.csdn.net/2402_87385120/article/details/143468103?spm=1001.2014.3001.5502)~~
      （内容过时，不建议参考。）
   - **推荐教程**：[安装 Fcitx5 输入框架和输入法自动部署脚本（来自 Mark24）- Ubuntu 通用](https://blog.csdn.net/2402_87385120/article/details/144148411?fromshare=blogdetail&sharetype=blogdetail&sharerId=144148411&sharerefer=PC&sharesource=2402_87385120&sharefrom=from_link)

   建议直接参考第二篇文章，步骤清晰，适配范围广，能快速完成输入法的安装和配置。

   ---

   ## 安装 VS Code

   到 [VS Code 官网](https://code.visualstudio.com/) 下载适配的安装包完成安装。同时推荐我的一些配置教程：

   - [VS Code 基本配置 - 基于 clang-format](https://github.com/DWDROME/VSC-environment-custom)
   - [VS Code 的 C/C++ 环境配置（包含 clang-format 配置）](https://blog.csdn.net/2402_87385120/article/details/143581830?fromshare=blogdetail&sharetype=blogdetail&sharerId=143581830&sharerefer=PC&sharesource=2402_87385120&sharefrom=from_link)
   - [VS Code 插件推荐](https://blog.csdn.net/2402_87385120/article/details/143589206?fromshare=blogdetail&sharetype=blogdetail&sharerId=143589206&sharerefer=PC&sharesource=2402_87385120&sharefrom=from_link)

   另外，在使用 clang-format 前，别忘了先安装必要的依赖：

   ```bash
   sudo apt install clang-format
   ```

   以上教程中包含了详细的配置步骤，可以根据需要选择参考。

---

# 完全删除 Snap

## 删除 Snap

**注意顺序**：在禁用服务之前，建议先禁用 `socket`，否则可能会出错。

### 禁用开机启动

使用以下命令禁用 Snap 的相关服务，防止其在系统启动时自动加载：

```bash
sudo systemctl disable snapd.socket
sudo systemctl disable snapd.service
sudo systemctl disable snapd.seeded.service
```

### 删除 Snap 软件

1. **检查已安装的 Snap 应用**  
   使用以下命令列出当前系统中所有的 Snap 应用：  

   ```bash
   snap list
   ```

2. **逐一删除 Snap 应用**  
   按顺序删除 Snap 应用，建议先卸载 Snap 商店，然后是其他应用：  

   ```bash
   sudo snap remove snap-store
   sudo snap remove firefox
   sudo snap remove gnome-42-2204 
   sudo snap remove core20
   sudo snap remove bare
   ```

3. **终止运行中的 Snap 进程**  
   如果仍然看到 `/usr/lib/snapd/snapd` 等进程，可以使用以下命令终止：  

   ```bash
   sudo killall snapd
   ```

4. **移除 Snap 服务及相关残留**  
   完全清理 Snap 的残留服务和文件：  

   ```bash
   sudo apt purge snapd
   sudo rm -rf /snap /var/snap /var/lib/snapd ~/snap
   ```

---

## 彻底删除 Snap

1. **手动清理残留文件**  
   如果之前的步骤没有完全清除，可以手动删除 Snap 的相关目录：  

   ```bash
   sudo rm -rf /snap
   sudo rm -rf /var/snap
   sudo rm -rf /var/lib/snapd
   sudo rm -rf ~/snap
   ```

2. **验证 Snap 是否完全删除**  
   使用以下命令检查系统中是否还有 Snap 的痕迹：  

   ```bash
   which snap
   ```

   如果没有输出，说明 Snap 已经被完全卸载。

---

## 禁止重新安装 Snap

为防止 Snap 被重新安装，可以通过 APT 配置来禁用相关依赖项：

1. 创建一个配置文件：  

   ```bash
   sudo vim /etc/apt/preferences.d/nosnap.pref
   ```

2. 在文件中输入以下内容：  

   ```bash
   Package: snapd
   Pin: release a=*
   Pin-Priority: -10
   ```

---

## 如果需要重新安装 Snap

如果后续需要重新安装 Snap，可以按照以下步骤操作：

1. **重新安装 Snap 包管理器**  

   ```bash
   sudo apt update
   sudo apt install snapd
   ```

2. **启用 Snap 服务**  

   ```bash
   sudo systemctl enable --now snapd.socket
   ```

3. **检查 Snap 是否正常工作**  

   ```bash
   snap version
   ```

   如果输出版本号，说明 Snap 已成功安装并运行。

---

# **安装 Nala**

nala比apt更快，而且有更多动画，看的很爽.



1. **添加 Nala 的 PPA**  
   首先需要添加 Nala 的软件源，并更新包管理器索引：  

   ```bash
   sudo add-apt-repository ppa:volian/nala
   sudo apt update
   ```

2. **安装 Nala**  
   使用以下命令安装 Nala：  

   ```bash
   sudo apt install nala
   ```

## 基本命令

Nala 的命令与 `apt` 类似.

- **更新软件包索引**  

  ```bash
  sudo nala update
  ```

- **安装软件包**  

  ```bash
  sudo nala install <package_name>
  ```

- **升级系统**  

  ```bash
  sudo nala upgrade
  ```

- **删除软件包**  

  ```bash
  sudo nala remove <package_name>
  ```

- **查看安装记录**

  ```bash
  sudo nala history
  ```

- **检查损坏并修复**

  ```bash
  sudo nala fix
  ```

---

明白了，我会完全按照您给出的格式进行输出，以下是调整后的内容：

---

## **Fastfetch: 系统信息显示工具**

Fastfetch 是一个用于显示系统信息的工具。

---

### **安装 Fastfetch**

1. **使用系统包管理器安装**  
   适用于 Ubuntu 或基于 Debian 的系统：  

   ```bash
   sudo apt update
   sudo apt install fastfetch
   ```

2. **从源码安装**  
   如果在官方仓库中找不到 Fastfetch，可以通过源码进行安装：

   - 克隆 GitHub 仓库：  

     ```bash
     git clone https://github.com/SnowflakeMC/fastfetch.git
     ```

   - 进入仓库目录：  

     ```bash
     cd fastfetch
     ```

   - 安装：  

     ```bash
     sudo make install
     ```

---

### **使用 Fastfetch**

1. **显示系统信息**  
   运行以下命令直接查看系统信息：  

   ```bash
   fastfetch
   ```

2. **常用选项**  

   - 禁用 ASCII 图形，仅显示文字信息：  

     ```bash
     fastfetch --no-ascii
     ```

   - 自定义配置文件路径：  

     ```bash
     fastfetch --config /path/to/config.conf
     ```

3. **修改配置**  
   配置文件通常位于 `~/.config/fastfetch/config.conf`，可以通过编辑该文件来自定义显示内容，比如隐藏特定信息或调整颜色主题。

---

## Conda: Miniconda

### 简介

我之后会使用 Anaconda，但 Miniconda 的操作基本一致。以下是 Miniconda 的安装和配置步骤。

---

### 安装步骤

1. **下载 Miniconda 安装脚本**
    打开终端，运行以下命令下载适用于 Linux 的 Miniconda 安装脚本：

   ```bash
   wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
   ```

2. **添加执行权限**
    为脚本添加执行权限：

   ```bash
   chmod +x miniconda.sh
   ```

3. **运行安装脚本**
    执行脚本并按提示完成安装：

   ```bash
   ./miniconda.sh
   ```

   默认安装路径为 `~/miniconda3`，可根据需要修改。安装完成后，按提示添加 Conda 到 shell 配置文件。

4. **初始化 Conda**
    安装完成后，运行以下命令初始化 Conda，使其自动加载：

   ```bash
   source ~/miniconda3/bin/activate
   conda init
   ```

5. **重新打开终端**
    关闭并重新启动终端。看到 `(base)` 提示符时，说明 Conda 已成功激活。

6. **检查安装是否成功**
    验证安装：

   ```bash
   conda --version
   ```

7. **查看 Conda 环境信息**
    查看环境列表：

   ```bash
   conda info --envs
   ```

   如果是 `(base)` 环境，则无需指定环境名称。

8. **确保 Python 版本匹配**
    如果 `base` 环境的 Python 版本与系统不同，使用以下命令调整：

   ```bash
   conda install python=3.12.3
   ```

   否则，可能在 Jupyter 中无法找到 `rclpy`。

---

### 添加 Conda 镜像源

1. **查看当前镜像源**

   ```bash
   conda config --show channels
   ```

2. **恢复默认配置**

   ```bash
   conda config --remove-key channels
   ```

3. **配置清华镜像源**
    编辑配置文件 `/home/dw/miniconda3/.condarc`，添加以下内容：

   ```bash
   channels:
     - defaults
   show_channel_urls: true
   default_channels:
     - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
     - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
     - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2
   custom_channels:
     conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
     msys2: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
     bioconda: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
     menpo: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
     pytorch: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
     simpleitk: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
   ```

---

### 配置环境和安装包

1. **创建新环境**

   ```bash
   conda create --name myenv python=3.12
   ```

2. **激活环境**

   ```bash
   conda activate myenv
   ```

3. **安装需要的包**

   ```bash
   conda install tensorflow
   ```

---

感谢您的反馈！我将仔细检查并确保完全按照您的要求调整，不遗漏任何信息或格式。重新调整如下：

---

## 基础优化终端

### 安装 Kitty

#### 步骤 1: 安装 Kitty 终端

更新系统包并安装 Kitty：

```bash
sudo nala update
sudo nala install kitty
```

#### 步骤 2: 设置 Kitty 为默认终端

通过 `update-alternatives` 将 Kitty 设置为默认终端：

```bash
sudo update-alternatives --config x-terminal-emulator
```

- 在弹出的选择菜单中，选择 Kitty（通常会显示 Kitty 的路径，例如 `/usr/bin/kitty`）。
- 输入对应的数字，按回车确认。

---

### Kitty 基本配置

#### 1.1 创建配置文件

确保 Kitty 配置文件存在，如果没有，则创建：

```bash
mkdir -p ~/.config/kitty
touch ~/.config/kitty/kitty.conf
```

#### 1.2 编辑配置文件

使用编辑器打开配置文件：

```bash
nano ~/.config/kitty/kitty.conf
```

#### 1.3 基本设置

以下是我的基本设置，基于官方样式：

```bash
# 字体设置
font_family FiraCode  # 设置字体为 FiraCode
font_size 14.0        # 字体大小

# 启用字体连字
enable_ligatures yes

# 设置背景透明度
background_opacity 0.85  # 透明度（0 为全透明，1 为不透明）

# 设置窗口边距
window_padding 10  # 内边距，单位：像素

# 启用软换行
soft_wrap 1
```

---

#### 2.2 使用第三方主题

从 [kitty-themes GitHub](https://github.com/dexpota/kitty-themes) 下载主题，在 Kitty 终端中输入以下命令切换：

```bash
kitty +kitten themes
```

---

### 删除 Kitty

#### 1. **使用包管理器卸载**

如果通过 `nala` 安装 Kitty，可以使用以下命令卸载：

```bash
sudo nala remove --purge kitty
sudo nala autoremove
```

#### 2. **如果是从源代码编译安装**

1. 进入 Kitty 源代码目录。
2. 执行以下命令卸载：

```bash
sudo make uninstall
```

1. 删除安装目录：

```bash
sudo rm -rf /opt/kitty
sudo rm /usr/local/bin/kitty
sudo rm /usr/local/bin/kitty_config
```

确保删除所有相关二进制文件和配置文件。

---

### 删除配置文件和缓存

删除 Kitty 的配置文件和缓存：

```bash
rm -rf ~/.config/kitty
rm -rf ~/.local/share/kitty
rm -rf ~/.cache/kitty
```

---

### 检查是否删除干净

检查系统中是否仍有 Kitty 的痕迹：

```bash
which kitty
```

如果没有输出，则说明 Kitty 已完全卸载。如果仍显示路径，可以手动删除相应文件。

---


### base环境没出来

在阅读了官方的配置文档之后。
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/bb08bb47732f4f84a1eaac1f82b83fa8.png)
我们只需要添加这个指令进去就好了

```bash
code ~/.config/starship.toml
```

# (可选)安装 NVIDIA 驱动

虽然开源驱动已经能满足大部分需求，但为了更高的性能，可以考虑安装官方驱动。

推荐参考教程：

- [Ubuntu 24.04 安装 Nvidia 显卡驱动 + CUDA + cuDNN，配置 AI 深度学习训练环境](https://blog.csdn.net/u010912615/article/details/141195878)

---

## 删除 NVIDIA 驱动及其相关文件（如果发生问题）

其他参考方案：

- [NVIDIA 显卡的 Ubuntu 驱动程序安装方法](https://www.bilibili.com/video/BV1wY411p7mU)

---

### 通过 GRUB 菜单进入恢复模式

1. **重启系统**

   - 系统启动时，按住 `Shift` 键，直到出现 GRUB 菜单。
   - 若使用 UEFI 启动，按 `Esc` 键。

2. **选择恢复模式**
    在 GRUB 菜单中选择带有 `Recovery Mode` 的内核版本。

3. **进入根终端**
    在恢复模式菜单中，选择 `root`。

4. **停止图形界面**
    如果图形界面仍在运行，使用以下命令停止：

   ```bash
   sudo systemctl stop gdm       # GNOME
   sudo systemctl stop sddm      # KDE
   sudo systemctl stop lightdm   # LightDM
   ```

5. **卸载 NVIDIA 模块**

   ```bash
   sudo rmmod nvidia_drm nvidia_modeset nvidia
   ```

---

### 确保卸载干净

1. **完全卸载 NVIDIA 驱动**

   ```bash
   sudo apt-get purge '^nvidia-.*'
   ```

2. **移除残留配置文件和内核模块**

   ```bash
   sudo rm -rf /lib/modules/$(uname -r)/kernel/drivers/video/nvidia*
   sudo rm -rf /etc/X11/xorg.conf.d/10-nvidia.conf
   sudo rm -rf /etc/modprobe.d/nvidia.conf
   sudo rm -rf /etc/modprobe.d/nvidia-installer-disable-nouveau.conf
   ```

3. **更新 initramfs**

   ```bash
   sudo update-initramfs -u
   ```

4. **重启计算机**

   ```bash
   sudo reboot
   ```

---

### 卸载后重新安装驱动（可选）

1. **使用 NVIDIA `.run` 文件安装驱动**

   ```bash
   sudo ./NVIDIA-Linux-x86_64-560.31.02.run
   ```

2. **通过 Ubuntu 驱动管理工具安装**

   ```bash
   sudo ubuntu-drivers autoinstall
   sudo reboot
   ```

---

### 验证驱动安装

1. **检查 NVIDIA 驱动**

   ```bash
   nvidia-smi
   ```

   如果成功，应该看到当前驱动和显卡信息，例如：

   ```bash
   +-----------------------------------------------------------------------------+
   | NVIDIA-SMI 460.80    Driver Version: 460.80    CUDA Version: 11.2           |
   +-----------------------------------------------------------------------------+
   ```

2. **检查 NVIDIA 内核模块**

   ```bash
   lsmod | grep nvidia
   ```

   应看到 `nvidia`, `nvidia_modeset`, `nvidia_drm` 等模块。

---

# (可选)安装 ROS2-Jazzy

## 教程参考

- [Ubuntu 24.04 安装 Jazzy 版 ROS2 的前置操作（防报错）](https://blog.csdn.net/jk0_0/article/details/143659242)
   **注意**：此教程可能存在安装版本上的错误，可酌情参考。
- [Ubuntu 24.04 安装 ROS2 Jazzy](https://blog.csdn.net/weixin_74185504/article/details/141285787)
   **说明**：无需参考换源，遇到问题可通过编辑配置文件解决。

---

## 安装 Jupyter

### 注意事项

**确保 Python 版本统一**：
 Jupyter 环境中的 Python 版本需与系统或 Conda 虚拟环境中的 Python 版本一致，避免依赖冲突。

**检查调用环境**：
 在 `.ipynb` 文件中运行以下代码，检查是否正确调用了 Conda 环境：

```python
import sys
print(sys.executable)
```

---

### 使用 Conda 安装 Jupyter

建议在 Conda 环境中安装 Jupyter，避免依赖问题：

```bash
conda install jupyter
```

---

### 正确安装操作

**创建新虚拟环境**
 为减少出错可能性，建议创建一个新的虚拟环境，避免使用 `base` 环境。
 **确保 Python 版本一致**，例如：

```bash
conda create -n ros2 python=3.12.3
```

**安装 Jupyter**
 激活环境后，安装 Jupyter：

```bash
conda install jupyter
```

---

### 常见问题

1. **依赖过旧**
    使用 Conda 安装时，可能依赖版本较低，导致无法满足 ROS2-Jazzy 的要求：
   - 系统 `libstdc++` 库版本低，缺少 `GLIBCXX_3.4.30`。
2. **Python 版本不一致**
    ROS2 安装在系统环境，而 Conda 虚拟环境的 Python 版本不统一，可能导致模块如 `rclpy` 无法加载。

---

### 参考解决方案

- [ROS2 用 Jupyter 时在 `import rclpy` 时的报错](https://fishros.org.cn/forum/topic/1581/动手学ros2第六章基础篇3-2用import来导入出错)
- [修复 `GLIBCXX_3.4.30` not found 的问题](https://blog.csdn.net/weixin_44853528/article/details/135226071?fromshare=blogdetail&sharetype=blogdetail&sharerId=135226071&sharerefer=PC&sharesource=weixin_52322190&sharefrom=from_link)



---

## Jupyter **nbextensions** 一个拓展

Jupyter nbextensions 可以为 Jupyter Notebook 添加更多功能和工具，极大提升使用体验。

### 安装 `jupyter_contrib_nbextensions`

在 Conda 环境中运行以下命令，安装包含多个扩展和配置工具的包：

```bash
conda install -c conda-forge jupyter_contrib_nbextensions
```

### 安装并启用扩展

运行以下命令将扩展文件安装到 Jupyter 配置目录：

```bash
jupyter contrib nbextension install --user
```

启动 Jupyter Notebook 查看效果：

```bash
jupyter notebook
```

更多使用参考：

- [jupyter notebook 进阶使用：nbextensions](https://blog.csdn.net/qq_40206371/article/details/119655931)

---

### 其他支持工具安装

安装一些常用的科学计算工具：

```bash
conda install numpy transforms3d
```

---
我上传了一张图片，上面有非常局部的大纲图。在目录架构上能不能更加完整，你可以举一些非常具体的例子给我比如说cartographer如何配置，cartographer的膨胀层（这个例子不好，我在想，我是用算法名称驱动学习技术概念，还是直接从技术概念入手，以后新建cartographer区、fast-lio区、数学概念区？你能在这方面给我一个建议吗），还有比如说vnc是怎么设置自动开机的。同时新建一些模板分区，比如说常用命令、代码模板、

