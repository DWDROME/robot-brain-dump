
### **📚 Robot Brain Dump - 知识库**

欢迎来到 **Robot Brain Dump**！这是一个专注于 **ROS 2、SLAM、嵌入式开发、算法研究** 及 **工具链学习** 的知识库，旨在系统化整理我的学习笔记与实践经验。

📌 **本仓库的核心目标：**

- **🌍 机器人导航与 SLAM 研究**：包含 Cartographer、FAST-LIO、Gazebo 仿真、硬件接口等相关内容。
- **📖 知识整理**：记录 ROS 2 生态、算法实现、数学基础、环境配置等主题。
- **💡 个人项目与实战经验**：涵盖自动导航小车、MOGI 仿真、小海龟写字等项目。
- **🛠️ 工具链与嵌入式开发**：包括 MarkDown、MySQL、树莓派、zsh 美化等技术栈。

目前我正在进行的完善项目有：
- [DWDROME_MOG](https://github.com/DWDROME/MOGI)

---

## 📂 目录结构

```bash
robot-brain-dump
├── .obsidian/                     # Obsidian 配置文件（插件等）
├── Life-Archives/                 # 生活记录
│   ├── 成长计划/
│   └── 投资研究/
├── ROS2-SLAM-Mastery/             # 机器人导航 & SLAM 研究
│   ├── !索引系统/
│   ├── 学习规划/
│   ├── 技术栈/
│   │   ├── gazebo/
│   │   │   ├── ros_gz_project_template/
│   │   │   │   ├── ros_gz_example_gazebo/
│   │   │   │   │   ├── new_drive初始.assets/
│   │   │   │   │   ├── worlds/
│   │   │   │   │   ├── 改造方式.assets/
│   │   │   │   │   ├── 新的嵌入/
│   │   │   │   └── turtlebot3迁移版/
│   │   ├── ROS2/
│   │   │   ├── 数学基础/linear algebra/
│   │   │   ├── 核心概念/
│   │   │   │   ├── ros动作详解/机器人/action_control_01.assets/
│   │   │   │   ├── URDF/
│   │   │   ├── 桥/
│   │   │   ├── 硬件接口/slamtec A1M8/学习路径.assets/
│   │   │   ├── 算法模块/
│   │   │   │   ├── Cartographer/概念/配置模板/问题库/
│   │   │   │   ├── FAST-LIO/
│   ├── 数学基础/
│   ├── 环境配置/
│   │   ├── fastdds/
│   │   ├── gazebo/
│   │   ├── Ubuntu/Ubuntu虚拟环境/
│   │   ├── vscode/
│   │   ├── wsl/
│   │   ├── zsh美化/
│   │   ├── 树莓派/
│   │   │   ├── cartographer建图.assets/
│   │   │   ├── vnc远程安装.assets/
│   │   │   ├── 待处理/问题配置区/
│   ├── 项目实战/
│   │   ├── MOGI自建ros模拟器/
│   │   ├── 小海龟写字/
│   │   ├── 手持雷达建图/
│   │   ├── 新丰农场小车/octomap_server.assets/
│   │   ├── 自动导航小车/
├── Tech-Expansion/                # 技术扩展
│   ├── 嵌入式开发/
│   ├── 工具链/
│   │   ├── MarkDown教程/latex使用指南.assets/
│   │   ├── MySQL/基本教程/
│   ├── 编程语言/
│   │   ├── C++/项目/电影院选票系统/
│   │   ├── Python/
```

---

## 🚀 如何使用本仓库？

1. **📥 克隆仓库**
    
    ```bash
    git clone https://github.com/DWDROME/robot-brain-dump.git
    cd robot-brain-dump
    ```
    
2. **🛠️ 使用 Obsidian 阅读笔记**
    
    - 推荐使用 [Obsidian](https://obsidian.md/) 打开此仓库，能获得更好的笔记体验（已包含 `.obsidian` 配置）。
    - 也可以直接浏览 Markdown 文件。
3. **🔍 快速查找内容**
    
    - 机器人导航 & SLAM → `ROS2-SLAM-Mastery/`
    - 环境配置 & 依赖管理 → `环境配置/`
    - 数学 & 算法 → `数学基础/`
    - 个人项目 → `项目实战/`

---

## 🌟 如何贡献？

欢迎任何形式的贡献！如果你有新的想法或改进建议，可以：

1. **提交 Issue**
    
    - 在 [Issue 页面](https://github.com/DWDROME/robot-brain-dump/issues) 反馈建议或问题。
2. **Fork & Pull Request**
    
    ```bash
    git fork https://github.com/DWDROME/robot-brain-dump.git
    git clone https://github.com/你的用户名/robot-brain-dump.git
    cd robot-brain-dump
    git checkout -b new-feature
    git commit -m "添加新内容"
    git push origin new-feature
    ```
    
    - 提交 PR，我会 review 你的更改！

---

## 📢 交流与反馈

如果你对本知识库有任何问题或建议，欢迎联系：

- **GitHub Issue**：[提交问题](https://github.com/DWDROME/robot-brain-dump/issues)
- **Email**：[ds_everything-ocean@outlook.com](mailto:ds_everything-ocean@outlook.com)

希望这个仓库对你的学习有所帮助！🚀
