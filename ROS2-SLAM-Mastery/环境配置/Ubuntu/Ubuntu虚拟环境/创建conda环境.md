根据你上传的截图，以下是虚拟环境和系统环境的对比分析，以及可能的解决方案：

### 分析：

1. **虚拟环境和系统环境的区别**：
   - **虚拟环境**中同时存在 `libgdal.so.34` 和 `libgdal.so.35`，而 **系统环境**中只有 `libgdal.so.35`。
   - `dpkg -S` 查询结果显示，`libgdal.so.34` 和 `libgdal.so.35` 都无法通过系统包管理器 (`dpkg`) 定位到对应的安装来源，说明这些库可能是通过其他方式（如 Conda 或手动编译）安装的。
2. **问题可能的来源**：
   - `libgdal.so.34` 可能由虚拟环境的 Conda 包自动引入，而 **系统环境中使用的 `libgdal.so.35`** 则是通过 APT 安装的。
   - 虚拟环境优先加载了 `libgdal.so.34`，这可能导致了与 `libgdal.so.35` 依赖的冲突。

------

### 解决方案：

#### 方法 1：删除虚拟环境中多余的 `libgdal.so.34`

在 Conda 虚拟环境中，尝试通过 Conda 包管理器卸载 `libgdal`：

```bash
conda deactivate  # 确保退出当前环境
conda activate ros2  # 进入虚拟环境
conda list | grep gdal  # 查看 gdal 相关包
```

如果 `gdal` 或相关包存在：

```bash
conda remove gdal
conda remove libgdal
```

卸载后重建 `ros2` 虚拟环境的依赖：

```bash
conda install gdal  # 确保安装最新版本的 gdal
```

验证 `libgdal` 是否正确：

```bash
ldconfig -p | grep gdal
```

#### 方法 2：强制虚拟环境使用系统库

修改 `LD_LIBRARY_PATH`，让虚拟环境优先加载系统的 `libgdal.so.35`：

```bash
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

将其添加到虚拟环境的 `activate.d` 脚本中，确保每次激活虚拟环境时都设置正确的路径：

```bash
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' > $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
```

#### 方法 3：清理虚拟环境并重新创建

如果虚拟环境依赖过于复杂，建议重新创建一个干净的虚拟环境：

```bash
conda deactivate
conda remove --name ros2 --all
conda create --name ros2 python=3.12.3
conda activate ros2
conda install -c conda-forge rosdep colcon-common-extensions gdal
```

之后，重新配置 ROS 环境，确保使用系统依赖：

```bash
source /opt/ros/jazzy/setup.zsh
```

