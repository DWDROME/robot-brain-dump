---

# **MySQL 基本安装教程**

#### **1. 安装 MySQL**
通过 `nala` 安装 MySQL 服务器：
```bash
sudo nala install mysql-server
```

#### **2. 启动并设置 MySQL 服务**
安装完成后，启动 MySQL 服务并设置开机自启动：
```bash
sudo systemctl start mysql
sudo systemctl enable mysql
```

#### **3. 检查 MySQL 服务状态**
确保 MySQL 服务运行正常：
```bash
sudo systemctl status mysql
```
如果显示 `active (running)`，说明服务启动成功。

#### **4. 运行安全初始化脚本**
这个步骤会帮助你完成基础配置，比如设置 root 密码、移除测试数据库等：
```bash
sudo mysql_secure_installation
```
**按照提示进行配置：**
- 设置 root 密码：根据提示输入一个简单易记的密码。
- 移除匿名用户：选择 `Yes`。
- 禁止 root 远程登录：选择 `Yes`。
- 删除测试数据库：选择 `Yes`。
- 重载权限表：选择 `Yes`。

---

### **完成！**
以上就是 MySQL 的基础安装和初步配置教程，足够你后续操作和开发使用。

