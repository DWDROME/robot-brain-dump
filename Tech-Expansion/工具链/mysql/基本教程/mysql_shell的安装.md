------

# **MySQL Shell 安装与配置**

#### **1. 安装 MySQL Shell**

MySQL Shell 是 MySQL 提供的高级命令行工具，安装命令如下：

```bash
sudo nala install mysql-shell
```

#### **2. 启动 MySQL Shell**

启动 MySQL Shell：

```mysql
mysqlsh
```

默认进入 **JavaScript 模式**（标志是 `MySQL  JS>`）。你可以通过以下命令切换到 SQL 模式：

```mysql
\sql
```

进入 SQL 模式后，提示符变为：

```mysql
MySQL  SQL>
```

#### **3. 连接到 MySQL 实例**

用 `root` 用户连接本地 MySQL 数据库：

```mysql
\connect root@localhost
```

提示输入密码，输入你在 `mysql_secure_installation` 中设置的 root 密码。

#### **4. 基本操作**

连接成功后，就可以执行基本的 SQL 命令了。以下是几个常用操作：

- 显示数据库列表：

  ```mysql
  SHOW DATABASES;
  ```

- 选择数据库：

  ```mysql
  USE cinema_db;
  ```

- 查看表结构：

  ```mysql
  DESCRIBE tickets;
  ```

- 插入数据：

  ```mysql
  INSERT INTO tickets (movie_name, seat_number, user_name) 
  VALUES ('Avatar', 'A1', 'John Doe');
  ```

- 查询数据：

  ```mysql
  SELECT * FROM tickets;
  ```

#### **5. 退出 MySQL Shell**

输入以下命令退出 Shell：

```mysql
\quit
```

------

### **完成！**

MySQL Shell 更加直观高效，尤其在需要快速调试或运行脚本时非常方便。等你熟悉了，我们可以进一步用它来执行 `.sql` 文件或者自动化脚本，随时喊我！