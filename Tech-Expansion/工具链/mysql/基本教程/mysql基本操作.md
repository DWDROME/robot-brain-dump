好的！以下是 **MySQL 的基本操作总结**，只包含最常用的命令，专注于你项目的实用需求。

------

### **1. 登录 MySQL**

使用 root 用户登录：

```
sudo mysql -u root -p
```

### **2. 数据库管理**

- 显示所有数据库：

  ```
  SHOW DATABASES;
  ```

- 创建数据库：

  ```
  CREATE DATABASE cinema_db;
  ```

- 切换到某个数据库：

  ```
  USE cinema_db;
  ```

- 删除数据库：

  ```
  DROP DATABASE cinema_db;
  ```

------

### **3. 表操作**

- 查看当前数据库中的表：

  ```
  SHOW TABLES;
  ```

- 创建表：

  ```
  CREATE TABLE tickets (
      id INT AUTO_INCREMENT PRIMARY KEY,
      movie_name VARCHAR(100),
      seat_number VARCHAR(10),
      user_name VARCHAR(100)
  );
  ```

- 查看表结构：

  ```
  DESCRIBE tickets;
  ```

- 删除表：

  ```
  DROP TABLE tickets;
  ```

------

### **4. 数据操作**

- 插入数据：

  ```
  INSERT INTO tickets (movie_name, seat_number, user_name)
  VALUES ('Avatar', 'A1', 'John Doe');
  ```

- 查询数据：

  ```
  SELECT * FROM tickets;
  ```

- 条件查询：

  ```
  SELECT * FROM tickets WHERE movie_name = 'Avatar';
  ```

- 更新数据：

  ```
  UPDATE tickets SET seat_number = 'B2' WHERE id = 1;
  ```

- 删除数据：

  ```
  DELETE FROM tickets WHERE id = 1;
  ```

------

### **5. 用户管理**

- 创建新用户：

  ```
  CREATE USER 'my_user'@'localhost' IDENTIFIED BY 'password';
  ```

- 授予权限：

  ```
  GRANT ALL PRIVILEGES ON cinema_db.* TO 'my_user'@'localhost';
  FLUSH PRIVILEGES;
  ```

- 查看用户权限：

  ```
  SHOW GRANTS FOR 'my_user'@'localhost';
  ```

- 删除用户：

  ```
  DROP USER 'my_user'@'localhost';
  ```

------

### **6. 退出 MySQL**

输入以下命令退出：

```
EXIT;
```

------

### **总结**

这些操作覆盖了数据库创建、表管理、数据增删改查和用户权限配置，完全能满足你项目的需求。如果有更多需要，比如复杂查询或备份恢复，可以再深入学习。遇到问题随时找我！