以下是 **MySQL Shell (`mysqlsh`)** 的基本命令总结，帮助你快速上手，只涵盖常用的内容，够用即可：

------

### **1. 启动 MySQL Shell**

启动 MySQL Shell：

```
mysqlsh
```

------

### **2. 模式切换**

MySQL Shell 支持三种模式，默认是 JavaScript 模式，可以通过以下命令切换：

- **切换到 SQL 模式：**

  ```
  \sql
  ```

  提示符变为：`MySQL  SQL>`

- **切换到 JavaScript 模式：**

  ```
  \js
  ```

  提示符变为：`MySQL  JS>`

- **切换到 Python 模式：**

  ```
  \py
  ```

  提示符变为：`MySQL  PY>`

------

### **3. 连接到数据库**

- **连接本地数据库：**

  ```
  \connect root@localhost
  ```

  输入密码后即可登录。

- **连接远程数据库：**

  ```
  \connect user@192.168.1.100
  ```

- **退出当前连接：**

  ```
  \disconnect
  ```

------

### **4. SQL 操作（在 SQL 模式下）**

- 显示数据库：

  ```
  SHOW DATABASES;
  ```

- 选择数据库：

  ```
  USE cinema_db;
  ```

- 显示表：

  ```
  SHOW TABLES;
  ```

- 插入数据：

  ```
  INSERT INTO tickets (movie_name, seat_number, user_name) 
  VALUES ('Avatar', 'A1', 'John Doe');
  ```

- 查询数据：

  ```
  SELECT * FROM tickets;
  ```

------

### **5. 运行脚本**

- **运行 SQL 文件：**

  ```
  \source /path/to/your/script.sql
  ```

  （确保文件路径正确）

- **运行 JavaScript/Python 文件：**

  ```
  \source /path/to/your/script.js   # JS 脚本
  \source /path/to/your/script.py   # Python 脚本
  ```

------

### **6. JSON 输出**

- **以 JSON 格式显示查询结果：**

  ```
  SELECT * FROM tickets FORMAT JSON;
  ```

- **保存查询结果为 JSON 文件：**

  ```js
  \js
  var res = session.runSql('SELECT * FROM tickets');
  shell.dumpToFile('results.json', res.fetchAll());
  ```

------

### **7. 帮助与退出**

- 查看帮助：

  ```
  \help
  ```

- 退出 MySQL Shell：

  ```
  \quit
  ```

------

### **8. 数据库对象管理（Python/JavaScript 模式）**

- 在 JavaScript 模式下操作数据库：

  ```js
  var db = session.getSchema('cinema_db');
  var table = db.getTable('tickets');
  table.insert(['movie_name', 'seat_number', 'user_name'])
       .values('Inception', 'B2', 'Jane Doe')
       .execute();
  ```

- 在 Python 模式下操作数据库：

  ```python
  db = session.get_schema('cinema_db')
  table = db.get_table('tickets')
  table.insert(['movie_name', 'seat_number', 'user_name']).values('Matrix', 'C1', 'Alice').execute()
  ```

------

### **总结**

这些命令覆盖了 `mysqlsh` 的日常使用，包括模式切换、连接数据库、运行脚本和执行 SQL 操作。如果你的项目需求不复杂，可以主要使用 SQL 模式，后续有更多需要时再深入学习 Python 或 JavaScript 模式。随时找我帮忙！