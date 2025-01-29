# Mermaid 基本用法

> [!NOTE]
>
> Mermaid 方便在 markdown 中建图,
>
> 包含流程图、序列图、甘特图、状态图等各种图表



**可视化网站**

- "[Mermaidv11.4.0 Live Editor](https://mermaid.live/)"

## 示例：Mermaid 基本语法

1. **流程图 (Flowchart)**

   ```
   graph TD
       A[开始] --> B{是否继续?}
       B -- 是 --> C[执行任务]
       B -- 否 --> D[结束]
   ```

   ```mermaid
   graph TD
       A[开始] --> B{是否继续?}
       B -- 是 --> C[执行任务]
       B -- 否 --> D[结束]
   ```

   - `graph TD` 指定了流程图从上到下 (Top to Down) 的布局。
   - 每个节点用箭头连接，`-- 是 -->` 和 `-- 否 -->` 表示不同的分支。

2. **序列图 (Sequence Diagram)**

   ```
   sequenceDiagram
       participant A as 用户
       participant B as 系统
       A->>B: 请求数据
       B-->>A: 返回数据
   ```

   ```mermaid
   sequenceDiagram
       participant A as 用户
       participant B as 系统
       A->>B: 请求数据
       B-->>A: 返回数据
   ```

   - `sequenceDiagram` 指定了这是一个序列图。
   - `participant` 定义了图中各个参与者。

3. **甘特图 (Gantt Chart)**

   ```
   gantt
       title 项目进度
       dateFormat  YYYY-MM-DD
       section 设计阶段
       任务A :a1, 2023-11-01, 10d
       任务B :a2, after a1, 5d
       section 开发阶段
       任务C :2023-11-15, 10d
       任务D :2023-11-25, 7d
   ```

   ```mermaid
   gantt
       title 项目进度
       dateFormat  YYYY-MM-DD
       section 设计阶段
       任务A :a1, 2023-11-01, 10d
       任务B :a2, after a1, 5d
       section 开发阶段
       任务C :2023-11-15, 10d
       任务D :2023-11-25, 7d
   ```

   - `gantt` 表示甘特图，`title` 定义标题，`dateFormat` 指定日期格式。
   - `section` 表示甘特图的不同阶段。

4. **状态图 (State Diagram)**

   ```
   stateDiagram
       [*] --> 空闲
       空闲 --> 工作中
       工作中 --> 暂停
       暂停 --> 工作中
       工作中 --> 完成
       完成 --> [*]
   ```

   ```mermaid
   stateDiagram
       [*] --> 空闲
       空闲 --> 工作中
       工作中 --> 暂停
       暂停 --> 工作中
       工作中 --> 完成
       完成 --> [*]
   ```

   - `stateDiagram` 用于创建状态图。
   - `[*]` 表示开始或结束状态。
