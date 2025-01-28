# package.xml

```bash
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- 包名 -->
  <name>example_action_rclcpp</name>
  
  <!-- 版本号 -->
  <version>0.0.0</version>
  
  <!-- 包描述 -->
  <description>TODO: Package description</description>
  
  <!-- 维护者信息 -->
  <maintainer email="fishros@foxmail.com">fishros</maintainer>
  
  <!-- 许可证 -->
  <license>Apache-2.0</license>

  <!-- 构建工具依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 运行时依赖 -->
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>robot_control_interfaces</depend>

  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <!-- 导出信息 -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

==重点放在运行时的依赖上==

