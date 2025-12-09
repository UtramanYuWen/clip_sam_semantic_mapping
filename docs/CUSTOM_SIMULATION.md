# 自定义仿真配置说明

## 文件说明

### wpb_stage_robocup_custom.launch

此文件位于 `launch/wpb_stage_robocup_custom.launch`，是基于 `wpr_simulation/launch/wpb_stage_robocup.launch` 的自定义版本。

**主要改进：**

| 特性 | 原始版本 | 自定义版本 |
|------|--------|---------|
| 机器人移动 | ❌ 无法直接控制 | ✅ 支持键盘控制 |
| 环境初始化 | ✓ 基础 | ✓ 保持不变 |
| 物体生成 | ✓ 保持 | ✓ 保持 |
| 传感器驱动 | ✓ 保持 | ✓ 保持 |
| 集成度 | 低 | 高 |

## 关键修改

### 1. 添加了键盘控制节点

```xml
<!-- 键盘控制节点 - 用于手动驾驶机器人 -->
<node pkg="wpr_simulation" 
      type="keyboard_vel_ctrl" 
      name="keyboard_vel_ctrl" 
      output="screen"/>
```

**用途：** 允许用户通过键盘控制机器人移动，便于手动探索环境进行语义建图。

**控制键：**
- `W/↑` - 前进
- `S/↓` - 后退  
- `A/←` - 左转
- `D/→` - 右转
- `Space` - 停止

### 2. 保留了所有原始功能

- Gazebo环境启动
- 物体生成（沙发、床、桌子、椅子等）
- 机器人模型加载
- TF变换发布
- 激光数据过滤
- 传感器驱动

## 何时使用本文件

### ✅ 推荐使用 `wpb_stage_robocup_custom.launch` 的场景：

1. **完整系统启动** - 包含SLAM和语义建图
   ```bash
   roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
   ```

2. **需要机器人移动控制** - 手动驾驶探索环境
   ```bash
   roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
   ```

3. **调试仿真环境** - 检查传感器和物体配置
   ```bash
   roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
   ```

### ❌ 不推荐使用的场景：

- 如果需要原始的 wpr_simulation 功能 → 使用 `wpr_simulation/launch/wpb_stage_robocup.launch`
- 如果只进行SLAM没有语义建图 → 使用 `semantic_only.launch`

## 与原始文件的对比

### 文件位置

| 文件 | 位置 | 用途 |
|------|------|------|
| 原始版本 | `wpr_simulation/launch/wpb_stage_robocup.launch` | wpr_simulation项目的官方启动文件 |
| 自定义版本 | `clip_sam_semantic_mapping/launch/wpb_stage_robocup_custom.launch` | 本项目的自定义启动文件 |

### 维护说明

- **何时更新：** 当需要添加新功能或修改Gazebo配置时
- **如何更新：** 编辑 `launch/wpb_stage_robocup_custom.launch` 文件
- **不会影响：** wpr_simulation项目本身

## 集成到hierarchical_clip_sam_mapping.launch

主启动文件中的集成：

```xml
<launch>
  <!-- ... 其他配置 ... -->
  
  <!-- ========== 1. WPR仿真环境 ========== -->
  <group>
    <include file="$(find clip_sam_semantic_mapping)/launch/wpb_stage_robocup_custom.launch" />
  </group>
  
  <!-- ========== 2. gmapping SLAM建图 ========== -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" ...>
    ...
  </node>
  
  <!-- ========== 3. 语义建图 ========== -->
  <node name="hierarchical_clip_sam_mapper" ...>
    ...
  </node>
  
  <!-- ========== 4. RViz可视化 ========== -->
  <node name="rviz_semantic_mapping" ...>
    ...
  </node>
</launch>
```

## 如何修改仿真配置

### 修改机器人初始位置

在 `wpb_stage_robocup_custom.launch` 中：

```xml
<!-- 生成机器人模型 -->
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
      args="-file $(find wpr_simulation)/models/wpb_home.model -urdf 
             -x 0.0 -y 0.0 -z 0.0 -model wpb_home" />
             ↑     ↑     ↑
           X坐标  Y坐标  Z坐标 (单位:米)
```

### 添加新物体

```xml
<!-- 示例: 添加新物体 -->
<node name="new_object" pkg="gazebo_ros" type="spawn_model" 
      args="-file $(find wpr_simulation)/models/your_model.model 
             -x 3.0 -y 2.0 -z 0 -Y 1.57 -urdf -model new_object" />
```

参数说明：
- `-x, -y, -z` - 位置坐标
- `-Y` - 旋转角度（弧度）
- `-urdf` - 模型格式

### 修改键盘控制参数

当前键盘控制节点使用默认参数。如需修改，编辑:

```xml
<node pkg="wpr_simulation" 
      type="keyboard_vel_ctrl" 
      name="keyboard_vel_ctrl" 
      output="screen">
  <!-- 这里可以添加参数 -->
  <param name="linear_speed" value="0.5" />
  <param name="angular_speed" value="1.0" />
</node>
```

## 故障排查

### 问题：机器人无法通过键盘控制

**解决方案：**
1. 检查wpr_simulation是否正确安装
   ```bash
   rospack find wpr_simulation
   ```

2. 检查keyboard_vel_ctrl节点是否运行
   ```bash
   rosnode list | grep keyboard
   ```

3. 查看节点错误输出
   ```bash
   roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
   ```

### 问题：Gazebo无法启动

**解决方案：**
1. 检查gazebo_ros是否安装
   ```bash
   apt list --installed | grep gazebo
   ```

2. 重新安装gazebo包
   ```bash
   sudo apt-get install -y ros-noetic-gazebo-ros
   ```

### 问题：物体未正确生成

**解决方案：**
1. 检查模型文件路径
2. 查看Gazebo日志
3. 尝试手动重新生成物体

## 参考文档

- 📖 [仿真环境完整指南](SIMULATION_SETUP.md)
- 📖 [使用指南](USAGE.md)
- 📖 [WPR Simulation](https://github.com/6-robot/wpr_simulation)
- 📖 [Gazebo官方文档](http://gazebosim.org/)

## 获取帮助

遇到问题？

1. 查看 [故障排查](TROUBLESHOOTING.md)
2. 检查 [使用指南](USAGE.md) 中的配置参数
3. 提交 GitHub Issue 并包含：
   - 错误信息
   - 系统信息 (Ubuntu版本、ROS版本)
   - 复现步骤
