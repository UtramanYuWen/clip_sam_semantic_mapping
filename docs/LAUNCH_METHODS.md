# 系统启动方式指南

本项目提供 **两种启动方式** 供用户选择:

## 方式 1: 一体化启动 (推荐) ⭐

### 说明
在单个终端中启动完整系统的所有组件。所有功能集成在一个 ROS launch 文件中，包括仿真环境、SLAM建图、语义分析和可视化。

### 启动命令
```bash
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
```

### 包含组件
| 组件 | 描述 | 功能 |
|------|------|------|
| **Gazebo仿真** | WPR Stage RoboCup环境 | 模拟真实机器人和家具环境 |
| **键盘控制** | keyboard_vel_ctrl节点 | `w/s` 前进/后退, `a/d` 左转/右转, `q/e` 自旋, `空格` 停止 |
| **SLAM建图** | gmapping算法 | 实时构建栅格地图 |
| **语义分析** | CLIP+SAM系统 | 房间识别、家具检测、结构分割 |
| **可视化** | RViz | 显示地图、语义标签、机器人位姿 |

### 优点
✅ **简单易用** - 一个命令启动所有功能  
✅ **高度集成** - 所有组件自动协调工作  
✅ **完整功能** - 仿真、建图、语义分析三位一体  
✅ **推荐方式** - 经过充分测试和优化  

### 缺点
❌ 系统负载较高 (所有组件同时运行)  
❌ 调试困难 (多个组件混在一起)  
❌ 不适合独立开发某个子模块  

### 典型工作流
```
1. 打开一个终端
2. 执行: roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
3. 等待 Gazebo 窗口和 RViz 打开
4. 在 Gazebo 窗口中按 wasdqe 控制机器人移动
5. 观看 RViz 中的地图实时构建和语义标签
6. 按 Ctrl+C 停止所有节点
```

---

## 方式 2: 分步启动 (灵活性高)

### 说明
在多个终端中分别启动仿真环境和语义建图系统。这种方式适合调试、开发或只运行部分功能。

### 启动命令

**终端 1 - 启动仿真环境:**
```bash
roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
```

**终端 2 - 启动语义建图系统:**
```bash
roslaunch clip_sam_semantic_mapping semantic_only.launch
```

### 包含组件

#### 第1步: wpb_stage_robocup_custom.launch
| 组件 | 描述 | 功能 |
|------|------|------|
| **Gazebo仿真** | WPR Stage RoboCup 27个家具 | 模拟真实环境 |
| **键盘控制** | keyboard_vel_ctrl节点 | 手动驾驶机器人 |
| **传感器驱动** | 激光雷达/相机/IMU驱动 | 发布传感器数据 |

#### 第2步: semantic_only.launch
| 组件 | 描述 | 功能 |
|------|------|------|
| **SLAM建图** | gmapping算法 | 消费激光雷达数据，构建地图 |
| **语义分析** | CLIP+SAM系统 | 消费相机数据，进行语义分割 |
| **可视化** | RViz | 显示所有结果 |

### 优点
✅ **模块化** - 可独立启动/停止各组件  
✅ **灵活性高** - 可以跳过某些步骤  
✅ **易于调试** - 组件隔离，便于问题诊断  
✅ **资源优化** - 可选择性启动组件  

### 缺点
❌ **操作繁琐** - 需要打开多个终端  
❌ **易出错** - 需记住启动顺序  
❌ **需要同步** - 组件间需要正确的话题连接  

### 启动顺序重要！ ⚠️
必须先启动 `wpb_stage_robocup_custom.launch`，再启动 `semantic_only.launch`：
- `wpb_stage_robocup_custom.launch` **发布** 激光雷达和相机话题
- `semantic_only.launch` **消费** 这些话题

### 典型工作流
```
1. 打开终端 1
   执行: roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
   等待: Gazebo 窗口打开，机器人生成，传感器驱动加载

2. 打开终端 2
   执行: roslaunch clip_sam_semantic_mapping semantic_only.launch
   等待: RViz 打开，开始消费传感器数据

3. 在 Gazebo 窗口中按 wasdqe 控制机器人

4. 停止系统
   终端 2: Ctrl+C 停止语义建图
   终端 1: Ctrl+C 停止仿真环境
```

---

## 方式对比

| 特性 | 方式1 (一体化) | 方式2 (分步) |
|------|----------------|------------|
| **启动终端数** | 1个 | 2个 |
| **启动命令数** | 1个 | 2个 |
| **学习曲线** | 简单 ⭐ | 中等 |
| **调试能力** | 低 | 高 ⭐ |
| **系统负载** | 高 | 可控 |
| **故障隔离** | 困难 | 容易 ⭐ |
| **推荐场景** | 日常使用 ⭐ | 开发/调试 ⭐ |

---

## 常见问题

### Q1: 我是否应该总是使用方式1?
**A:** 对于日常使用，是的。方式1最简单可靠。但如果：
- 你正在开发新功能 → 使用方式2
- 你需要调试问题 → 使用方式2  
- 你只想运行语义分析 → 使用方式2
- 你需要最小系统负载 → 使用方式2

### Q2: 能否在方式2中只运行仿真而不运行语义分析?
**A:** 可以。只执行：
```bash
roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
```
这样只有 Gazebo 和键盘控制运行，没有SLAM或语义分析。

### Q3: 能否在真实机器人上使用方式1?
**A:** 不能。方式1依赖 `wpb_stage_robocup_custom.launch` 中的 Gazebo 仿真。  
对真实机器人，需要：
- 修改 `hierarchical_clip_sam_mapping.launch`
- 将 Gazebo 启动代码替换为真实机器人驱动
- 或使用方式2的模式，分别启动机器人驱动和语义分析

### Q4: 方式2中的两个launch文件可以使用同一个终端依次启动吗?
**A:** 不推荐。因为：
- 第一个launch文件会阻塞终端
- 必须在后台运行 (`&`) 或使用 `screen`/`tmux`
- 调试时容易混淆输出日志

**建议:** 打开两个独立终端。

### Q5: 如何知道两个launch文件是否正确连接?
**A:** 检查 ROS 话题：
```bash
# 在第三个终端检查话题
rostopic list

# 应该看到:
/scan                    # 激光雷达话题 (由方式2.1发布)
/camera/color/image_raw  # 相机话题 (由方式2.1发布)
/map                     # 地图话题 (由方式2.2发布)
/semantics_map           # 语义地图话题 (由方式2.2发布)
```

---

## 推荐选择

### 对于大多数用户
👉 **使用方式1 (一体化启动)**

理由：
- 最简单
- 功能完整
- 经过充分测试
- 无需考虑组件间的协调

### 对于开发者
👉 **使用方式2 (分步启动)**

理由：
- 模块化结构便于开发
- 易于调试
- 可选择性启动
- 便于性能分析

---

## 启动文件详解

### hierarchical_clip_sam_mapping.launch (方式1)
**位置:** `launch/hierarchical_clip_sam_mapping.launch`

这是推荐的启动文件，包含：
1. 仿真环境 (通过包含 `wpb_stage_robocup_custom.launch`)
2. SLAM节点 (gmapping)
3. 语义建图节点 (CLIP+SAM)
4. RViz可视化

### wpb_stage_robocup_custom.launch (方式2.1)
**位置:** `launch/wpb_stage_robocup_custom.launch`

自定义的Gazebo启动文件，基于 `wpr_simulation` 修改：
- 启动Gazebo仿真器
- 生成机器人和27个家具对象
- 加载传感器驱动 (激光雷达、相机、IMU)
- 启动键盘控制节点

### semantic_only.launch (方式2.2)
**位置:** `launch/semantic_only.launch`

仅运行SLAM和语义分析，不包含仿真：
- gmapping SLAM节点
- CLIP+SAM语义建图节点
- RViz可视化

---

## 下一步

- [完整使用指南](./USAGE.md)
- [仿真环境配置](./SIMULATION_SETUP.md)
- [自定义仿真设置](./CUSTOM_SIMULATION.md)
- [安装指南](./INSTALLATION.md)
