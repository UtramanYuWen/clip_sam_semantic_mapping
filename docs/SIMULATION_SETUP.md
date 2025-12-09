# WPR Stage 仿真环境集成

## 简介

CLIP+SAM语义建图系统已针对**WPR Stage RoboCup仿真环境**进行了优化适配。本文档介绍如何在虚拟环境中使用该系统进行语义建图。

## 环境要求

### 必需软件

| 软件 | 版本 | 用途 |
|------|------|------|
| Ubuntu | 18.04/20.04 | 基础系统 |
| ROS | Melodic/Noetic | 机器人框架 |
| Stage | 4.3.0+ | 2D仿真器 |
| wpr_simulation | 最新 | WPR机器人包 |
| gmapping | | SLAM建图 |

### 硬件要求（仿真）

- CPU: 4核+ (Intel i5 或等价)
- 内存: 8GB+
- 显卡: 可选（CLIP+SAM使用）

## 安装wpr_simulation

### 第1步：获取wpr_simulation包

```bash
cd ~/catkin_ws/src

# 从GitHub克隆（或根据实际地址调整）
git clone https://github.com/6-robot/wpr_simulation.git

# 或手动下载并解压
```

### 第2步：安装仿真依赖

```bash
# 安装Stage仿真器
sudo apt-get install -y ros-noetic-stage ros-noetic-navigation

# 安装gmapping
sudo apt-get install -y ros-noetic-gmapping

# 对于Melodic替换noetic
```

### 第3步：编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动仿真环境

### 方法1：完整集成启动（推荐）

```bash
# 一条命令启动所有组件
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch

# 在另一终端启动RViz可视化（可选）
rviz -d src/clip_sam_semantic_mapping/rviz/hierarchical_semantic_mapping.rviz
```

此方法启动：
- ✅ Gazebo仿真环境（基于自定义 wpb_stage_robocup_custom.launch）
- ✅ gmapping SLAM建图
- ✅ CLIP+SAM语义建图
- ✅ RViz可视化
- ✅ 键盘控制节点

### 方法2：分步启动（用于调试）

```bash
# 终端1：启动Gazebo + gmapping
roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch

# 终端2：启动语义建图
roslaunch clip_sam_semantic_mapping semantic_only.launch

# 终端3：启动RViz
rviz
```

### 方法3：使用启动脚本

```bash
cd ~/catkin_ws/src/clip_sam_semantic_mapping
bash scripts/launch_complete_system.sh
```

## 自定义仿真配置

本项目使用自定义的 `wpb_stage_robocup_custom.launch` 文件，在官方wpr_simulation的基础上进行了改进：

**改进内容：**
- ✅ 添加了键盘控制节点（机器人移动）
- ✅ 保留所有原始功能（物体、传感器、Gazebo环境）
- ✅ 无需修改wpr_simulation源代码

详见 [自定义仿真配置](CUSTOM_SIMULATION.md)

```
┌──────────────────────┐
│  WPR Stage 环境      │
├──────────────────────┤
│ • 总面积: ~50m²      │
│ • 房间数: 8个        │
│ • 家具: 30+种        │
│ • 光线: 均匀         │
└──────────────────────┘
```

**房间布局：**
```
  ┌─────────┬─────────┐
  │ 卧室    │ 客厅    │
  ├─────────┼─────────┤
  │ 卫生间  │ 厨房    │
  ├─────────┼─────────┤
  │ 书房    │ 餐厅    │
  ├─────────┼─────────┤
  │ 玄关    │ 阳台    │
  └─────────┴─────────┘
```

## 配置仿真参数

编辑 `config/sim_config.yaml`:

```yaml
# 仿真器设置
simulator: "stage"
world_file: "worlds/robocup_home.world"

# Stage参数
stage_resolution: 0.05     # 网格分辨率
stage_speed_factor: 1.0    # 仿真速度（1.0=实时）

# 相机配置
camera_simulation: true
simulated_camera_fps: 30
simulated_image_width: 640
simulated_image_height: 480

# 机器人配置
robot_name: "wpr_c"
base_frame: "base_link"
odom_frame: "odom"
map_frame: "map"

# SLAM配置
use_gmapping: true
mapping_rate: 5.0           # Hz
update_min_d: 0.2           # 平移阈值
update_min_a: 0.2           # 旋转阈值

# 语义建图配置
semantic_mapping_enabled: true
room_detection_enabled: true
furniture_detection_enabled: true
```

## ROS话题映射

仿真中的关键话题：

```
Stage 仿真器
  ├─ /stage_ros/scan              (LaserScan)
  ├─ /stage_ros/camera            (Image)
  ├─ /tf                           (Transform)
  └─ /stage_ros/cmd_vel            (Twist)
        ↓
    gmapping SLAM
  ├─ /map                          (OccupancyGrid)
  ├─ /tf                           (更新)
  └─ /slam/entropy                 (Float64)
        ↓
    语义建图
  ├─ /semantic_map/room            (Image)
  ├─ /semantic_map/furniture       (Image)
  ├─ /semantic_map/structure       (Image)
  └─ /waypoints                    (PoseArray)
```

## 控制机器人

### 方法1：使用RViz导航

```bash
# 1. RViz中选择 "2D Nav Goal"
# 2. 在地图上点击目标位置
# 3. 机器人自动导航
```

### 方法2：发送Twist消息

```python
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('teleop')

twist = Twist()
twist.linear.x = 0.5      # 前进速度
twist.angular.z = 0.2     # 旋转速度
pub.publish(twist)
```

### 方法3：命令行

```bash
# 前进
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0}}'

# 原地旋转
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0}, angular: {z: 1.0}}'
```

## 监控和调试

### 检查系统状态

```bash
# 列出所有话题
rostopic list | grep -E "semantic|map|camera"

# 检查话题发布频率
rostopic hz /semantic_map/room

# 查看实时消息
rostopic echo /waypoints

# 查看系统性能
rosnode list
rosnode info /clip_sam_semantic_mapper
```

### RViz可视化

```bash
# 启动RViz
rviz -d src/clip_sam_semantic_mapping/rviz/rviz_sim_config.rviz
```

**RViz配置包含：**
- Stage仿真环境显示
- 激光扫描可视化
- gmapping地图
- 语义标签叠加
- 路点标记

### 性能监控

```bash
# 查看性能统计
rostopic echo /system_stats

# 监控GPU内存（如果使用GPU）
watch nvidia-smi

# 查看ROS节点内存占用
ps aux | grep clip_sam
```

## 常见仿真任务

### 任务1：地图探索

目标：机器人自主探索环境并生成语义地图

```bash
# 1. 启动仿真
roslaunch clip_sam_semantic_mapping sim_complete.launch

# 2. 启动自主探索
python3 scripts/autonomous_exploration.py \
  --max_time 300 \
  --coverage_threshold 0.85

# 3. 查看结果
# results/semantic_maps/ 中会生成地图文件
```

### 任务2：房间识别评估

目标：测试房间分类准确率

```bash
# 1. 启动语义建图
roslaunch clip_sam_semantic_mapping semantic_only.launch

# 2. 运行测试脚本
python3 scripts/evaluate_room_detection.py \
  --ground_truth config/ground_truth.yaml \
  --output evaluation_results.json

# 3. 查看报告
cat evaluation_results.json
```

### 任务3：路点生成验证

目标：验证生成的路点是否可到达

```bash
# 1. 启动导航系统
roslaunch clip_sam_semantic_mapping navigation.launch

# 2. 运行路点验证
python3 scripts/verify_waypoints.py \
  --waypoints results/waypoints/*/waypoints.xml \
  --timeout_per_point 60

# 3. 检查结果
# 每个路点是否成功到达
```

### 任务4：性能基准测试

```bash
# 运行基准测试脚本
python3 scripts/benchmark_sim.py \
  --duration 120 \
  --output benchmark_results.csv

# 结果包括：
# - 平均处理延迟
# - 内存占用
# - GPU使用率
# - 房间识别准确率
```

## 高级配置

### 自定义仿真世界

```bash
# 创建自定义world文件
cp worlds/robocup_home.world worlds/custom_home.world

# 编辑并配置
vim worlds/custom_home.world

# 在launch文件中引用
roslaunch clip_sam_semantic_mapping sim_complete.launch \
  world_file:=worlds/custom_home.world
```

### 多机器人仿真

```yaml
# config/multi_robot_sim.yaml
robots:
  - name: "wpr_c_1"
    start_x: 0.0
    start_y: 0.0
    start_theta: 0.0
  
  - name: "wpr_c_2"
    start_x: 5.0
    start_y: 5.0
    start_theta: 1.57

# 启动
roslaunch clip_sam_semantic_mapping multi_robot_sim.launch
```

## 故障排查

### 问题：仿真器不启动

```bash
# 检查Stage安装
which stageros

# 重新安装
sudo apt-get install --reinstall ros-noetic-stage

# 检查world文件
ls worlds/

# 启用debug输出
roslaunch wpr_simulation wpb_stage_robocup.launch debug:=true
```

### 问题：gmapping未发布地图

```bash
# 检查激光数据
rostopic echo /stage_ros/scan

# 启动gmapping调试
rosrun gmapping slam_gmapping \
  scan:=/stage_ros/scan \
  _delta:=0.05 \
  _xmax:=100 \
  _ymax:=100
```

### 问题：语义建图未接收图像

```bash
# 检查相机话题
rostopic list | grep camera
rostopic hz /stage_ros/camera

# 检查图像格式
rostopic echo /stage_ros/camera --noarr

# 修改config中的image_topic
image_topic: "/stage_ros/camera"
```

## 性能提示

### 加速仿真

```yaml
# config/clip_sam_config.yaml
stage_speed_factor: 2.0      # 2倍速
resolution: "360p"           # 降低分辨率
frame_skip: 2                # 跳帧处理
```

### 降低计算负载

```bash
# 运行CPU版本
python3 clip_sam_semantic_mapper.py --device cpu

# 减少工作进程
num_workers: 2

# 禁用可视化
visualization: false
```

## 与真实环境的区别

| 方面 | 仿真 | 真实 |
|------|------|------|
| 光照 | 均匀 | 变化 |
| 纹理 | 简化 | 复杂 |
| 传感器噪声 | 低 | 高 |
| 动作精度 | 完美 | 有误差 |
| 物体外观 | 标准化 | 多样化 |

**建议：** 在仿真环境中训练/测试基本功能，再迁移到真实环境进行微调。

## 参考资源

- [WPR仓库](https://github.com/6-robot/wpr_simulation)
- [Stage文档](https://github.com/rtv/Stage)
- [ROS导航栈](http://wiki.ros.org/navigation)
- [gmapping文档](http://wiki.ros.org/gmapping)

## 获取帮助

遇到问题？参考：
- 📖 [故障排查](TROUBLESHOOTING.md)
- 📖 [使用指南](USAGE.md)
- 📖 [安装指南](INSTALLATION.md)
