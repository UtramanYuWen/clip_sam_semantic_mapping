# CLIP+SAM 多层次语义建图系统

基于CLIP和SAM的实时多层次语义建图系统，同时生成房间、家具和结构元素的语义地图，并支持导航路点生成。

## ✨ 核心功能

- 🎯 **房间识别**: CLIP实时识别8种房间类型（客厅、卧室、厨房等）
- 🪑 **家具检测**: 分类识别沙发、床、桌子、椅子等30+种家具
- 🧱 **结构分割**: SAM分割识别墙壁、门、窗、楼梯等结构元素  
- 📍 **导航路点**: 自动生成PGM/YAML地图和XML格式的房间中心路点
- 🗺️ **多层次输出**: 房间、家具、结构三层独立语义地图
- ⚡ **高效推理**: 多进程架构，单帧处理 <500ms
- 🤖 **ROS集成**: 完整ROS Noetic/Melodic支持
- 📊 **性能监控**: 实时性能统计与准确率跟踪

## 📊 系统架构

```
输入源 (USB相机/ROS话题)
    ↓
[CLIP房间分类] + [SAM图像分割]
    ↓
[家具识别] + [结构检测]
    ↓
[多层次语义映射] + [路点生成]
    ↓
输出 (PNG地图/YAML/XML/PGM)
```

## 📋 支持的语义类别

### 房间类型 (8种)
- 客厅 (Living Room) | 卧室 (Bedroom) | 厨房 (Kitchen) | 卫生间 (Bathroom)
- 书房 (Study Room) | 餐厅 (Dining Room) | 阳台 (Balcony) | 玄关 (Entrance)

### 家具对象 (30+种)
沙发 | 床 | 桌子 | 椅子 | 柜子 | 电视 | 冰箱 | 洗衣机 | 微波炉 | 烤箱 | 灶台 | 水槽 | 淋浴房 | 浴缸 | 座便器 | 洗手池 | 书架 | 工作台 | 灯 | 植物 等

### 结构元素
墙壁 | 门 | 窗 | 地板 | 天花板 | 楼梯 | 台阶 | 栏杆

## 🚀 快速安装

### 前提条件
- Ubuntu 18.04 (ROS Melodic) 或 20.04 (ROS Noetic)
- **wpr_simulation** (WPR机器人仿真包) - 提供Gazebo仿真环境和模型
- Python 3.7+
- 4GB+ RAM, GPU推荐 (NVIDIA CUDA 11.8+)
- 5GB磁盘空间（含模型）

### 自动安装（推荐）

```bash
# 克隆ROS依赖包
cd ~/catkin_ws/src
git clone https://github.com/6-robot/wpr_simulation.git    # 仿真环境依赖

# 克隆本项目
git clone https://github.com/UtramanYuWen/clip_sam_semantic_mapping.git
cd clip_sam_semantic_mapping

# 运行自动安装脚本
bash scripts/install_dependencies.sh

# 编译工作空间
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 手动安装

**第1步: 安装ROS依赖包**
```bash
cd ~/catkin_ws/src
# 克隆仿真环境包（提供Gazebo环境和家具模型）
git clone https://github.com/6-robot/wpr_simulation.git

# 安装ROS系统依赖
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-geometry-msgs
```

**第2步: 安装Python依赖**
```bash
pip install -r requirements.txt
# 包括: torch, torchvision, clip, segment-anything, opencv-python, numpy 等
```

**第3步: 下载AI模型**
```bash
# CLIP ViT-B/32 模型 (~350MB)
python3 -c "import clip; clip.load('ViT-B/32')"

# SAM 模型 (~375MB)  
python3 -c "from segment_anything import sam_model_registry; \
  sam_model_registry['vit_b']('models/sam_vit_b_01ec64.pth')"
```

**第4步: 编译项目**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 📖 快速开始

### 🚀 方式1: 一体化启动 (推荐) ⭐

在一个终端中启动完整系统的所有组件:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
```

包含组件:
- ✓ Gazebo仿真 (WPR Stage RoboCup环境)
- ✓ 键盘控制 (wasdqe 移动机器人)
- ✓ gmapping SLAM建图
- ✓ CLIP+SAM多层次语义建图
- ✓ RViz可视化

**优点:** 简单一命令启动所有功能，推荐日常使用

### 🎮 方式2: 分步启动 (灵活高效)

如需更灵活的控制或仅运行部分功能：

**终端1 - 启动仿真环境:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clip_sam_semantic_mapping wpb_stage_robocup_custom.launch
```

**终端2 - 启动语义建图系统:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clip_sam_semantic_mapping semantic_only.launch
```

**优点:** 模块化控制，便于调试，可独立启停各组件

**说明:** 必须先启动终端1的仿真环境，再启动终端2的语义系统

### 📖 详细对比与推荐

详见 [两种启动方式完整指南](docs/LAUNCH_METHODS.md)

### 配置参数

编辑 `config/clip_sam_config.yaml`:

```yaml
# 房间识别提示词 (按优先级排序)
per_room_prompts:
  living_room: ["客厅", "起居室", "会客厅"]
  bedroom: ["卧室", "主卧", "次卧"]
  # ... 其他房间类型

# 推理设置
clip_model: "ViT-B/32"      # CLIP模型版本
sam_model: "vit_b"           # SAM模型版本  
device: "cuda:0"             # 计算设备
confidence_threshold: 0.3    # 置信度阈值

# 输出设置
output_format: ["pgm", "yaml", "xml"]  # 生成的地图格式
save_intermediate: true               # 保存中间结果
visualization: true                   # 实时可视化
```

### ROS话题接口

**订阅话题:**
- `/camera/rgb/image_raw` (sensor_msgs/Image) - 输入RGB图像

**发布话题:**
- `/semantic_map/room` (sensor_msgs/Image) - 房间语义地图
- `/semantic_map/furniture` (sensor_msgs/Image) - 家具语义地图  
- `/semantic_map/structure` (sensor_msgs/Image) - 结构语义地图
- `/waypoints` (geometry_msgs/PoseArray) - 导航路点

## 📁 输出文件格式

输出保存在 `results/` 目录下，按时间戳组织:

```
results/
├── semantic_maps/
│   └── map_YYYYMMDD_HHMMSS/
│       ├── room_semantic_*.png         (房间分层地图)
│       ├── furniture_semantic_*.png    (家具分布地图)
│       ├── structure_semantic_*.png    (结构元素地图)
│       └── combined_semantic_*.png     (组合地图)
│
└── waypoints/
    └── map_YYYYMMDD_HHMMSS/
        ├── map.pgm                     (ROS导航地图)
        ├── map.yaml                    (地图配置)
        └── waypoints.xml               (房间路点定义)
```

**XML路点格式示例:**
```xml
<?xml version="1.0"?>
<waypoints>
  <waypoint>
    <name>Living_Room_Center</name>
    <x>10.5</x>
    <y>8.3</y>
  </waypoint>
  <!-- ... 其他房间 -->
</waypoints>
```

## 🔧 高级配置

### 自定义房间提示词

编辑 `config/clip_sam_config.yaml` 中的 `per_room_prompts` 部分，为每个房间类型添加多个同义词：

```yaml
per_room_prompts:
  study_room: 
    - "书房"
    - "办公室"
    - "工作间"
    - "书搁"
```

### 性能优化

```bash
# GPU内存不足时，启用模型量化
python3 clip_sam_semantic_mapper.py --quantize

# 降低处理分辨率
python3 clip_sam_semantic_mapper.py --resolution 480p

# 并行处理模式
python3 clip_sam_semantic_mapper.py --workers 4
```

## 🏠 WPR Stage仿真集成

本系统支持与WPR Stage RoboCup仿真环境集成，用于虚拟环境中的完整语义建图。

**启动仿真系统：**
```bash
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
```

**注意：** 此启动文件使用自定义的 `wpb_stage_robocup_custom.launch`，包含以下自定义功能：
- 机器人移动控制（键盘控制节点）
- Gazebo仿真环境初始化
- 物体和传感器配置

详见 [SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md)

## 🗺️ 导航路点使用

生成的路点可直接用于机器人导航:

```python
import rospy
from geometry_msgs.msg import PoseStamped

# 加载路点
waypoints = load_waypoints('results/waypoints/map_*/waypoints.xml')

# 发送导航目标
for wp in waypoints:
    goal = PoseStamped()
    goal.pose.position.x = wp['x']
    goal.pose.position.y = wp['y']
    nav_client.send_goal(goal)
```

详见 [WAYPOINT_NAVIGATION.md](docs/WAYPOINT_NAVIGATION.md)

## 📊 性能指标

在RTX 3060 GPU上的基准测试:

| 操作 | 时间 | 内存 |
|------|------|------|
| 房间分类 | 45ms | 180MB |
| SAM分割 | 320ms | 2.1GB |
| 家具检测 | 85ms | 450MB |
| 路点生成 | 8ms | 80MB |
| **总计** | **458ms** | **2.8GB** |

## 🐛 故障排查

**问题: CUDA内存不足**
```bash
# 解决方案: 使用CPU或降低批处理大小
python3 clip_sam_semantic_mapper.py --device cpu --batch-size 1
```

**问题: 导入模块失败**
```bash
# 确保环境已激活
source ~/catkin_ws/devel/setup.bash
python3 -c "import clip; import segment_anything"
```

**问题: ROS话题无消息**
```bash
# 检查话题是否发布
rostopic list | grep semantic
rostopic echo /semantic_map/room
```

详见 [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

## 📚 项目结构

```
clip_sam_semantic_mapping/
├── src/
│   ├── clip_sam_semantic_mapper.py      # 主系统
│   ├── clip_room_classifier.py          # CLIP分类器
│   ├── sam_segmentation.py              # SAM分割
│   ├── furniture_structure_detector.py  # 家具/结构检测
│   ├── waypoint_generator.py            # 路点生成
│   ├── performance_optimizer.py         # 性能优化
│   ├── system_monitor.py                # 系统监控
│   ├── config_loader.py                 # 配置加载
│   ├── image_utils.py                   # 图像工具
│   └── utils/
│       └── path_manager.py              # 跨平台路径管理
├── launch/
│   ├── complete_system.launch           # 完整系统启动文件
│   ├── semantic_only.launch             # 仅语义建图启动
│   └── rviz_config.rviz                 # RViz可视化配置
├── config/
│   └── clip_sam_config.yaml             # 系统配置文件
├── scripts/
│   ├── install_dependencies.sh          # 自动安装脚本
│   ├── download_models.py               # 模型下载工具
│   └── run_complete_system.sh           # 系统启动脚本
├── docs/
│   ├── INSTALLATION.md                  # 详细安装指南
│   ├── USAGE_GUIDE.md                   # 使用指南
│   ├── SIMULATION_SETUP.md              # 仿真环境设置
│   ├── WAYPOINT_NAVIGATION.md           # 路点导航指南
│   ├── PORTABILITY.md                   # 跨平台适配说明
│   └── TROUBLESHOOTING.md               # 故障排查
├── requirements.txt                      # Python依赖列表
├── package.xml                           # ROS包配置
├── CMakeLists.txt                        # CMake构建配置
└── README.md                             # 本文件
```

## 📄 文件说明

| 文件 | 功能 |
|------|------|
| `clip_room_classifier.py` | 基于CLIP的房间类型分类 |
| `sam_segmentation.py` | SAM图像分割和实例提取 |
| `furniture_structure_detector.py` | 家具和结构元素识别 |
| `waypoint_generator.py` | PGM/YAML/XML路点生成 |
| `path_manager.py` | 跨平台路径自动检测 |
| `clip_sam_config.yaml` | 房间提示词、模型配置 |

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📜 许可证

MIT License

## 📞 联系方式

- 报告问题: 提交GitHub Issues
- 功能建议: Discussions

## 🙏 致谢

- CLIP: [OpenAI](https://github.com/openai/CLIP)
- SAM: [Meta Research](https://github.com/facebookresearch/segment-anything)
- ROS: [Open Source Robotics Foundation](https://www.ros.org/)
- wpr_simulation: [zhangwanjie](https://github.com/6-robot/wpr_simulation.git)

---

**版本:** 2.0 | **最后更新:** 2025年 | **状态:** 生产就绪
