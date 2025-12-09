# 使用指南

## 快速开始

### 启动方式

#### 方式1：完整系统（推荐）

包含Gazebo仿真、SLAM建图和语义分析：

```bash
# 启动完整系统
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch

# 在另一终端启动RViz可视化（可选）
rviz -d src/clip_sam_semantic_mapping/rviz/hierarchical_semantic_mapping.rviz
```

此方法启动：
- Gazebo仿真环境（WPR Stage RoboCup）
- gmapping SLAM建图
- CLIP+SAM多层次语义建图
- RViz可视化
- 键盘控制节点

#### 方式2：仅语义建图

如果已有外部SLAM系统：

```bash
roslaunch clip_sam_semantic_mapping semantic_only.launch
```

#### 方式3：启动脚本

```bash
cd ~/catkin_ws/src/clip_sam_semantic_mapping
bash scripts/launch_complete_system.sh
```

#### 方式4：独立模式（无ROS）

处理本地图像文件：

```bash
python3 clip_sam_semantic_mapper.py \
  --input /path/to/images \
  --output results/ \
  --mode hierarchical
```

## 配置参数详解

### config/clip_sam_config.yaml

#### 基础配置

```yaml
# 系统名称
system_name: "CLIP+SAM Semantic Mapper"

# 是否启用GPU
cuda_enabled: true
device: "cuda:0"  # 或 "cpu"
```

#### 模型配置

```yaml
# CLIP模型选择
clip_model: "ViT-B/32"
# 可选值:
#   "ViT-B/32"   - 最快，内存少 (350MB)
#   "ViT-B/16"   - 平衡 (390MB)
#   "ViT-L/14"   - 最准确，内存多 (900MB)

# SAM模型选择
sam_model: "vit_b"
# 可选值:
#   "vit_b"      - 平衡 (375MB)
#   "vit_l"      - 更大 (1.2GB)
#   "vit_h"      - 最大 (2.5GB)
```

#### 房间识别

```yaml
# 房间类型和提示词（按优先级排序）
per_room_prompts:
  living_room:
    - "客厅"
    - "起居室"
    - "会客厅"
  
  bedroom:
    - "卧室"
    - "主卧"
    - "次卧"
    - "小卧室"
  
  kitchen:
    - "厨房"
    - "烹饪区"
  
  bathroom:
    - "卫生间"
    - "浴室"
    - "洗手间"
  
  study_room:
    - "书房"
    - "办公室"
    - "工作间"
  
  dining_room:
    - "餐厅"
    - "饭厅"
  
  balcony:
    - "阳台"
    - "露台"
  
  entrance_hall:
    - "玄关"
    - "入口"
    - "门厅"

# 房间名称英文映射（用于XML输出）
room_name_mapping:
  living_room: "Living_Room"
  bedroom: "Bedroom"
  kitchen: "Kitchen"
  bathroom: "Bathroom"
  study_room: "Study_Room"
  dining_room: "Dining_Room"
  balcony: "Balcony"
  entrance_hall: "Entrance_Hall"
```

#### 推理参数

```yaml
# CLIP置信度阈值 (0.0-1.0)
# 更低 = 更灵敏但可能误判
# 更高 = 更保守但可能漏检
confidence_threshold: 0.3

# IoU（交并比）阈值，用于去重
overlap_threshold: 0.5

# 每帧最大检测数
max_detections_per_frame: 100

# 处理分辨率
resolution: "720p"  # "360p", "480p", "720p", "1080p"

# 帧跳过数 (1=每帧处理，2=每两帧处理)
frame_skip: 1
```

#### 输出配置

```yaml
# 输出文件格式
output_format:
  - "png"   # 可视化图像
  - "pgm"   # 导航地图（ROS标准）
  - "yaml"  # 地图配置
  - "xml"   # 路点定义

# 结果保存目录
output_dir: "results"

# 保存中间结果
save_intermediate: false

# 实时可视化
visualization: true

# 显示性能统计
show_stats: true
```

#### 输入源配置

```yaml
# 输入方式: "camera" / "ros_topic" / "image_folder"
input_source: "camera"

# USB相机设备
camera_device: "/dev/video0"
camera_fps: 30

# ROS话题
image_topic: "/camera/rgb/image_raw"

# 图像文件夹
image_folder: "/path/to/images"
image_extension: "*.jpg"  # 支持 *.png, *.jpg, *.jpeg 等

# 输入分辨率
input_width: 1280
input_height: 720
```

#### 性能设置

```yaml
# 多进程数
num_workers: 4

# 批处理大小
batch_size: 1

# 启用模型量化（降低精度换取速度）
quantization_enabled: false

# GPU内存优化
memory_optimization: true
```

## ROS接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | RGB图像输入 |
| `/tf` | tf/tfMessage | 变换框架（可选） |

### 发布话题

| 话题 | 类型 | 发布频率 | 说明 |
|------|------|---------|------|
| `/semantic_map/room` | sensor_msgs/Image | 实时 | 房间语义地图 |
| `/semantic_map/furniture` | sensor_msgs/Image | 实时 | 家具语义地图 |
| `/semantic_map/structure` | sensor_msgs/Image | 实时 | 结构语义地图 |
| `/semantic_map/combined` | sensor_msgs/Image | 实时 | 组合语义地图 |
| `/semantic_annotations` | sensor_msgs/Image | 实时 | 标注图像 |
| `/waypoints` | geometry_msgs/PoseArray | 1Hz | 导航路点 |
| `/system_stats` | std_msgs/String | 1Hz | 性能统计 |

### 使用ROS话题

```python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # 处理图像...
    
rospy.init_node('semantic_map_subscriber')
rospy.Subscriber('/semantic_map/room', Image, image_callback)
rospy.spin()
```

## 命令行参数

### 主要参数

```bash
python3 clip_sam_semantic_mapper.py [options]

# 常用选项:
  --config FILE              配置文件路径
  --input PATH               输入图像/文件夹路径
  --output DIR               输出目录
  --device {cuda,cpu}        计算设备
  --mode {hierarchical,room} 处理模式
  --resolution {360,480,720} 处理分辨率
  --batch-size N             批处理大小
  --show-stats               显示性能统计
  --no-visualization         禁用可视化
  --save-intermediate        保存中间结果
```

### 示例

```bash
# 使用GPU处理USB相机，分辨率720p
python3 clip_sam_semantic_mapper.py \
  --device cuda \
  --resolution 720 \
  --show-stats

# 处理本地图像文件夹，保存所有输出
python3 clip_sam_semantic_mapper.py \
  --input /home/data/images \
  --output /home/data/results \
  --save-intermediate \
  --mode hierarchical

# 使用CPU进行测试
python3 clip_sam_semantic_mapper.py \
  --device cpu \
  --resolution 360
```

## 输出文件

### 文件结构

```
results/
├── semantic_maps/
│   └── map_YYYYMMDD_HHMMSS/
│       ├── room_semantic_001.png
│       ├── furniture_semantic_001.png
│       ├── structure_semantic_001.png
│       └── combined_semantic_001.png
│
└── waypoints/
    └── map_YYYYMMDD_HHMMSS/
        ├── map.pgm                 (ROS导航地图)
        ├── map.yaml                (地图配置)
        ├── waypoints.xml           (路点定义)
        └── room_centers.json       (原始中心坐标)
```

### PNG语义地图

**颜色编码：**

房间语义：
- 红色 (RGB: 255,0,0) - 客厅
- 绿色 (RGB: 0,255,0) - 卧室
- 蓝色 (RGB: 0,0,255) - 厨房
- 黄色 (RGB: 255,255,0) - 卫生间
- 品红 (RGB: 255,0,255) - 书房
- 青色 (RGB: 0,255,255) - 餐厅
- 浅红 (RGB: 255,128,128) - 阳台
- 灰色 (RGB: 128,128,128) - 玄关

### PGM/YAML地图格式

**map.pgm** - 灰度地图图像
```
P5
width height
255
[binary data]
```

**map.yaml** - 地图配置
```yaml
image: map.pgm
resolution: 0.05           # 每像素米数
origin: [0.0, 0.0, 0.0]  # 原点位置
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### XML路点格式

```xml
<?xml version="1.0"?>
<waypoints>
  <waypoint>
    <name>Living_Room_Center</name>
    <x>10.5</x>
    <y>8.3</y>
    <theta>0.0</theta>
    <room_type>living_room</room_type>
  </waypoint>
  
  <waypoint>
    <name>Bedroom_Center</name>
    <x>15.2</x>
    <y>12.7</y>
    <theta>1.57</theta>
    <room_type>bedroom</room_type>
  </waypoint>
  <!-- 其他路点... -->
</waypoints>
```

## 常见使用场景

### 场景1：实时室内建图

```bash
# 启动完整系统并实时处理相机输入
roslaunch clip_sam_semantic_mapping complete_system.launch \
  input_source:=camera \
  camera_device:=/dev/video0

# 在另一终端查看结果
rviz -d rviz/rviz_config.rviz
```

### 场景2：批量处理图像文件

```bash
python3 clip_sam_semantic_mapper.py \
  --input /home/data/images/ \
  --output /home/data/results/ \
  --device cuda \
  --save-intermediate
```

### 场景3：与SLAM集成

```bash
# 终端1：启动gmapping
rosrun gmapping slam_gmapping \
  scan:=/scan

# 终端2：启动语义建图（无内置SLAM）
roslaunch clip_sam_semantic_mapping semantic_only.launch

# 结果会融合来自gmapping的地图
```

### 场景4：导航路点生成

```bash
# 处理完成后自动生成路点
python3 clip_sam_semantic_mapper.py \
  --input /path/to/map \
  --output results/ \
  --mode hierarchical
  # waypoints/ 目录中会生成XML文件
```

## 性能优化

### 降低处理延迟

```bash
# 1. 降低分辨率
resolution: "360p"  # 在config中设置

# 2. 增加帧跳过
frame_skip: 2

# 3. 使用更小的CLIP模型
clip_model: "ViT-B/32"

# 4. 禁用可视化
visualization: false

# 5. 减少SAM分割点数
max_points_per_frame: 10
```

### 降低内存占用

```bash
# 1. 使用CPU
device: "cpu"

# 2. 启用量化
quantization_enabled: true

# 3. 减小批处理
batch_size: 1

# 4. 启用内存优化
memory_optimization: true
```

## 获取帮助

- 📖 参考 [故障排查](TROUBLESHOOTING.md)
- 🏠 查看 [仿真环境](SIMULATION_SETUP.md) 了解集成
- 🗺️ 参考 [路点导航](WAYPOINT_NAVIGATION.md) 了解导航功能
