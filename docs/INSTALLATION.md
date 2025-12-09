# 详细安装指南

## 前置条件

### 硬件要求
- **CPU**: Intel i5/Ryzen 5 或更好
- **内存**: 8GB RAM（推荐16GB+）
- **GPU**: NVIDIA GPU with CUDA Compute Capability 3.5+ （推荐）
- **存储**: 10GB 空闲空间

### 软件要求
- **OS**: Ubuntu 18.04 LTS (ROS Melodic) 或 20.04 LTS (ROS Noetic)
- **ROS**: 已完全安装（参考 [官方安装指南](http://wiki.ros.org/ROS/Installation)）
- **Python**: 3.7+
- **CUDA** (可选): 11.8+ 如需GPU支持

## 分步安装

### 方法1：自动安装（推荐）

#### 第1步：准备工作空间

```bash
# 创建catkin工作空间（如果还没有）
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 初始化工作空间
catkin_make

# 激活ROS环境
source /opt/ros/noetic/setup.bash  # 或 melodic
source devel/setup.bash
```

#### 第2步：克隆项目

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/clip_sam_semantic_mapping.git
cd clip_sam_semantic_mapping
```

#### 第3步：运行安装脚本

```bash
# 给脚本执行权限
chmod +x scripts/install_dependencies.sh

# 运行安装脚本
bash scripts/install_dependencies.sh
```

脚本会自动：
- ✓ 安装系统依赖（ROS cv-bridge、build tools等）
- ✓ 升级pip并安装Python包
- ✓ 下载CLIP和SAM模型
- ✓ 编译ROS包
- ✓ 验证安装

### 方法2：手动安装

如果自动脚本出现问题，按以下步骤手动安装：

#### 第1步：安装系统依赖

```bash
# 更新包列表
sudo apt-get update

# 安装基础工具
sudo apt-get install -y \
  python3-pip \
  python3-dev \
  build-essential \
  cmake \
  git \
  wget

# 安装ROS依赖
sudo apt-get install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-geometry-msgs \
  ros-noetic-sensor-msgs

# 对于ROS Melodic，替换 noetic 为 melodic
```

#### 第2步：安装Python依赖

```bash
# 升级pip
python3 -m pip install --upgrade pip

# 安装PyTorch (选择对应CUDA版本)

# GPU用户 (CUDA 11.8)
pip install torch==1.13.0 torchvision==0.14.0 --index-url https://download.pytorch.org/whl/cu118

# CPU用户
pip install torch==1.13.0 torchvision==0.14.0

# 安装其他依赖
pip install \
  clip \
  segment-anything \
  opencv-python \
  numpy \
  Pillow \
  scikit-image \
  scipy \
  pyyaml \
  rospkg \
  catkin_pkg
```

#### 第3步：下载AI模型

```bash
cd ~/catkin_ws/src/clip_sam_semantic_mapping
mkdir -p models

# 下载CLIP模型 (首次会自动下载到~/.cache/clip)
python3 -c "import clip; model, preprocess = clip.load('ViT-B/32', device='cpu')"

# 下载SAM模型
python3 -c "from segment_anything import sam_model_registry; model = sam_model_registry['vit_b']()"
```

#### 第4步：编译ROS包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### 第5步：验证安装

```bash
python3 << 'EOF'
import cv2
import numpy as np
import torch
import clip
from segment_anything import sam_model_registry

print("✓ cv2:", cv2.__version__)
print("✓ numpy:", np.__version__)
print("✓ torch:", torch.__version__)
print("✓ clip: OK")
print("✓ segment_anything: OK")
print("\n所有依赖已安装！")
EOF
```

## 配置

### 1. 设置ROS环境

在 `~/.bashrc` 中添加：

```bash
# ROS环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 可选：设置ROS_MASTER_URI（多机器人系统）
# export ROS_MASTER_URI=http://master_hostname:11311
# export ROS_IP=192.168.1.x
```

然后重新加载：
```bash
source ~/.bashrc
```

### 2. 配置CLIP+SAM系统

编辑 `config/clip_sam_config.yaml`:

```yaml
# CLIP模型选择
clip_model: "ViT-B/32"  # 选项: ViT-B/32, ViT-B/16, ViT-L/14

# SAM模型选择  
sam_model: "vit_b"  # 选项: vit_b, vit_l, vit_h

# 计算设备
device: "cuda:0"  # 或 "cpu"
cuda_enabled: true

# 推理参数
confidence_threshold: 0.3  # CLIP置信度
overlap_threshold: 0.5     # IoU阈值

# 输出格式
output_format: 
  - "pgm"  # ROS导航地图
  - "yaml" # 地图配置
  - "xml"  # XML路点
  - "png"  # 可视化

# 房间提示词
per_room_prompts:
  living_room: ["客厅", "起居室"]
  bedroom: ["卧室", "主卧"]
  # ... 其他房间
```

### 3. 配置输入源

根据使用场景：

**使用USB相机:**
```bash
# 检查相机设备
ls /dev/video*

# 配置config中的camera_device参数
camera_device: "/dev/video0"
```

**使用ROS话题:**
```bash
# 订阅现有相机话题
image_topic: "/camera/rgb/image_raw"
```

**处理图像文件夹:**
```bash
# 指定输入文件夹
input_folder: "/path/to/images"
image_extension: "*.png"
```

## 故障排查

### GPU相关问题

**问题：PyTorch无法使用GPU**
```bash
# 检查CUDA安装
nvidia-smi

# 检查PyTorch CUDA支持
python3 -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name())"

# 解决方案：重新安装正确的PyTorch版本
pip uninstall torch torchvision -y
pip install torch==1.13.0 torchvision==0.14.0 --index-url https://download.pytorch.org/whl/cu118
```

**问题：内存溢出**
```bash
# 使用CPU模式
python3 clip_sam_semantic_mapper.py --device cpu

# 或在config中设置
device: "cpu"
```

### 依赖相关问题

**问题：ModuleNotFoundError: No module named 'clip'**
```bash
# 重新安装CLIP
pip install --upgrade clip

# 或从源代码安装
pip install git+https://github.com/openai/CLIP.git
```

**问题：SAM模型下载失败**
```bash
# 手动下载模型
mkdir -p ~/.cache/segment_anything
cd ~/.cache/segment_anything
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth

# 设置环境变量
export SAM_CHECKPOINT_DIR=~/.cache/segment_anything
```

### ROS相关问题

**问题：roslaunch找不到包**
```bash
# 确保环境已激活
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 重新编译
cd ~/catkin_ws
catkin_make
```

**问题：cv_bridge版本不兼容**
```bash
# 检查cv_bridge版本
python3 -c "import cv_bridge; print(cv_bridge.__version__)"

# 重新安装
sudo apt-get install --reinstall ros-noetic-cv-bridge
```

## 验证安装

运行以下命令验证完整安装：

```bash
# 1. 检查Python环境
python3 -c "import sys; print(f'Python: {sys.version}')"

# 2. 检查关键模块
python3 << 'EOF'
modules = ['cv2', 'numpy', 'torch', 'clip', 'segment_anything', 'rospy', 'cv_bridge']
for m in modules:
    try:
        __import__(m)
        print(f"✓ {m}")
    except ImportError as e:
        print(f"✗ {m}: {e}")
EOF

# 3. 检查ROS环境
echo "ROS_DISTRO: $ROS_DISTRO"
rospack find clip_sam_semantic_mapping

# 4. 检查GPU（可选）
nvidia-smi

# 5. 运行简单测试
python3 -c "from clip_sam_semantic_mapper import SemanticMapper; print('✓ 主模块加载成功')"
```

## 下一步

安装完成后，参考以下文档继续：

- 📖 [使用指南](USAGE.md) - 运行系统
- 🏠 [仿真环境](SIMULATION_SETUP.md) - WPR Stage集成  
- 🗺️ [路点导航](WAYPOINT_NAVIGATION.md) - 导航功能
- 🐛 [故障排查](TROUBLESHOOTING.md) - 常见问题

## 获取帮助

如有问题，请：

1. 查看 [故障排查](TROUBLESHOOTING.md) 文档
2. 检查 [GitHub Issues](https://github.com/yourusername/clip_sam_semantic_mapping/issues)
3. 提交新Issue提供详细信息（错误信息、系统信息、复现步骤）
