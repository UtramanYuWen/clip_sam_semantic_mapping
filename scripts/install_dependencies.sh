#!/bin/bash

# CLIP+SAM 语义建图系统 - 自动安装脚本
# 安装所有必需的系统和Python依赖，并下载AI模型

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}CLIP+SAM 语义建图系统 - 自动安装${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告: ROS环境未激活${NC}"
    echo "请先运行: source /opt/ros/noetic/setup.bash  # 或 melodic"
    # 尝试自动检测
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        ROS_DISTRO="noetic"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
        ROS_DISTRO="melodic"
    else
        exit 1
    fi
fi

echo -e "${GREEN}✓ 检测到ROS版本: $ROS_DISTRO${NC}"

# 获取Ubuntu版本
UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" == "18.04" ]]; then
    ROS_DISTRO_NAME="melodic"
elif [[ "$UBUNTU_VERSION" == "20.04" ]]; then
    ROS_DISTRO_NAME="noetic"
else
    echo -e "${RED}✗ 不支持的Ubuntu版本: $UBUNTU_VERSION${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Ubuntu版本: $UBUNTU_VERSION${NC}"

# ========== 第1步: 安装系统依赖 ==========
echo -e "\n${YELLOW}[1/4] 安装系统依赖...${NC}"

sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-pip \
    python3-dev \
    python3-venv \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-message-generation \
    build-essential \
    cmake \
    git \
    wget

echo -e "${GREEN}✓ 系统依赖安装完成${NC}"

# ========== 第2步: 升级pip并安装Python依赖 ==========
echo -e "\n${YELLOW}[2/4] 安装Python依赖...${NC}"

python3 -m pip install --upgrade pip setuptools wheel -q

# 检查requirements.txt是否存在
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
REQUIREMENTS_FILE="$PROJECT_ROOT/requirements.txt"

if [ -f "$REQUIREMENTS_FILE" ]; then
    pip install -r "$REQUIREMENTS_FILE" -q
    echo -e "${GREEN}✓ 从requirements.txt安装依赖完成${NC}"
else
    # 直接安装关键依赖
    echo -e "${YELLOW}  未找到requirements.txt，安装基础依赖...${NC}"
    pip install -q \
        torch==1.13.0 \
        torchvision==0.14.0 \
        clip \
        segment-anything \
        opencv-python \
        numpy \
        Pillow \
        scikit-image \
        scipy \
        pyyaml
    echo -e "${GREEN}✓ 基础依赖安装完成${NC}"
fi

# ========== 第3步: 创建模型目录并下载模型 ==========
echo -e "\n${YELLOW}[3/4] 下载AI模型...${NC}"

MODELS_DIR="$PROJECT_ROOT/models"
mkdir -p "$MODELS_DIR"
cd "$MODELS_DIR"

# 下载CLIP模型
echo -e "${YELLOW}  正在下载CLIP ViT-B/32模型 (~350MB)...${NC}"
python3 << 'EOF'
try:
    import clip
    device = "cpu"
    print("  正在加载CLIP模型...")
    model, preprocess = clip.load("ViT-B/32", device=device)
    print("  ✓ CLIP ViT-B/32 模型已下载")
except Exception as e:
    print(f"  ✗ CLIP模型下载失败: {e}")
    exit(1)
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ CLIP模型下载失败${NC}"
    exit 1
fi

# 下载SAM模型
echo -e "${YELLOW}  正在下载SAM ViT-B模型 (~375MB)...${NC}"
python3 << 'EOF'
try:
    from segment_anything import sam_model_registry
    import os
    
    model_name = 'vit_b'
    model_path = f'sam_{model_name}_01ec64.pth'
    
    print(f"  正在加载{model_name}模型...")
    # sam_model_registry会自动下载模型到缓存目录
    model = sam_model_registry[model_name]()
    print(f"  ✓ SAM {model_name} 模型已下载")
except Exception as e:
    print(f"  ✗ SAM模型下载失败: {e}")
    exit(1)
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ SAM模型下载失败${NC}"
    exit 1
fi

echo -e "${GREEN}✓ AI模型下载完成${NC}"

# ========== 第4步: 验证安装 ==========
echo -e "\n${YELLOW}[4/4] 验证安装...${NC}"

python3 << 'EOF'
import sys

modules = ['cv2', 'numpy', 'torch', 'clip', 'segment_anything']
failed = []

for module in modules:
    try:
        __import__(module)
        print(f"  ✓ {module}")
    except ImportError:
        print(f"  ✗ {module}")
        failed.append(module)

if failed:
    print(f"\n✗ 缺少模块: {', '.join(failed)}")
    sys.exit(1)
else:
    print("\n✓ 所有关键模块已安装")
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ 验证失败${NC}"
    exit 1
fi

# ========== 完成 ==========
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}✓ 安装完成！${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${YELLOW}后续步骤:${NC}"
echo "1. 激活环境:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -z "$CATKIN_WS" ]; then
    echo "   source $CATKIN_WS/devel/setup.bash"
fi
echo ""
echo "2. 启动系统 (完整系统，包含仿真):"
echo "   roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch"
echo ""
echo "3. 或启动仅语义建图 (需要外部SLAM源):"
echo "   roslaunch clip_sam_semantic_mapping semantic_only.launch"
echo ""
echo "4. 更多信息，参考 README.md"
echo ""