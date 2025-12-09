#!/bin/bash

# CLIP+SAM 多层次语义建图系统 - 完整系统启动脚本
# 启动: Gazebo仿真 + gmapping建图 + CLIP+SAM语义建图 + RViz可视化

set -e

# 自动检测catkin工作空间
CATKIN_WS="${CATKIN_WS:-.}"
if [ ! -f "$CATKIN_WS/devel/setup.bash" ]; then
    # 尝试查找默认位置
    if [ -d "$HOME/catkin_ws" ]; then
        CATKIN_WS="$HOME/catkin_ws"
    elif [ -d "$HOME/..robot/catkin_ws" ]; then
        CATKIN_WS="/home/robot/catkin_ws"
    else
        echo "❌ 错误: 找不到catkin工作空间"
        echo "请设置 CATKIN_WS 环境变量或在工作空间根目录运行此脚本"
        exit 1
    fi
fi

PROJECT_ROOT="$CATKIN_WS/src/clip_sam_semantic_mapping"

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  🚀 CLIP+SAM 多层次语义建图系统 - 完整系统启动          ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# 激活ROS环境
echo "⚙️  激活ROS环境..."
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || \
  source /opt/ros/noetic/setup.bash 2>/dev/null || \
  source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "❌ 错误: ROS环境未找到"
    exit 1
  }

# 激活catkin工作空间
source "$CATKIN_WS/devel/setup.bash" || {
    echo "❌ 错误: 无法激活catkin工作空间"
    exit 1
}

echo "✓ 环境已激活"
echo ""

# 检查必要文件
echo "📁 检查项目文件..."
if [ ! -f "$PROJECT_ROOT/launch/hierarchical_clip_sam_mapping.launch" ]; then
    echo "❌ 错误: 找不到launch文件 (hierarchical_clip_sam_mapping.launch)"
    exit 1
fi
echo "✓ Launch文件存在"

if [ ! -f "$PROJECT_ROOT/config/clip_sam_config.yaml" ]; then
    echo "❌ 错误: 找不到配置文件 (clip_sam_config.yaml)"
    exit 1
fi
echo "✓ 配置文件存在"
echo ""

echo "═══════════════════════════════════════════════════════════"
echo ""
echo "🚀 即将启动完整系统（包含以下组件）:"
echo "  ✓ Gazebo 仿真环境 (WPR Stage RoboCup)"
echo "  ✓ 键盘控制节点（机器人移动）"
echo "  ✓ gmapping SLAM 建图"
echo "  ✓ CLIP+SAM 多层次语义建图"
echo "  ✓ RViz 可视化"
echo ""
echo "启动命令:"
echo "  roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""

# 启动系统
cd "$PROJECT_ROOT"
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch
