#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径管理工具 - 统一处理所有相对路径和绝对路径
确保项目在任何位置都能正确运行
"""

import os
import sys
try:
    import rospkg
    HAS_ROSPKG = True
except ImportError:
    HAS_ROSPKG = False


class PathManager:
    """路径管理器 - 解决路径可移植性问题"""
    
    _instance = None
    _package_root = None
    
    def __new__(cls):
        """单例模式"""
        if cls._instance is None:
            cls._instance = super(PathManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        """初始化路径管理器"""
        if self._initialized:
            return
        
        self._initialized = True
        self._determine_package_root()
    
    def _determine_package_root(self):
        """确定包的根目录"""
        # 优先方法 1：使用 rospkg
        if HAS_ROSPKG:
            try:
                rospack = rospkg.RosPack()
                self._package_root = rospack.get_path('clip_sam_semantic_mapping')
                print(f"✅ 使用 rospkg 确定包路径: {self._package_root}")
                return
            except Exception as e:
                print(f"⚠️ rospkg 失败 ({e})，尝试其他方法")
        
        # 方法 2：使用 ROS_PACKAGE_PATH 环境变量
        if 'ROS_PACKAGE_PATH' in os.environ:
            ros_paths = os.environ['ROS_PACKAGE_PATH'].split(':')
            for ros_path in ros_paths:
                potential_root = os.path.join(ros_path, 'clip_sam_semantic_mapping')
                if os.path.isdir(potential_root):
                    self._package_root = potential_root
                    print(f"✅ 使用环境变量确定包路径: {self._package_root}")
                    return
        
        # 方法 3：基于当前脚本位置推断
        current_file = os.path.abspath(__file__)
        # 从 src/utils/path_manager.py 向上回溯到 clip_sam_semantic_mapping/
        current_dir = os.path.dirname(current_file)  # src/utils/
        current_dir = os.path.dirname(current_dir)   # src/
        current_dir = os.path.dirname(current_dir)   # clip_sam_semantic_mapping/
        
        if os.path.isdir(os.path.join(current_dir, 'src')) and \
           os.path.isdir(os.path.join(current_dir, 'config')):
            self._package_root = current_dir
            print(f"✅ 使用脚本位置确定包路径: {self._package_root}")
            return
        
        # 方法 4：标准 catkin 结构推断
        if 'catkin_ws' in current_file:
            parts = current_file.split('catkin_ws')
            if len(parts) >= 2:
                catkin_root = parts[0] + 'catkin_ws'
                potential_root = os.path.join(catkin_root, 'src', 'clip_sam_semantic_mapping')
                if os.path.isdir(potential_root):
                    self._package_root = potential_root
                    print(f"✅ 使用 catkin 结构确定包路径: {self._package_root}")
                    return
        
        # 最后备选：使用当前工作目录
        self._package_root = os.getcwd()
        print(f"⚠️ 使用当前工作目录作为包路径: {self._package_root}")
    
    def get_package_root(self):
        """获取包根目录"""
        return self._package_root
    
    def get_config_dir(self):
        """获取配置目录"""
        return os.path.join(self._package_root, 'config')
    
    def get_src_dir(self):
        """获取源代码目录"""
        return os.path.join(self._package_root, 'src')
    
    def get_models_dir(self):
        """获取模型目录"""
        return os.path.join(self._package_root, 'models')
    
    def get_results_dir(self):
        """获取结果目录"""
        results_dir = os.path.join(self._package_root, 'results')
        os.makedirs(results_dir, exist_ok=True)
        return results_dir
    
    def get_semantic_maps_dir(self):
        """获取语义地图保存目录"""
        maps_dir = os.path.join(self.get_results_dir(), 'semantic_maps')
        os.makedirs(maps_dir, exist_ok=True)
        return maps_dir
    
    def get_waypoints_dir(self):
        """获取航点导航文件保存目录"""
        waypoints_dir = os.path.join(self.get_results_dir(), 'waypoints')
        os.makedirs(waypoints_dir, exist_ok=True)
        return waypoints_dir
    
    def get_analysis_dir(self):
        """获取分析数据目录"""
        analysis_dir = os.path.join(self.get_results_dir(), 'analysis')
        os.makedirs(analysis_dir, exist_ok=True)
        return analysis_dir
    
    def get_logs_dir(self):
        """获取日志目录"""
        logs_dir = os.path.join(self.get_results_dir(), 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        return logs_dir
    
    def get_config_file(self, filename='clip_sam_config.yaml'):
        """获取配置文件路径"""
        config_path = os.path.join(self.get_config_dir(), filename)
        if not os.path.exists(config_path):
            print(f"⚠️ 配置文件不存在: {config_path}")
        return config_path
    
    def get_model_file(self, filename):
        """获取模型文件路径"""
        model_path = os.path.join(self.get_models_dir(), filename)
        if not os.path.exists(model_path):
            print(f"⚠️ 模型文件不存在: {model_path}")
        return model_path
    
    def get_temp_dir(self):
        """获取临时目录"""
        import tempfile
        temp_dir = os.path.join(tempfile.gettempdir(), 'clip_sam_frames')
        os.makedirs(temp_dir, exist_ok=True)
        return temp_dir
    
    def ensure_dir_exists(self, dir_path):
        """确保目录存在"""
        os.makedirs(dir_path, exist_ok=True)
        return dir_path
    
    def print_structure(self):
        """打印项目结构信息"""
        print("\n🗂️ CLIP+SAM 语义建图系统 - 目录结构:")
        print(f"  📦 包根目录: {self._package_root}")
        print(f"  📄 配置文件: {self.get_config_dir()}")
        print(f"  🤖 模型文件: {self.get_models_dir()}")
        print(f"  📊 结果输出:")
        print(f"     ├─ 语义地图: {self.get_semantic_maps_dir()}")
        print(f"     ├─ 航点导航: {self.get_waypoints_dir()}")
        print(f"     ├─ 分析数据: {self.get_analysis_dir()}")
        print(f"     └─ 运行日志: {self.get_logs_dir()}")


# 全局单例实例
_path_manager = PathManager()


def get_path_manager():
    """获取全局路径管理器实例"""
    return _path_manager


if __name__ == '__main__':
    # 测试路径管理器
    pm = get_path_manager()
    pm.print_structure()
