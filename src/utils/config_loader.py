#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置加载工具 - 支持相对路径和绝对路径
"""

import yaml
import os
from typing import Dict, Any

class ConfigLoader:
    def __init__(self, config_path: str = None):
        if config_path is None:
            # 使用路径管理器获取配置路径
            try:
                from path_manager import get_path_manager
                pm = get_path_manager()
                config_path = pm.get_config_file('clip_sam_config.yaml')
            except ImportError:
                # 备选：使用相对路径
                current_dir = os.path.dirname(os.path.abspath(__file__))
                config_path = os.path.join(current_dir, "../../config/clip_sam_config.yaml")
        
        self.config_path = os.path.abspath(config_path)
        self.config = self.load_config()
    
    def load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            if not os.path.exists(self.config_path):
                print(f"⚠️  配置文件不存在: {self.config_path}")
                return self._get_default_config()
                
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            print(f"✅ 配置加载成功: {self.config_path}")
            return config
        except Exception as e:
            print(f"❌ 配置加载失败: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """默认配置"""
        return {
            'clip': {
                'model_name': 'ViT-B/32',
                'device': 'auto',
                'cache_dir': './models/clip_cache'
            },
            'sam': {
                'model_type': 'vit_b',
                'checkpoint_path': './models/sam_vit_b_01ec64.pth',
                'points_per_side': 16,
                'pred_iou_thresh': 0.7,
                'min_mask_region_area': 500
            },
            'inference': {
                'max_concurrent': 2,
                'queue_size': 10,
                'image_change_threshold': 0.2,
                'confidence_threshold': 0.2
            },
            'room_types': {
                'chinese': ["客厅", "卧室", "厨房", "卫生间"],
                'english': ["living room", "bedroom", "kitchen", "bathroom"]
            }
        }
    
    def get(self, key_path: str, default=None):
        """获取嵌套配置值 例: get('clip.model_name')"""
        keys = key_path.split('.')
        value = self.config
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def save_config(self, new_config: Dict[str, Any]):
        """保存配置"""
        try:
            with open(self.config_path, 'w', encoding='utf-8') as f:
                yaml.dump(new_config, f, default_flow_style=False, allow_unicode=True)
            self.config = new_config
            print(f"✅ 配置保存成功: {self.config_path}")
        except Exception as e:
            print(f"❌ 配置保存失败: {e}")

if __name__ == "__main__":
    # 测试配置加载
    config = ConfigLoader()
    print("CLIP模型:", config.get('clip.model_name'))
    print("SAM模型:", config.get('sam.model_type'))
    print("所有房间类型:", config.get('room_types.chinese'))
