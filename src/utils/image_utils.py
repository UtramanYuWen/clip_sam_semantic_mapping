#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像处理工具
"""

import cv2
import numpy as np
from PIL import Image as PILImage
from typing import Tuple, Optional

class ImageProcessor:
    def __init__(self):
        pass
    
    @staticmethod
    def cv2_to_pil(cv_image: np.ndarray) -> Optional[PILImage.Image]:
        """CV2图像转PIL"""
        try:
            if len(cv_image.shape) == 3:
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = cv_image
            return PILImage.fromarray(rgb_image)
        except Exception as e:
            print(f"❌ CV2转PIL错误: {e}")
            return None
    
    @staticmethod
    def pil_to_cv2(pil_image: PILImage.Image) -> Optional[np.ndarray]:
        """PIL图像转CV2"""
        try:
            rgb_array = np.array(pil_image)
            if len(rgb_array.shape) == 3:
                bgr_image = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            else:
                bgr_image = rgb_array
            return bgr_image
        except Exception as e:
            print(f"❌ PIL转CV2错误: {e}")
            return None
    
    @staticmethod
    def resize_image(image: np.ndarray, target_size: int = 512) -> np.ndarray:
        """调整图像大小，保持宽高比"""
        try:
            height, width = image.shape[:2]
            
            if max(width, height) > target_size:
                if width > height:
                    scale = target_size / width
                    new_width = target_size
                    new_height = int(height * scale)
                else:
                    scale = target_size / height
                    new_height = target_size
                    new_width = int(width * scale)
                
                resized_image = cv2.resize(image, (new_width, new_height))
            else:
                resized_image = image
            
            return resized_image
        except Exception as e:
            print(f"❌ 图像缩放错误: {e}")
            return image
    
    @staticmethod
    def detect_image_change(current_image: np.ndarray, 
                          last_image: Optional[np.ndarray], 
                          threshold: float = 0.2) -> bool:
        """检测图像变化"""
        try:
            if last_image is None:
                return True
            
            # 缩小图像进行快速比较
            h, w = current_image.shape[:2]
            target_size = min(64, min(w//8, h//8))
            
            small_current = cv2.resize(current_image, (target_size, target_size))
            small_last = cv2.resize(last_image, (target_size, target_size))
            
            # 计算差异
            diff = cv2.absdiff(small_current, small_last)
            if len(diff.shape) == 3:
                diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
            else:
                diff_gray = diff
            
            # 计算变化像素比例
            changed_pixels = np.sum(diff_gray > 30)
            total_pixels = diff_gray.size
            change_ratio = changed_pixels / total_pixels
            
            return change_ratio > threshold
            
        except Exception as e:
            print(f"❌ 图像变化检测错误: {e}")
            return True
    
    @staticmethod
    def apply_mask_to_image(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """将掩码应用到图像"""
        try:
            masked_image = image.copy()
            if len(image.shape) == 3:
                masked_image[~mask] = [0, 0, 0]
            else:
                masked_image[~mask] = 0
            return masked_image
        except Exception as e:
            print(f"❌ 掩码应用错误: {e}")
            return image
    
    @staticmethod
    def draw_text_safe(image: np.ndarray, text: str, position: Tuple[int, int], 
                      color: Tuple[int, int, int] = (0, 255, 0), 
                      font_scale: float = 0.7) -> np.ndarray:
        """安全绘制文字（处理中文）"""
        try:
            # 中英文映射
            chinese_to_english = {
                "客厅": "Living Room", "卧室": "Bedroom", "厨房": "Kitchen",
                "卫生间": "Bathroom", "阳台": "Balcony", "书房": "Study",
                "餐厅": "Dining Room", "玄关": "Entrance", "未知": "Unknown"
            }
            
            display_text = text
            for chinese, english in chinese_to_english.items():
                display_text = display_text.replace(chinese, english)
            
            cv2.putText(image, display_text, position, 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)
            return image
            
        except Exception as e:
            print(f"❌ 文字绘制错误: {e}")
            return image

if __name__ == "__main__":
    # 测试图像工具
    processor = ImageProcessor()
    
    # 创建测试图像
    test_image = np.random.randint(0, 255, (100, 200, 3), dtype=np.uint8)
    
    # 测试转换
    pil_img = processor.cv2_to_pil(test_image)
    back_to_cv = processor.pil_to_cv2(pil_img)
    
    print(f"原图尺寸: {test_image.shape}")
    print(f"转换后尺寸: {back_to_cv.shape}")
    
    # 测试缩放
    resized = processor.resize_image(test_image, 50)
    print(f"缩放后尺寸: {resized.shape}")
    
    print("✅ 图像工具测试完成")
