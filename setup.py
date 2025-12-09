#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 为catkin生成Python包设置
d = generate_distutils_setup(
    packages=['clip_sam_semantic_mapping'],
    package_dir={'': 'src'},
    install_requires=[
        'torch>=1.9.0',
        'torchvision>=0.10.0',
        'opencv-python>=4.5.0',
        'pillow>=8.0.0',
        'numpy>=1.19.0',
        'scipy>=1.7.0',
        'scikit-image>=0.18.0',
    ],
)

setup(**d)