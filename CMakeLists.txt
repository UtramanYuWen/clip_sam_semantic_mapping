cmake_minimum_required(VERSION 3.0.2)
project(clip_sam_semantic_mapping)

## 找到catkin和其他依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  tf
)

## Python setup
catkin_python_setup()

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs 
    cv_bridge 
    tf
)

## 安装Python脚本
catkin_install_python(PROGRAMS
  src/clip_sam_semantic_mapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## 安装RViz配置
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## 安装其他资源
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)