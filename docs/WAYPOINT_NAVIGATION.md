# 导航路点使用指南

## 概述

CLIP+SAM语义建图系统可自动生成**导航路点文件**，包括房间中心坐标和英文标签。这些路点可直接用于机器人自主导航。

## 输出文件格式

### 1. XML路点文件 (waypoints.xml)

```xml
<?xml version="1.0"?>
<waypoints>
  <metadata>
    <timestamp>2024-01-15T10:30:45Z</timestamp>
    <map_name>map_20240115_103045</map_name>
    <total_waypoints>8</total_waypoints>
  </metadata>
  
  <waypoint>
    <id>1</id>
    <name>Living_Room_Center</name>
    <room_type>living_room</room_type>
    <x>10.5</x>
    <y>8.3</y>
    <theta>0.0</theta>
    <confidence>0.95</confidence>
  </waypoint>
  
  <waypoint>
    <id>2</id>
    <name>Bedroom_Center</name>
    <room_type>bedroom</room_type>
    <x>15.2</x>
    <y>12.7</y>
    <theta>1.57</theta>
    <confidence>0.88</confidence>
  </waypoint>
  
  <!-- 更多路点... -->
</waypoints>
```

### 2. JSON路点文件 (waypoints.json)

```json
{
  "metadata": {
    "timestamp": "2024-01-15T10:30:45Z",
    "map_name": "map_20240115_103045",
    "total_waypoints": 8,
    "map_resolution": 0.05
  },
  "waypoints": [
    {
      "id": 1,
      "name": "Living_Room_Center",
      "room_type": "living_room",
      "position": {"x": 10.5, "y": 8.3},
      "orientation": 0.0,
      "confidence": 0.95
    },
    {
      "id": 2,
      "name": "Bedroom_Center",
      "room_type": "bedroom",
      "position": {"x": 15.2, "y": 12.7},
      "orientation": 1.57,
      "confidence": 0.88
    }
  ]
}
```

### 3. ROS地图文件

**map.pgm** - 灰度导航地图
```
P5
1024 768
255
[二进制数据]
```

**map.yaml** - 地图元数据
```yaml
image: map.pgm
resolution: 0.05          # 每像素0.05米
origin: [0.0, 0.0, 0.0]  # 左下角坐标
negate: 0                 # 0=白色空闲，黑色占用
occupied_thresh: 0.65
free_thresh: 0.196
```

## 加载和使用路点

### Python API

#### 加载路点

```python
import xml.etree.ElementTree as ET
import json

# 方法1：从XML加载
def load_waypoints_xml(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    
    waypoints = []
    for wp in root.findall('waypoint'):
        waypoint = {
            'id': int(wp.find('id').text),
            'name': wp.find('name').text,
            'room_type': wp.find('room_type').text,
            'x': float(wp.find('x').text),
            'y': float(wp.find('y').text),
            'theta': float(wp.find('theta').text),
            'confidence': float(wp.find('confidence').text)
        }
        waypoints.append(waypoint)
    
    return waypoints

# 方法2：从JSON加载
def load_waypoints_json(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data['waypoints']

# 使用示例
waypoints = load_waypoints_xml('results/waypoints/map_*/waypoints.xml')
for wp in waypoints:
    print(f"{wp['name']}: ({wp['x']:.2f}, {wp['y']:.2f})")
```

#### 发送导航目标

```python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import math

def send_navigation_goal(x, y, theta):
    """
    发送导航目标到move_base
    
    参数:
        x, y: 世界坐标
        theta: 目标方向（弧度）
    """
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0
    
    # 从欧拉角创建四元数
    quat = quaternion_from_euler(0, 0, theta)
    goal.pose.orientation = Quaternion(*quat)
    
    # 发布到导航栈
    goal_pub = rospy.Publisher('/move_base_simple/goal', 
                               PoseStamped, queue_size=1)
    goal_pub.publish(goal)

# 使用示例
rospy.init_node('waypoint_navigator')
waypoints = load_waypoints_xml('waypoints.xml')

for wp in waypoints:
    print(f"正在导航到 {wp['name']}")
    send_navigation_goal(wp['x'], wp['y'], wp['theta'])
    
    # 等待导航完成（需要实现检查逻辑）
    rospy.sleep(30)
```

### ROS服务接口

#### 创建自定义ROS服务

```python
# waypoint_service.py
import rospy
from clip_sam_semantic_mapping.srv import NavigateToWaypoint
import xml.etree.ElementTree as ET

class WaypointNavigator:
    def __init__(self):
        rospy.Service('navigate_to_waypoint', 
                      NavigateToWaypoint, 
                      self.navigate_callback)
        self.waypoints = self.load_waypoints()
    
    def load_waypoints(self):
        tree = ET.parse('waypoints.xml')
        waypoints = {}
        for wp in tree.findall('.//waypoint'):
            name = wp.find('name').text
            waypoints[name] = {
                'x': float(wp.find('x').text),
                'y': float(wp.find('y').text),
                'theta': float(wp.find('theta').text)
            }
        return waypoints
    
    def navigate_callback(self, req):
        if req.waypoint_name in self.waypoints:
            wp = self.waypoints[req.waypoint_name]
            # 调用导航栈
            self.send_goal(wp['x'], wp['y'], wp['theta'])
            return True
        return False
    
    def send_goal(self, x, y, theta):
        # 导航实现
        pass

if __name__ == '__main__':
    rospy.init_node('waypoint_navigator')
    navigator = WaypointNavigator()
    rospy.spin()
```

#### 使用服务

```bash
# 调用导航到客厅
rosservice call /navigate_to_waypoint \
  "waypoint_name: 'Living_Room_Center'"

# 调用导航到卧室
rosservice call /navigate_to_waypoint \
  "waypoint_name: 'Bedroom_Center'"
```

## 完整导航流程

### 集成流程图

```
语义建图系统
    ↓
生成 waypoints.xml
    ↓
加载到导航模块
    ↓
循环导航：
    ├─ 1. 读取下一个路点
    ├─ 2. 发送PoseStamped目标
    ├─ 3. 等待到达（监听 /amcl_pose）
    ├─ 4. 执行室内任务
    └─ 5. 返回第1步
    ↓
导航完成
```

### 完整示例：自主导航所有房间

```python
import rospy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import time

class AutonomousRoomNavigator:
    def __init__(self, waypoint_file):
        rospy.init_node('room_navigator')
        
        self.waypoints = self.load_waypoints(waypoint_file)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', 
                                       PoseStamped, queue_size=1)
        self.current_pose = None
        
        # 订阅当前位置
        rospy.Subscriber('/amcl_pose', 
                        PoseWithCovarianceStamped,
                        self.pose_callback)
        
        rospy.sleep(1)  # 等待连接建立
    
    def load_waypoints(self, filepath):
        tree = ET.parse(filepath)
        waypoints = []
        for wp in tree.findall('.//waypoint'):
            waypoints.append({
                'name': wp.find('name').text,
                'room_type': wp.find('room_type').text,
                'x': float(wp.find('x').text),
                'y': float(wp.find('y').text),
                'theta': float(wp.find('theta').text)
            })
        return waypoints
    
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def send_goal(self, x, y, theta):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        
        quat = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        
        self.goal_pub.publish(goal)
        rospy.loginfo(f"发送目标: ({x:.2f}, {y:.2f})")
    
    def distance_to_goal(self, goal_x, goal_y):
        if self.current_pose is None:
            return float('inf')
        
        dx = goal_x - self.current_pose.position.x
        dy = goal_y - self.current_pose.position.y
        return (dx**2 + dy**2) ** 0.5
    
    def wait_for_arrival(self, goal_x, goal_y, tolerance=0.3, timeout=120):
        """等待机器人到达目标"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            dist = self.distance_to_goal(goal_x, goal_y)
            
            if dist < tolerance:
                rospy.loginfo("✓ 已到达目标")
                return True
            
            rospy.loginfo(f"距离目标: {dist:.2f}m")
            rospy.sleep(1)
        
        rospy.logwarn("✗ 导航超时")
        return False
    
    def navigate_all_rooms(self):
        """导航访问所有房间"""
        rospy.loginfo(f"开始自主导航，共{len(self.waypoints)}个房间")
        
        for i, wp in enumerate(self.waypoints, 1):
            rospy.loginfo(f"\n[{i}/{len(self.waypoints)}] 前往: {wp['name']}")
            
            # 发送目标
            self.send_goal(wp['x'], wp['y'], wp['theta'])
            
            # 等待到达
            if self.wait_for_arrival(wp['x'], wp['y']):
                rospy.sleep(5)  # 在房间内停留5秒进行任务
                rospy.loginfo(f"完成: {wp['name']}")
            else:
                rospy.logwarn(f"无法到达: {wp['name']}")
        
        rospy.loginfo("\n✓ 所有房间导航完成！")

if __name__ == '__main__':
    navigator = AutonomousRoomNavigator('results/waypoints/map_20240115_103045/waypoints.xml')
    navigator.navigate_all_rooms()
```

## 路点优化

### 去重和平滑

```python
import numpy as np

def optimize_waypoints(waypoints, min_distance=0.5):
    """
    移除距离过近的重复路点
    
    参数:
        waypoints: 路点列表
        min_distance: 最小距离阈值（米）
    
    返回:
        优化后的路点列表
    """
    if not waypoints:
        return []
    
    optimized = [waypoints[0]]
    
    for wp in waypoints[1:]:
        last = optimized[-1]
        dist = ((wp['x'] - last['x'])**2 + 
                (wp['y'] - last['y'])**2) ** 0.5
        
        if dist >= min_distance:
            optimized.append(wp)
    
    return optimized

def smooth_path(waypoints, window_size=3):
    """使用滑动平均平滑路点"""
    if len(waypoints) < window_size:
        return waypoints
    
    smoothed = []
    half_window = window_size // 2
    
    for i in range(len(waypoints)):
        start = max(0, i - half_window)
        end = min(len(waypoints), i + half_window + 1)
        
        avg_x = sum(w['x'] for w in waypoints[start:end]) / (end - start)
        avg_y = sum(w['y'] for w in waypoints[start:end]) / (end - start)
        
        smoothed.append({
            **waypoints[i],
            'x': avg_x,
            'y': avg_y
        })
    
    return smoothed

# 使用示例
waypoints = load_waypoints_xml('waypoints.xml')
optimized = optimize_waypoints(waypoints, min_distance=0.5)
smoothed = smooth_path(optimized, window_size=3)
```

## 验证和测试

### 检查路点可达性

```python
def verify_waypoint_reachability(waypoints, map_yaml, costmap):
    """
    验证路点是否可到达
    
    参数:
        waypoints: 路点列表
        map_yaml: 地图YAML配置
        costmap: 成本地图
    
    返回:
        可达性报告
    """
    results = []
    
    for wp in waypoints:
        # 转换为地图坐标
        map_x = int((wp['x'] - map_yaml['origin'][0]) / map_yaml['resolution'])
        map_y = int((wp['y'] - map_yaml['origin'][1]) / map_yaml['resolution'])
        
        # 检查costmap值
        if costmap[map_y, map_x] > 0:  # 被占用
            results.append({
                'waypoint': wp['name'],
                'reachable': False,
                'reason': 'collision'
            })
        else:
            results.append({
                'waypoint': wp['name'],
                'reachable': True,
                'reason': 'ok'
            })
    
    return results
```

### 生成导航报告

```python
def generate_navigation_report(waypoints, results):
    """生成导航报告"""
    print("=" * 50)
    print("导航路点验证报告")
    print("=" * 50)
    print(f"总路点数: {len(waypoints)}")
    
    reachable = sum(1 for r in results if r['reachable'])
    unreachable = len(results) - reachable
    
    print(f"可达: {reachable} ({100*reachable/len(waypoints):.1f}%)")
    print(f"不可达: {unreachable} ({100*unreachable/len(waypoints):.1f}%)")
    print("-" * 50)
    
    for r in results:
        status = "✓" if r['reachable'] else "✗"
        print(f"{status} {r['waypoint']}: {r['reason']}")
```

## 参考资源

- 📖 [使用指南](USAGE.md)
- 📖 [ROS导航栈](http://wiki.ros.org/navigation)
- 📖 [move_base教程](http://wiki.ros.org/move_base)
- 📖 [仿真环境](SIMULATION_SETUP.md)

## 获取帮助

遇到导航问题？参考：
- 📖 [故障排查](TROUBLESHOOTING.md)
- 📖 [安装指南](INSTALLATION.md)
