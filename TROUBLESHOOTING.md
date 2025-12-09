📌 快速问题解决卡

═══════════════════════════════════════════════════════════════════

❓ 问题：启动后只有 Gazebo 和 RViz，看不到语义地图话题

✅ 解决方案（按顺序尝试）：

1️⃣  检查语义建图程序是否运行
   ps aux | grep clip_sam_semantic_mapper
   
   如果没有进程，说明程序启动失败
   → 查看 launch 日志（见下方）

2️⃣  检查话题是否存在
   rostopic list | grep semantic
   
   应该看到：
   /semantic_map_room
   /semantic_map_furniture
   /semantic_map_structure
   
   如果没有 → 程序没有正常发布话题
   → 重启 launch 或查看程序日志

3️⃣  检查话题是否有数据
   rostopic hz /semantic_map_room
   
   应该看到 rate: XX.XX Hz
   
   如果显示 no rate（=0 Hz）
   → 话题没有数据发布
   → 检查 Kinect2 图像话题是否正常

4️⃣  在 RViz 中手动添加 Image
   
   步骤：
   a) 点击左下 "Add" 按钮
   b) 选择 "Image"
   c) 选择话题: /semantic_map_room
   
   看到图像？ → 成功！
   看不到？ → 继续下面步骤

5️⃣  查看程序日志
   
   查看最新日志：
   tail -f ~/.ros/log/latest/hierarchical_clip_sam_mapper-*.log
   
   查看是否有错误信息（ERROR、Exception）

═══════════════════════════════════════════════════════════════════

❓ 问题：保存脚本运行但没有保存地图

✅ 快速修复：

1. 检查保存脚本是否运行
   ps aux | grep save_semantic_maps.py

2. 检查保存目录权限
   ls -la ~/catkin_ws/src/clip_sam_semantic_mapping/results/
   
   应该是 rwx 权限

3. 检查话题数据
   rostopic echo /semantic_map_room -n 1
   
   应该看到 Image 数据

4. 重新运行保存脚本
   python3 src/clip_sam_semantic_mapping/scripts/save_semantic_maps.py

═══════════════════════════════════════════════════════════════════

❓ 问题：RViz 中的 Image 显示为空白或黑色

✅ 解决方案：

1. 调整 Max Value
   Image 组件 → Max Value: 改为 255

2. 调整 Transport Hint
   Image 组件 → Transport Hint: 选择 raw

3. 等待数据发布
   等待 5-10 秒，让 SLAM 积累足够数据

4. 检查地图是否有内容
   rostopic echo /semantic_map_room -n 1
   
   查看 width, height, data 是否非空

═══════════════════════════════════════════════════════════════════

❓ 问题：话题列表中完全看不到 /semantic_map_* 话题

✅ 排查清单：

□ 语义建图进程正在运行？
  ps aux | grep clip_sam | grep -v grep

□ 没有任何错误？
  查看终端输出，查看日志中是否有 ERROR

□ Kinect2 话题正在发布？
  rostopic list | grep kinect2
  应该看到多个 /kinect2 相关话题

□ SLAM 地图已生成？
  rostopic list | grep /map
  应该看到 /map 话题

如果以上都正常，说明程序启动但没有发布语义地图话题
→ 查看程序是否崩溃
→ 查看 CLIP 模型是否正确加载

═══════════════════════════════════════════════════════════════════

❓ 问题：launch 文件启动失败

❌ 错误信息：conda.sh: No such file or directory

✅ 修复方法：

这是 conda 环境路径错误，已在最新版本中修复
重新获取最新代码：

cd /home/robot/catkin_ws/src/clip_sam_semantic_mapping
git pull  # 或重新下载

再次运行：
roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch

═══════════════════════════════════════════════════════════════════

❓ 问题：Gazebo 启动很慢或卡顿

✅ 解决方案：

1. 关闭 Gazebo GUI（保持仿真）
   roslaunch clip_sam_semantic_mapping hierarchical_clip_sam_mapping.launch \
       gazebo_gui:=false start_rviz:=true

2. 等待更长时间（30 秒）让 Gazebo 初始化

3. 如果还是卡，重启系统或检查硬件资源

═══════════════════════════════════════════════════════════════════

❓ 问题：想停止程序

✅ 方法：

后台运行的程序：
pkill -f "roslaunch\|gazebo\|rviz\|clip_sam" 2>/dev/null

或者：
Ctrl+C （如果在前台运行）

清理所有 ROS 进程：
killall rosmaster roslaunch roscore

═══════════════════════════════════════════════════════════════════

❓ 问题：系统初始化报错

✅ 常见错误和修复：

错误: ModuleNotFoundError: No module named 'clip'
→ 激活 conda 环境：
   source /home/robot/anaconda3/etc/profile.d/conda.sh
   conda activate nav_system
   python3 -c "import clip; print('OK')"

错误: [ERROR] Cannot find package clip_sam_semantic_mapping
→ source 环境变量：
   cd /home/robot/catkin_ws
   source devel/setup.bash

错误: ROS master at http://localhost:11311 is not up yet
→ 启动 ROS master：
   roscore &

错误: rospy.exceptions.ROSException: ROS is not initialized
→ 检查 ROS 环境是否 source：
   echo $ROS_MASTER_URI
   应该看到：http://localhost:11311

═══════════════════════════════════════════════════════════════════

🔧 实用命令速查表

# 查看所有 ROS 话题
rostopic list

# 查看话题数据类型
rostopic type /semantic_map_room

# 查看话题发布频率
rostopic hz /semantic_map_room

# 查看一条话题消息
rostopic echo /semantic_map_room -n 1

# 查看所有运行的 ROS 节点
rosnode list

# 查看节点的发布和订阅话题
rosnode info hierarchical_clip_sam_mapper

# 查看 ROS 主机和连接状态
rostopic list -v

# 查看 ROS 日志
rosparam list

# 杀死所有 ROS 进程
killall -9 roslaunch roscore

═══════════════════════════════════════════════════════════════════

💡 有用的调试技巧

1. 同时打开多个终端（使用 tmux 或多个标签）

2. 在一个终端监听话题
   rostopic echo /semantic_map_room
   
3. 在另一个终端运行保存脚本
   python3 save_semantic_maps.py

4. 使用 rqt_graph 可视化话题连接
   rqt_graph

5. 使用 rosbag 录制和回放
   rosbag record -a
   rosbag play *.bag

═══════════════════════════════════════════════════════════════════

📞 如果还是无法解决

检查以下文件的详细说明：

- SIMULATION_GUIDE.md          ← 完整仿真指南
- RVIZ_AND_MAP_SAVING_GUIDE.md ← RViz 和保存地图详解
- LAUNCH_GUIDE.md              ← Launch 文件各种启动方式
- README_OPTIMIZATION.md       ← 性能优化说明
- QUICK_REFERENCE.txt          ← 快速参考

═══════════════════════════════════════════════════════════════════
