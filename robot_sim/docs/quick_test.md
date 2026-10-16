# MGX23 路径跟踪系统 - 快速验证流程

## 完整的数据链路

```
mgx23_csv_replay → coordinate_converter → real_pure_pursuit → virtual_vehicle
     ↓                    ↓                      ↓                    ↓
  发布 GPS            GPS 转局部坐标        路径跟踪计算        接收速度指令
  姿态、速度          发布局部位置          发布 cmd_vel        打印验证
```

---

## 方法 1: 使用 launch 文件（最简单）

### 步骤 1: 启动虚拟车辆
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 launch agri_vehicle_sim vehicle_sim.launch.py
```

### 步骤 2: 启动 MGX23 数据回放
```bash
cd ~/agri_robot/mgx23_ws
source install/setup.bash
ros2 run mgx23_driver mgx23_csv_replay
```

### 步骤 3: 启动坐标转换（固定原点）
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_localization coordinate_converter --ros-args \
  -p use_first_fix_as_origin:=false \
  -p origin_latitude:=36.64684617892886 \
  -p origin_longitude:=117.02130432309536
```

### 步骤 4: 启动路径发布
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_path_planning path_publisher
```

### 步骤 5: 启动路径跟踪
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_path_tracking real_pure_pursuit --ros-args \
  -p lookahead_distance:=2.0 \
  -p linear_speed:=0.25 \
  -p max_path_deviation:=20.0
```

---

## 方法 2: 使用一键启动脚本

```bash
# 终端 1: 启动虚拟车辆
ros2 launch agri_vehicle_sim vehicle_sim.launch.py &

# 终端 2: 启动 MGX23 数据回放 + 坐标转换 + 路径跟踪
~/agri_robot/robot_sim/scripts/start_quick.sh
```

---

## 预期输出

### 虚拟车辆节点应该显示：
```
[INFO] [virtual_vehicle]: 虚拟车辆节点已启动
[INFO] [virtual_vehicle]: 监听 /cmd_vel 话题...
[INFO] [virtual_vehicle]: 收到速度指令：linear.x=0.250 m/s, angular.z=0.100 rad/s
[INFO] [virtual_vehicle]: 收到速度指令：linear.x=0.250 m/s, angular.z=0.120 rad/s
...
```

### coordinate_converter 应该显示：
```
[INFO] [coordinate_converter]: 固定原点：lat=36.64684618, lon=117.02130432
[INFO] [coordinate_converter]: UTM 分区：50，EPSG：32650
[INFO] [coordinate_converter]: 局部位置：x=0.123 m, y=-0.456 m
...
```

### real_pure_pursuit 应该显示：
```
[INFO] [real_pure_pursuit]: 收到参考路径：802 个点，frame_id=local_map
[INFO] [real_pure_pursuit]: 前视点距离：2.34 m，曲率：0.05 1/m
...
```

---

## 验证检查点

### 1. 检查所有节点是否运行
```bash
ros2 node list
```
应该看到：
- `/mgx23_csv_replay`
- `/coordinate_converter`
- `/path_publisher`
- `/real_pure_pursuit`
- `/virtual_vehicle`

### 2. 检查话题是否正常
```bash
ros2 topic list
```
应该看到：
- `/mgx23/position`
- `/mgx23/attitude`
- `/agri_localization/position`
- `/reference_path`
- `/cmd_vel`
- `/real_tracked_path`

### 3. 查看速度指令
```bash
ros2 topic echo /cmd_vel
```
应该看到连续的速度输出：
```yaml
linear:
  x: 0.25
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.123
```

### 4. 查看跟踪轨迹
```bash
ros2 topic echo /real_tracked_path | head -30
```

---

## 常见问题

### Q1: 虚拟车辆收不到速度指令
**检查**：
```bash
ros2 topic echo /cmd_vel
```
如果没有输出，说明 `real_pure_pursuit` 没有发布，检查其日志。

### Q2: 显示"车辆偏离路径过远"
**解决**：增大阈值
```bash
ros2 param set /real_pure_pursuit max_path_deviation 20.0
```

### Q3: 坐标转换节点报错
**检查**：确保 GPS 数据正常
```bash
ros2 topic echo /mgx23/position | head -10
```

---

## 停止所有节点

```bash
# 停止 launch 启动的节点
killall -9 gzserver rviz2 virtual_vehicle 2>/dev/null

# 停止其他节点
killall -9 mgx23_csv_replay path_publisher coordinate_converter real_pure_pursuit 2>/dev/null

# 或者简单地按 Ctrl+C 在每个终端
```

---

## 下一步

验证通过后，可以：
1. 替换 `virtual_vehicle` 为真实的底盘驱动节点
2. 替换 `mgx23_csv_replay` 为真实的 MGX23 硬件驱动
3. 添加 RViz 可视化
4. 调整路径跟踪参数优化性能
