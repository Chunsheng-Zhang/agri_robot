# MGX23 组合惯导 + 路径跟踪系统使用说明

## 系统配置

### 固定坐标原点
- **纬度**: 36.64684617892886
- **经度**: 117.02130432309536

这是场地中某个固定参考点的 GPS 坐标，所有局部坐标都将以此为原点 (0, 0)。

---

## 快速启动

### 方法 1: 使用启动脚本（推荐）

```bash
cd ~/agri_robot/robot_sim/scripts
./start_system_fixed_origin.sh
```

这将自动打开 4 个终端窗口，分别运行：
1. MGX23 数据回放
2. 坐标转换（固定原点）
3. 路径发布
4. Pure Pursuit 路径跟踪

### 方法 2: 手动启动（分终端）

#### 终端 1: MGX23 数据回放
```bash
cd ~/agri_robot/mgx23_ws
source install/setup.bash
ros2 run mgx23_driver mgx23_csv_replay
```

#### 终端 2: 坐标转换（固定原点）
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_localization coordinate_converter --ros-args \
  -p use_first_fix_as_origin:=false \
  -p origin_latitude:=36.64684617892886 \
  -p origin_longitude:=117.02130432309536
```

#### 终端 3: 路径发布
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_path_planning path_publisher
```

#### 终端 4: 路径跟踪
```bash
cd ~/agri_robot/robot_sim
source install/setup.bash
ros2 run agri_path_tracking real_pure_pursuit --ros-args \
  -p lookahead_distance:=2.0 \
  -p linear_speed:=0.25 \
  -p max_angular_speed:=0.6 \
  -p max_path_deviation:=5.0
```

---

## 数据流

```
┌──────────────────────┐
│  mgx23_csv_replay    │
│  (MGX23 数据回放)     │
│                      │
│  发布：              │
│  - /mgx23/position   │ (NavSatFix)
│  - /mgx23/attitude   │ (Vector3)
│  - /mgx23/velocity   │ (Vector3)
└──────────┬───────────┘
           │
           ├─────────────────────────────┐
           │                             │
           ▼                             ▼
──────────────────────┐    ┌──────────────────────┐
│  coordinate_converter│    │   path_publisher     │
│  (坐标转换)           │    │   (路径发布)         │
│                      │    │                      │
│  订阅：              │    │  发布：              │
│  - /mgx23/position   │    │  - /reference_path   │
│                      │    │                      │
│  发布：              │    └──────────┬───────────┘
│  - /agri_localization/position     │
└──────────┬───────────┘              │
           │                          │
           └──────────┬───────────────┘
                      │
                      ▼
          ┌──────────────────────┐
          │  real_pure_pursuit   │
          │  (路径跟踪)          │
          │                      │
          │  订阅：              │
          │  - /reference_path   │
          │  - /agri_localization/position │
          │  - /mgx23/attitude   │
          │                      │
          │  发布：              │
          │  - /cmd_vel          │
          │  - /real_tracked_path│
          └──────────────────────┘
```

---

## 参数配置

### 坐标转换节点 (coordinate_converter)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_first_fix_as_origin` | `false` | 是否使用第一条 GPS 作为原点 |
| `origin_latitude` | 36.64684617892886 | 固定原点纬度 |
| `origin_longitude` | 117.02130432309536 | 固定原点经度 |

### 路径跟踪节点 (real_pure_pursuit)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lookahead_distance` | 2.0 | 前视距离 (米)，越大转向越平缓 |
| `linear_speed` | 0.25 | 线速度 (m/s) |
| `max_angular_speed` | 0.6 | 最大角速度 (rad/s) |
| `max_path_deviation` | 5.0 | 路径偏离阈值 (米) |
| `goal_tolerance` | 0.25 | 到达终点阈值 (米) |
| `control_rate` | 20.0 | 控制频率 (Hz) |

---

## 调试与监控

### 查看运行中的节点
```bash
ros2 node list
```

### 查看速度指令
```bash
ros2 topic echo /cmd_vel
```

### 查看车辆位置
```bash
ros2 topic echo /agri_localization/position
```

### 查看跟踪轨迹
```bash
ros2 topic echo /real_tracked_path
```

### 查看节点日志
```bash
ros2 node info /real_pure_pursuit
```

---

## 常见问题

### Q1: 车辆偏离路径过远
**现象**: 出现警告 `车辆偏离路径过远：XX m`

**原因**: 坐标系原点不匹配

**解决**: 
- 确保 `coordinate_converter` 使用固定原点
- 确保路径文件的原点与 GPS 原点对应

### Q2: 收不到路径消息
**现象**: 路径跟踪节点提示收不到 `/reference_path`

**解决**:
```bash
# 检查 path_publisher 是否运行
ros2 topic list | grep reference_path

# 手动测试
ros2 topic echo /reference_path
```

### Q3: 车辆不移动
**现象**: `/cmd_vel` 没有输出或始终为 0

**检查**:
1. 确认收到路径：`ros2 topic echo /reference_path | head -20`
2. 确认收到位置：`ros2 topic echo /agri_localization/position | head -10`
3. 确认收到姿态：`ros2 topic echo /mgx23/attitude | head -10`
4. 检查偏离距离是否超过阈值

---

## 配置文件

- **系统配置**: `~/agri_robot/robot_sim/config/system_config.yaml`
- **启动脚本**: `~/agri_robot/robot_sim/scripts/start_system_fixed_origin.sh`

---

## 更新日期
2026-10-14
