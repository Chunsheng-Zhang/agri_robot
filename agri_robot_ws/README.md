# 农业机器人 ROS 2 单机器人核心

复用既有 MGX23、Yahboom 与 OpenNav/Fields2Cover 工作区，使用独立 overlay 集成。当前包含 `agri_base`、`agri_waypoint`、`agri_path_planning`、`agri_path_tracking`、`agri_safety`、`agri_bringup` 六个包。原始 Yahboom 代码不修改，直接启动 `Mcnamu_driver_X3` 并重映射其速度输入。

- [架构、源码分析与通信接口](../docs/ROS2核心架构与现有代码分析.md)
- [构建、启动与测试](../docs/ROS2核心使用说明.md)
- [室外实验指导与数据记录](../docs/室外实验指导文件.md)
- [默认系统参数](src/agri_bringup/config/system.yaml)
- [手柄、4ROS 和 Astra Pro Plus 接入](../docs/手柄雷达相机接入说明.md)
- [统一启动文件](src/agri_bringup/launch/system.launch.py)

在项目根目录执行：

```bash
bash scripts/build_core.bash
source scripts/setup_environment.bash
bash scripts/start_core.bash mode:=base chassis:=none
```

启动默认不连接底盘、不自动执行任务，并保持软件急停锁定。定位输入采用现有 TCP 配置；尾部字段保留原值，不声称已经判别 RTK/INS 解状态。具体运行前置条件和限制见文档。
