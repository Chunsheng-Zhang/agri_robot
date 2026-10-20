
逻辑

> 读取目标点
>     ↓
> 获取 MGX23 当前位置和航向
>     ↓
> 计算当前位置到目标点的距离、方位角
>     ↓
> 判断是否到达目标点
>     ├── 未到达 → 执行方向控制
>     │
>     └── 已到达 → 停车
>                     ↓
>                  停留 5 秒
>                     ↓
>                  切换下一个目标点
>                     ↓
>                  继续导航
>                     ↓
>                  全部目标点完成 → 停车
                 

查看当前导航状态：
ros2 topic echo /multi_point_nav/status

查看当前目标点：
ros2 topic echo /multi_point_nav/current_target

查看输出速度：
ros2 topic echo /cmd_vel