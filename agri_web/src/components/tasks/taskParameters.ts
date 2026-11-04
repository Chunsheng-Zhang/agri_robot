import type { MissionMode } from '../../api/workspace'
export interface ParameterField { key: string; label: string; unit?: string; min?: number; max?: number; step?: number; degrees?: boolean; choices?: Record<string, string>; hint?: string }
const shared: ParameterField[] = [
  { key: 'max_angular', label: '最大角速度', unit: 'rad/s', min: 0.01, max: 2, step: 0.05 },
]
export const parameterFields: Record<MissionMode, ParameterField[]> = {
  waypoint: [
    { key: 'max_linear', label: '最大行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.05 },
    { key: 'min_linear', label: '最小行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.01 }, ...shared,
    { key: 'arrival_tolerance', label: '到点距离阈值', unit: 'm', min: 0.02, max: 5, step: 0.01, hint: '与目标距离不超过此值时停车并进入停留；不是独立测量的定位精度' },
    { key: 'dwell_time', label: '默认到点停留', unit: 's', min: 0, max: 600, step: 1 },
    { key: 'rotate_threshold', label: '开始原地转向阈值', unit: '°', min: 1, max: 180, step: 5, degrees: true, hint: '航向误差达到此值时停止前进、原地对齐；默认约57.3°，必须大于结束阈值' },
    { key: 'align_threshold', label: '结束原地转向阈值', unit: '°', min: 1, max: 180, step: 2, degrees: true, hint: '原地转向期间误差降至此值时恢复前进；默认约11.5°，两个阈值之间保持原状态' },
    { key: 'heading_kp', label: '航向控制增益', min: 0.01, max: 10, step: 0.1, hint: '角速度=增益×航向误差(rad)，再限幅；越大纠偏越快，但可能增加摆动' },
    { key: 'distance_kp', label: '距离控制增益', min: 0.01, max: 10, step: 0.1, hint: '基础线速度=增益×目标距离，再限幅并按航向偏差减速；越大越晚开始降速' },
  ],
  coverage: [
    { key: 'operation_width', label: '作业幅宽', unit: 'm', min: 0.1, max: 20, step: 0.1 },
    { key: 'min_turning_radius', label: '最小转弯半径', unit: 'm', min: 0.05, max: 20, step: 0.1, hint: '用于前进地头曲线；履带原地转向不等同于零半径 Dubins 路径' },
    { key: 'headland_width', label: '地头预留宽度', unit: 'm', min: 0, max: 50, step: 0.5 },
    { key: 'swath_angle', label: '作业方向', unit: '°', min: -180, max: 180, step: 5, degrees: true, hint: '网格东向为 0°，逆时针为正；不是设备北零航向' },
    { key: 'route_mode', label: '条带作业顺序', choices: { BOUSTROPHEDON: '牛耕', SNAKE: '蛇形', SPIRAL: '螺旋' } },
    { key: 'angle_mode', label: '作业方向选择', choices: { SET_ANGLE: '指定方向', BRUTE_FORCE: '自动搜索方向' } },
    { key: 'path_continuity', label: 'Dubins 转弯曲率', choices: { CONTINUOUS: '连续曲率', DISCONTINUOUS: '圆弧与直线' } },
    { key: 'sampling_distance', label: '路径采样间距', unit: 'm', min: 0.02, max: 1, step: 0.02 },
    { key: 'lookahead', label: 'Pure Pursuit 前视距离', unit: 'm', min: 0.1, max: 10, step: 0.1 },
    { key: 'speed', label: '目标行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.05 }, ...shared,
    { key: 'completion_tolerance', label: '终点完成阈值', unit: 'm', min: 0.02, max: 2, step: 0.05 },
    { key: 'start_tolerance', label: '直接跟踪起点距离', unit: 'm', min: 0.05, max: 5, step: 0.1, hint: '超过此距离会先单点导航到覆盖起点，再自动跟踪；不限制单点接近的总距离' },
    { key: 'search_window', label: '路径进度搜索窗口', unit: 'm', min: 0.1, max: 10, step: 0.1 },
    { key: 'max_path_error', label: '最大路径偏离', unit: 'm', min: 0.05, max: 10, step: 0.1 },
  ],
}
