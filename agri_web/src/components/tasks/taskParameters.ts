import type { MissionMode } from '../../api/workspace'
export interface ParameterField { key: string; label: string; unit?: string; min?: number; max?: number; step?: number; degrees?: boolean; choices?: Record<string, string>; hint?: string }
const shared: ParameterField[] = [
  { key: 'max_angular', label: '最大角速度', unit: 'rad/s', min: 0.01, max: 2, step: 0.05 },
]
export const parameterFields: Record<MissionMode, ParameterField[]> = {
  waypoint: [
    { key: 'max_linear', label: '最大行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.05 },
    { key: 'min_linear', label: '最小行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.01 }, ...shared,
    { key: 'arrival_tolerance', label: '到点距离阈值', unit: 'm', min: 0.02, max: 5, step: 0.05 },
    { key: 'dwell_time', label: '默认到点停留', unit: 's', min: 0, max: 600, step: 1 },
    { key: 'rotate_threshold', label: '开始原地转向阈值', unit: '°', min: 1, max: 180, step: 5, degrees: true },
    { key: 'align_threshold', label: '结束原地转向阈值', unit: '°', min: 1, max: 180, step: 2, degrees: true },
    { key: 'heading_kp', label: '航向控制增益', min: 0.01, max: 10, step: 0.1 },
    { key: 'distance_kp', label: '距离控制增益', min: 0.01, max: 10, step: 0.1 },
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
    { key: 'start_tolerance', label: '允许距路径起点距离', unit: 'm', min: 0.05, max: 5, step: 0.1 },
    { key: 'search_window', label: '路径进度搜索窗口', unit: 'm', min: 0.1, max: 10, step: 0.1 },
    { key: 'max_path_error', label: '最大路径偏离', unit: 'm', min: 0.05, max: 10, step: 0.1 },
  ],
}
