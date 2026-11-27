import type { MissionMode } from '../../api/workspace'
export interface ParameterField { key: string; label: string; unit?: string; min?: number; max?: number; step?: number; degrees?: boolean; choices?: Record<string, string>; toggle?: boolean; hint?: string }
export const trackingControllerChoices: Record<string, string> = {
  pp: 'PP（速度自适应前视）', stanley: 'Stanley', lqr: 'LQR', mpc: 'MPC',
}
const shared: ParameterField[] = [
  { key: 'max_angular', label: '最大角速度', unit: 'rad/s', min: 0.01, max: 2, step: 0.05 },
]
export const coverageQualityFields: ParameterField[] = [
  { key: 'quality_resolution', label: '覆盖评价网格间距', unit: 'm', min: 0.01, max: 0.5, step: 0.01, hint: '默认0.05m；地块较大时会按计算预算加粗，实际间距与两级网格敏感性写入报告' },
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
    { key: 'controller_algorithm', label: '路径跟踪算法', choices: trackingControllerChoices,
      hint: '下拉选择只修改任务配置，开始任务时才加载并确认实际算法。修改后需保存并重新规划或加载匹配路径；运行或暂停期间不可切换。PP为仿真迁移版，四种算法共用停车转向和航向对齐流程' },
    { key: 'robot_width', label: '车体宽度 robot_width', unit: 'm', min: 0.05, max: 20, step: 0.01, hint: '默认0.2m；手工曲线按半车宽检查，智能原地转向按车体长宽的完整旋转包络检查。修改后保存并重新规划' },
    { key: 'robot_length', label: '车体包络长度', unit: 'm', min: 0.05, max: 20, step: 0.01, hint: '默认0.45m为估计值；填写以旋转中心为基准的车辆及随车机具包络尺寸' },
    { key: 'rotation_safety_margin', label: '原地转向安全余量', unit: 'm', min: 0, max: 5, step: 0.01, hint: '默认0.05m；智能规划按车体对角线半径加此余量设置最小安全地头' },
    { key: 'operation_width', label: '机具有效作业宽度', unit: 'm', min: 0.1, max: 20, step: 0.1, hint: '人工填写实际有效作业宽度；智能规划同时用于条带间距与覆盖评价，不自动调整' },
    { key: 'min_turning_radius', label: '最小转弯半径', unit: 'm', min: 0.05, max: 20, step: 0.1, hint: '用于前进地头曲线；履带原地转向不等同于零半径 Dubins 路径' },
    { key: 'headland_width', label: '地头预留宽度', unit: 'm', min: 0, max: 50, step: 0.5, hint: '仅手工“生成覆盖路径”采用此值；一键优化按车辆长宽和安全余量自动计算最小安全地头' },
    { key: 'swath_angle', label: '作业方向', unit: '°', min: -180, max: 180, step: 5, degrees: true, hint: '网格东向为 0°，逆时针为正；不是设备北零航向' },
    { key: 'route_mode', label: '条带作业顺序', choices: { BOUSTROPHEDON: '牛耕', SNAKE: '蛇形', SPIRAL: '螺旋', CONTOUR: '回形' }, hint: '回形为从内向外连续展开的开口路径，层间直接平滑过渡，不再逐圈闭合；层间距采用作业幅宽，仍受地头预留、转弯半径和净空约束' },
    { key: 'angle_mode', label: '作业方向选择', choices: { SET_ANGLE: '指定方向', BRUTE_FORCE: '自动搜索方向' } },
    { key: 'path_continuity', label: '转弯曲率形式', choices: { DUBINS: 'Dubins（前进，圆弧与直线）', DUBINS_CC: 'Dubins CC（前进，连续曲率）', REEDS_SHEPP: 'Reeds–Shepp（可前进、倒车）', REEDS_SHEPP_HC: 'Reeds–Shepp HC（可前进、倒车，混合曲率）' }, hint: '仅手工规划采用此曲线；按设定最小半径生成。Reeds–Shepp 的倒车段需要勾选允许倒车；一键优化固定原地转向' },
    { key: 'allow_reverse', label: '允许倒车', toggle: true, hint: '默认关闭；勾选并保存、重新规划后，允许执行Reeds–Shepp倒车段，换向时先停车对齐。Dubins仍只前进；现有前方180°雷达配置不保护车后方' },
    { key: 'sampling_distance', label: '路径采样间距', unit: 'm', min: 0.02, max: 1, step: 0.02 },
    { key: 'lookahead', label: 'PP基础前视距离', unit: 'm', min: 0.1, max: 10, step: 0.1, hint: '仅PP采用：实际前视距离=基础值+0.35×纵向速度反馈绝对值；其余算法不使用该参数' },
    { key: 'speed', label: '目标行驶速度', unit: 'm/s', min: 0.01, max: 1, step: 0.05, hint: '目标速度大小，最大1 m/s；随转弯、换向点和终点减速。允许倒车时由路径方向决定线速度正负' },
    { key: 'max_angular', label: '最大角速度', unit: 'rad/s', min: 0.01, max: 1, step: 0.05, hint: '角速度指令在正负此值内限幅，最大允许±1 rad/s；实际输出由跟踪误差和曲率决定' },
    { key: 'completion_tolerance', label: '终点完成阈值', unit: 'm', min: 0.02, max: 2, step: 0.05 },
    { key: 'start_tolerance', label: '直接跟踪起点距离', unit: 'm', min: 0.05, max: 5, step: 0.1, hint: '超过此距离会先单点导航到覆盖起点，再自动跟踪；不限制单点接近的总距离' },
    { key: 'search_window', label: '路径进度搜索窗口', unit: 'm', min: 0.1, max: 10, step: 0.1 },
    { key: 'max_path_error', label: '最大路径偏离', unit: 'm', min: 0.05, max: 10, step: 0.1 },
  ],
}

// The smart policy is fixed; these exports remain for external parameter tools.
export const smartPlanningFields: ParameterField[] = []
export const smartSearchFields: ParameterField[] = []
export const smartAdvancedFields: ParameterField[] = []
