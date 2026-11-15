"""Fixed point-turn policy; obsolete strategy and search limits are discarded."""
DEFAULTS={}
# Literature-informed engineering preference, not universal journal constants.
# Rationale and normalization: docs/沿边候选与四目标智能规划.md.
WEIGHTS={'coverage':.40,'overlap':.25,'length':.20,'turns':.15}
LEGACY={'optimization_headland_mode','optimization_search_angle','optimization_weight_coverage',
        'optimization_weight_overlap','optimization_weight_length','optimization_weight_energy',
        'energy_forward','energy_turn','energy_reverse','energy_switch','optimization_min_coverage',
        'optimization_turns','optimization_patterns','optimization_headland_min','optimization_headland_max',
        'optimization_max_candidates','optimization_time_budget'}


def options(value):
    if not isinstance(value,dict):raise ValueError('智能规划配置必须是对象')
    value={k:v for k,v in value.items() if k not in LEGACY}
    if value:raise ValueError('智能规划配置包含未知字段')
    return {}
