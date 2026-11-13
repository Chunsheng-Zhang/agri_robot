"""Fields2Cover curve names shared by Web, OpenNav and native connections."""

CURVES = {
    'DUBINS': ('PP_DubinsCurves', 'DUBIN', 'DISCONTINUOUS'),
    'DUBINS_CC': ('PP_DubinsCurvesCC', 'DUBIN', 'CONTINUOUS'),
    'REEDS_SHEPP': ('PP_ReedsSheppCurves', 'REEDS_SHEPP', 'DISCONTINUOUS'),
    'REEDS_SHEPP_HC': ('PP_ReedsSheppCurvesHC', 'REEDS_SHEPP', 'CONTINUOUS'),
}


def curve_name(value):
    # Old task/config values only selected smoothness and implicitly used Dubins.
    if not isinstance(value,str):
        raise ValueError('转弯曲率形式必须为字符串')
    value = {'CONTINUOUS': 'DUBINS_CC', 'DISCONTINUOUS': 'DUBINS'}.get(value, value)
    if value not in CURVES:
        raise ValueError('转弯曲率形式必须为 DUBINS、DUBINS_CC、REEDS_SHEPP 或 REEDS_SHEPP_HC')
    return value


def native_curve(f2c, value, spacing):
    curve = getattr(f2c, CURVES[curve_name(value)][0])()
    curve.setUsingCache(False)
    curve.setDiscretization(spacing)
    return curve
