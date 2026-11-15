"""Small read-only optimization summary, scoped to the immutable run directory."""
import json


def read(directory):
    path=directory/'paths/optimization.json'
    if not path.exists():return None
    if path.stat().st_size>8*1024*1024:raise ValueError('优化报告超过读取预算')
    value=json.loads(path.read_text())
    if value.get('schema_version') not in (1,2,3) or value.get('strategy')!='SMART':raise ValueError('优化报告版本无效')
    run=json.loads((directory/'run.json').read_text())
    if value.get('frame_id')!='map' or value.get('projection')!=run['projection']:
        raise ValueError('优化报告坐标原点不一致')
    if value.get('state')=='SUCCEEDED' and run.get('reference_fingerprint') and value.get('reference_fingerprint')!=run['reference_fingerprint']:
        raise ValueError('优化报告与当前参考路径指纹不一致')
    return value


def summary(directory):
    try:
        value=read(directory)
        if value is None:return None
        keys=('schema_version','policy','turn_mode','coverage_objective_enabled','state','reason','selected','evaluated_count','possible_count','budget_exhausted',
              'search_complete','candidate_limit_enabled','search_time_limit_enabled','weights','normalization','energy_model','coverage_constraint_enabled','search_resolution_m',
              'final_resolution_m','curve_type','allow_reverse','reverse_execution_supported','headland_search',
              'selection_scope','elapsed_seconds','configuration','weight_policy','angle_count',
              'boundary_basis_count','deduplicated_layout_count','layout_search')
        small={k:value[k] for k in keys if k in value}
        candidates=value.get('candidates',[])
        small['candidates']=candidates[:128]
        selected=value.get('selected')
        if selected and not any(c.get('id')==selected.get('id') for c in small['candidates']):
            small['candidates'].append(selected)
        small['candidates_truncated']=len(candidates)>len(small['candidates'])
        small['turn_curves']=[{k:c[k] for k in ('type','radius_m','metrics','clearance_valid','executable','reason') if k in c}
                               for c in value.get('turn_curves',[])[:24]]
        return small
    except (OSError,ValueError,TypeError) as exc:
        return {'state':'FAILED','reason':f'优化报告不可读：{exc}'}
