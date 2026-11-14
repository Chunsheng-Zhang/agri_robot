"""Read-only saved coverage-plan catalogue. Geometry is loaded only on demand."""
import hashlib
import json
from time import monotonic
from agri_path_planning.saved_path import read_saved_path
from .missions import CoverageParameters, LocalFrame, field_from_data

GENERATING=('PREPARING','PLANNING')


def signature(points):
    xy=[{'x':float(p[0]),'y':float(p[1])} for p in points]
    return hashlib.sha256(json.dumps(['map',xy],sort_keys=True).encode()).hexdigest()


def plan_file(directory):
    files=list((directory/'paths').glob('coverage_*.json'))
    if directory.is_symlink() or len(files)!=1 or files[0].is_symlink() or not files[0].resolve().is_relative_to(directory.resolve()):
        raise ValueError('本条记录缺少唯一的完整覆盖路径')
    return files[0]


def read_plan(store, mission, identity, *, allow_pending_save=False):
    directory=store.run_path(identity)
    if directory.is_symlink():raise ValueError('路径记录目录不可为符号链接')
    if store.deleted_run_path(identity).exists():raise ValueError('该规划记录已从列表删除')
    run=json.loads((directory/'run.json').read_text(encoding='utf-8'))
    if run.get('id')!=identity:raise ValueError('路径记录编号与目录不一致')
    if run.get('saved_plan_version')!=1 and not (allow_pending_save and run.get('plan_save_required') is True):
        raise ValueError('请先点击保存规划路径' if run.get('plan_save_required') is True else
                         '此记录未启用路径复用，请重新生成路径')
    if run.get('mode')!='coverage' or run.get('state') in GENERATING:
        raise ValueError('该记录不是已生成的覆盖规划')
    if run.get('mission_id')!=mission['id'] or run.get('mission_revision')!=mission['revision']:
        raise ValueError('已保存路径不属于当前任务版本，请重新规划')
    source=json.loads((directory/'config/mission.json').read_text(encoding='utf-8'))
    if source.get('id')!=mission['id'] or source.get('revision')!=run.get('mission_revision') or source.get('mode')!='coverage':
        raise ValueError('路径记录与原任务快照不一致')
    if source.get('projection')!=run.get('projection'):raise ValueError('原任务快照的固定原点不一致')
    if any(source.get(k)!=mission.get(k) for k in ('projection','boundary','entry')) or (
            CoverageParameters(**source['parameters']).model_dump()!=CoverageParameters(**mission['parameters']).model_dump()):
        raise ValueError('任务配置与已保存路径的快照不一致，请重新规划')
    if run.get('projection')!=mission['projection']:raise ValueError('路径记录的固定原点不一致')
    config=store.config();common=config['/**']['ros__parameters']
    frame=LocalFrame(mission['projection']['latitude'],mission['projection']['longitude'],common.get('max_local_radius',5000))
    field=field_from_data(source['boundary'],frame)
    width=mission['parameters'].get('robot_width',config.get('coverage_adapter',{}).get(
        'ros__parameters',{}).get('robot_width',common.get('robot_width',.2)))
    from agri_base.path_geometry import rotation_envelope
    p=mission['parameters']
    envelope=rotation_envelope(p.get('robot_length',.45),width,p.get('rotation_safety_margin',.05),
        p.get('rotation_center_x',0.),p.get('rotation_center_y',0.))
    from agri_path_planning.entry import parse_entry
    data,points=read_saved_path(plan_file(directory),frame,field,width,bool(p.get('allow_reverse',False)),
        expected_envelope=envelope,expected_entry=parse_entry(mission.get('entry'),frame))
    fingerprint=signature(points)
    if run.get('reference_fingerprint') and fingerprint!=run['reference_fingerprint']:
        raise ValueError('已保存路径指纹与记录不一致，禁止加载')
    return run,data,points,fingerprint


class SavedPlans:
    def __init__(self):
        self.root=None;self.updated=-float('inf');self.records=[]

    def summaries(self,store,missions):
        # No complete point arrays in the workspace's frequent status response.
        if self.root!=store.root or monotonic()-self.updated>2:
            self.records=store.list_runs(limit=None);self.root=store.root;self.updated=monotonic()
        by_id={m['id']:m for m in missions if m['mode']=='coverage'}
        result={key:[] for key in by_id}
        for run in self.records:
            mission=by_id.get(run.get('mission_id'))
            if not mission or run.get('saved_plan_version')!=1 or run.get('mode')!='coverage' or run.get('state') in GENERATING or run.get('source_plan_id'):
                continue
            if run.get('mission_revision')!=mission['revision'] or run.get('projection')!=mission['projection']:
                continue
            if len(result[mission['id']])>=10:continue
            directory=store.run_path(run['id'])
            if store.deleted_run_path(run['id']).exists():continue
            try:plan_file(directory)
            except (ValueError,OSError):continue
            result[mission['id']].append({k:run.get(k) for k in
                ('id','mission_id','mission_revision','created_at','planning_strategy','reference_fingerprint')})
        return result
