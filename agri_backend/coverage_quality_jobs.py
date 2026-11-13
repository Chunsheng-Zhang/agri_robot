"""Bounded background planning evaluation; no computation on ROS/API threads."""
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .missions import ROOT, atomic_json, now


def run_plan_quality(directory):
    output=Path(directory)/'analysis'
    status=output/'coverage_quality_plan_status.json'
    try:
        atomic_json(status,{'state':'PROCESSING','updated_at':now()})
        with (output/'coverage_quality_plan.log').open('w',encoding='utf-8') as log:
            subprocess.run([sys.executable,'-B','-m','agri_backend.coverage_quality','plan',str(directory)],
                cwd=ROOT,env=dict(os.environ,MPLCONFIGDIR=str(ROOT/'.diagnostics/matplotlib'),
                                  PYTHONDONTWRITEBYTECODE='1'),
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True)
        atomic_json(status,{'state':'SUCCEEDED','updated_at':now()})
    except Exception as exc:
        atomic_json(status,{'state':'FAILED','reason':f'覆盖质量评价失败：{exc}；检查 coverage_quality_plan.log',
                            'updated_at':now()})


class PlanQualityJobs:
    def __init__(self):
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='coverage-quality')
        self.jobs={}

    def submit(self,directory):
        directory=Path(directory)
        if directory in self.jobs:
            return
        path=directory/'analysis/coverage_quality_plan_status.json'
        if path.exists() and json.loads(path.read_text()).get('state') in ('SUCCEEDED','FAILED'):
            return
        self.jobs={p:f for p,f in self.jobs.items() if not f.done()}
        if len(self.jobs)>=4:
            atomic_json(path,{'state':'FAILED','reason':'质量评价队列已满，请重新规划生成新记录','updated_at':now()})
            return
        atomic_json(path,{'state':'PENDING','updated_at':now()})
        self.jobs[directory]=self.executor.submit(run_plan_quality,directory)

    def busy(self,directory):
        future=self.jobs.get(Path(directory))
        return future is not None and not future.done()

    def close(self):
        self.executor.shutdown(wait=False,cancel_futures=True)


def snapshot(directory,run):
    if run.get('coverage_quality_version')!=1:
        return None
    if run.get('coverage_quality_error'):
        return {'state':'FAILED','reason':run['coverage_quality_error']}
    output=Path(directory)/'analysis'
    def read(name):
        path=output/name
        return json.loads(path.read_text()) if path.exists() else None
    try:
        status=read('coverage_quality_plan_status.json') or ({'state':'FAILED','reason':run.get('reason') or '规划未完成'}
            if run.get('state') in ('FAILED','FAULT','CANCELED') else {'state':'WAITING_PATH'})
        planned=read('coverage_quality_plan.json') if status['state']=='SUCCEEDED' else None
        final=read('coverage_quality.json')
        failure=read('coverage_quality_status.json')
        analysis_status=read('status.json')
        if not failure and analysis_status and analysis_status.get('state')=='FAILED':
            failure=analysis_status
        if final:
            planned=final['planned']
            status={'state':'SUCCEEDED'}
        def summary(value):
            if not value:return None
            return {k:value[k] for k in ('available','reason','width_m','resolution_m','metrics','sensitivity','effective_headland_width_m') if k in value}
        return {**status,'planned':summary(planned),'actual':summary(final.get('actual')) if final else None,
                'actual_error':failure.get('reason') if failure and failure.get('state')=='FAILED' else None}
    except (OSError,ValueError,KeyError,TypeError) as exc:
        return {'state':'FAILED','reason':f'质量评价文件不可读：{exc}'}
