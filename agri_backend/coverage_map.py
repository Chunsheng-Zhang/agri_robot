"""Read-only, bounded live coverage layers in a low-priority spawned process.

No ROS imports. API threads only serve snapshots/images. The worker tails the
formal tracking CSV, sharing the report's footprint and continuity policy.
"""
import csv
import hashlib
import io
import json
import math
import multiprocessing
import os
from collections import OrderedDict, deque
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from threading import Lock
from time import monotonic

from .missions import ROOT, now

COLORS = {'completed': (72,163,104,165), 'remaining': (220,98,98,100),
          'overlap': (239,157,48,210)}


class Tail:
    def __init__(self):
        self.offset = 0
        self.header = None

    def lines(self, path, limit=50000):
        if not path.exists():
            return []
        if path.stat().st_size < self.offset:
            raise ValueError('轨迹/状态文件被截断，不能与已有区域累计混合')
        lines = []
        with path.open('rb') as stream:
            stream.seek(self.offset)
            for _ in range(limit):
                line = stream.readline()
                if not line or not line.endswith(b'\n'):
                    break  # Never consume a partially written row.
                lines.append(line.decode('utf-8'))
                self.offset = stream.tell()
        return lines

    def rows(self, path):
        lines = self.lines(path)
        if self.header is None and lines:
            self.header = next(csv.reader([lines.pop(0)]))
        return [dict(zip(self.header, values)) for values in csv.reader(lines)] if self.header else []


class MapState:
    def __init__(self, directory):
        from .coverage_quality import inputs
        from .coverage_geometry import AreaGrid, SweepState, polygons
        self.directory = Path(directory)
        run,mission,field,points,path,width,resolution,headland = inputs(self.directory)
        saved = json.loads(path.read_text())
        points_document = [{'x': p['x'], 'y': p['y']} for p in saved['points']]
        signature = hashlib.sha256(json.dumps(['map',points_document],sort_keys=True).encode()).hexdigest()
        if signature != run.get('reference_fingerprint'):
            raise ValueError('本轮规划路径指纹不一致')
        self.reference, self.width = points, width
        self.grid = AreaGrid(field,headland,resolution)
        self.planned, _ = self.grid.visits([points],width)
        self.sweep = SweepState(len(self.grid.weights))
        self.track_tail, self.status_tail = Tail(), Tail()
        self.previous = None
        self.barriers = []
        self.trail = deque(maxlen=4000)
        self.samples = self.splits = self.invalid = 0
        self.final = False
        self.final_available = False
        self.images = {}
        self.base = {'state':'READY','run_id':run['id'],'mission_id':run['mission_id'],
            'mission_revision':run['mission_revision'],'frame_id':'map','projection':run['projection'],
            'reference_fingerprint':signature,'width_m':width,'resolution_m':self.grid.resolution,
            'extent':[self.grid.x0,self.grid.y0,self.grid.x0+self.grid.nx*self.grid.resolution,
                      self.grid.y0+self.grid.ny*self.grid.resolution],
            'inset_polygons':[[[{'x':x,'y':y} for x,y in ring.coords]
                for ring in [polygon.exterior,*polygon.interiors]] for polygon in polygons(self.grid.interior)],
            'inset_distance_m':headland}
        self.base['field_rings']=[[{'x':x,'y':y} for x,y in ring.coords]
                                  for ring in [field.exterior,*field.interiors]]
        from .coverage_path_order import path_display
        self.base['path_order'] = path_display(saved)

    def update(self):
        from .coverage_quality import tracking_break
        for line in self.status_tail.lines(self.directory/'telemetry/status.jsonl'):
            try:
                item = json.loads(line); data = item['data']
                if (item['topic']=='/tracking/status' and data.get('run_directory')==str(self.directory/'tracks')
                        and (data.get('state')!='RUNNING' or data.get('phase') not in (None,'TRACKING'))):
                    stamp = float(item['received_ros_ns'])/1e9
                    if math.isfinite(stamp): self.barriers.append(stamp)
            except (ValueError,KeyError,TypeError):
                self.invalid += 1
        self.barriers.sort()
        files = sorted((self.directory/'tracks').glob('track_*.csv'))
        if len(files)>1:
            raise ValueError('存在多个正式跟踪文件，不能混合任务尝试')
        if files and not self.final:
            if self.track_tail.header is None:
                sidecar = json.loads(files[0].with_suffix('.json').read_text())
                if sidecar.get('projection')!=self.base['projection'] or sidecar.get('frame_id')!='map':
                    raise ValueError('作业区域轨迹坐标基准不一致')
                if [(p['x'],p['y']) for p in sidecar['reference_path']['points']] != self.reference:
                    raise ValueError('作业区域轨迹参考路径不一致')
            chunks, current = [], [self.previous[1:]] if self.previous else []
            for row in self.track_tail.rows(files[0]):
                try:
                    sample = tuple(float(row[k]) for k in ('stamp','x','y'))
                    if not all(math.isfinite(v) for v in sample): raise ValueError()
                except (KeyError,ValueError,TypeError):
                    self.invalid += 1
                    if len(current)>1: chunks.append(current)
                    current, self.previous = [], None
                    continue
                cut = self.previous is None or tracking_break(self.previous,sample,self.barriers)
                if cut:
                    if self.previous is not None: self.splits += 1
                    if len(current)>1: chunks.append(current)
                    current = []
                current.append(sample[1:])
                self.trail.append((*sample,cut))
                self.previous = sample
                self.samples += 1
            if len(current)>1: chunks.append(current)
            if chunks: self.grid.visits(chunks,self.width,self.sweep)
        # Reconcile provisional live accumulation against the closed report.
        # This also incorporates late status boundaries without rewriting files.
        final_path = self.directory/'analysis/coverage_quality.json'
        if final_path.exists() and not self.final:
            final = json.loads(final_path.read_text())
            if final['projection']!=self.base['projection']:
                raise ValueError('最终作业区域坐标不一致')
            if final['actual']['available']:
                import numpy as np
                if (final.get('run_id') != self.base['run_id']
                        or not math.isclose(final['actual']['resolution_m'], self.grid.resolution)
                        or not math.isclose(final['actual']['width_m'], self.width)
                        or final['actual']['grid_cells'] != len(self.grid.weights)):
                    raise ValueError('最终作业网格参数与本轮地图不一致')
                canonical = np.zeros(len(self.grid.weights),dtype=np.uint32)
                seen = set()
                with (self.directory/'analysis/coverage_quality_actual_cells.csv').open(newline='') as stream:
                    for row in csv.DictReader(stream):
                        r,c,n = (int(row[k]) for k in ('row','column','observed_passes'))
                        if not (0<=r<self.grid.ny and 0<=c<self.grid.nx and 0<=n<=200000):
                            raise ValueError('最终作业网格索引无效')
                        index=r*self.grid.nx+c
                        if index in seen: raise ValueError('最终作业网格重复')
                        if not math.isclose(float(row['field_weight_m2']),float(self.grid.weights[index]),rel_tol=1e-6,abs_tol=1e-8):
                            raise ValueError('最终作业网格面积不一致')
                        seen.add(index); canonical[index]=n
                if len(seen)!=int((self.grid.weights>0).sum()):
                    raise ValueError('最终作业网格不完整')
                self.sweep.counts = canonical
                self.final_available = True
            else:
                self.sweep.counts.fill(0)
            self.final = True
        run = json.loads((self.directory/'run.json').read_text())
        status = ('FINAL' if self.final_available else 'NO_DATA' if self.final else 'NOT_STARTED' if run['stage']=='planning'
                  else 'OBSERVED' if self.sweep.segments else 'WAITING_TRACKING')
        lines, line = [], []
        for _,x,y,cut in self.trail:
            if cut:
                if len(line)>1: lines.append(line)
                line=[]
            line.append({'x':x,'y':y})
        if len(line)>1: lines.append(line)
        if self.previous:
            self.barriers=[stamp for stamp in self.barriers if stamp>=self.previous[0]]
        else:
            self.barriers=self.barriers[-100000:]
        planned_version, planned_images = self.textures(self.planned)
        actual_version, actual_images = self.textures(self.sweep.counts)
        return {'meta':{**self.base,'generated_at':now(),
            'sources':{'planned':{'version':planned_version,'status':'PREDICTED'},
                       'actual':{'version':actual_version,'status':status,'samples':self.samples,
                                 'last_stamp':self.previous[0] if self.previous else None,
                                 'splits':self.splits,'invalid_rows':self.invalid,'track_lines':lines}}},
            'images':{'planned':planned_images,'actual':actual_images}}

    def textures(self, counts):
        import numpy as np
        from PIL import Image
        version = hashlib.sha256(counts.tobytes()).hexdigest()[:20]
        if version in self.images: return version,self.images[version]
        masks = {'completed':counts>=1,'remaining':counts==0,'overlap':counts>=2}
        images = {}
        for name,mask in masks.items():
            rgba=np.zeros((self.grid.ny*self.grid.nx,4),dtype=np.uint8)
            rgba[mask & (self.grid.weights>0)] = COLORS[name]
            stream=io.BytesIO()
            Image.fromarray(rgba.reshape(self.grid.ny,self.grid.nx,4)[::-1]).save(stream,format='PNG')
            images[name]=stream.getvalue()
        self.images[version]=images
        while len(self.images)>2:self.images.pop(next(iter(self.images)))
        return version,images


_CURRENT = None


def render_map(directory):
    global _CURRENT
    if _CURRENT is None or _CURRENT.directory!=Path(directory):
        _CURRENT=MapState(directory)
    return _CURRENT.update()


def worker_setup():
    os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.diagnostics/matplotlib'))
    if hasattr(os,'nice'):os.nice(10)


class CoverageMaps:
    """One worker, no queued backlog, two cached runs, three image versions."""
    def __init__(self):
        self.lock=Lock()
        self.executor=None
        self.future=None
        self.pending_id=None
        self.cache=OrderedDict()
        self.closed=False

    def request(self,directory,run):
        with self.lock:
            self.harvest()
            identity=run['id']
            if run.get('coverage_quality_version')!=1:
                return {'state':'UNAVAILABLE','run_id':identity,'reason':'本轮未启用作业区域评价，请重新生成覆盖路径'}
            if not run.get('reference_fingerprint'):
                return {'state':'PENDING','run_id':identity,'reason':'等待本轮规划路径'}
            cached=self.cache.get(identity)
            if not self.closed and self.future is None and (not cached or monotonic()-cached['received']>=2):
                if self.executor is None:
                    self.executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),initializer=worker_setup)
                self.future=self.executor.submit(render_map,str(directory))
                self.pending_id=identity
            if cached:
                return {**cached['meta'],'snapshot_age_seconds':max(0,monotonic()-cached['received'])}
            return {'state':'PENDING','run_id':identity,'reason':'正在准备本轮作业区域图层'}

    def harvest(self):
        if self.future is None or not self.future.done():return
        identity=self.pending_id
        try:
            result=self.future.result()
            previous=self.cache.get(identity,{}).get('frames',OrderedDict())
            for source,images in result['images'].items():
                key=(source,result['meta']['sources'][source]['version'])
                previous[key]=images
                previous.move_to_end(key)
            while len(previous)>6: previous.popitem(last=False)
            item={'meta':result['meta'],'frames':previous,'received':monotonic()}
        except Exception as exc:
            item={'meta':{'state':'FAILED','run_id':identity,'reason':f'作业区域更新失败：{exc}'},
                  'frames':OrderedDict(),'received':monotonic()}
        self.cache[identity]=item
        self.cache.move_to_end(identity)
        while len(self.cache)>2:self.cache.popitem(last=False)
        self.future,self.pending_id=None,None

    def image(self,identity,source,layer,version):
        with self.lock:
            self.harvest()
            return self.cache.get(identity,{}).get('frames',{}).get((source,version),{}).get(layer)

    def close(self,wait=False):
        with self.lock:
            self.closed=True
            if self.executor:self.executor.shutdown(wait=wait,cancel_futures=True)
