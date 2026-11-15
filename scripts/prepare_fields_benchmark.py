#!/usr/bin/env python3
"""Freeze a reproducible uniform sample before looking at planning outcomes."""
import argparse
import hashlib
import json
from pathlib import Path
import random
from pyproj import Transformer
from shapely import wkt
from shapely.ops import transform

ROOT=Path(__file__).resolve().parents[1]


def prepare(dataset,count,seed):
    files=sorted(Path(dataset).glob('*.wkt'))
    selected=random.Random(seed).sample(files,count);rows=[]
    for file in selected:
        raw=wkt.loads(file.read_text());lon,lat=raw.centroid.x,raw.centroid.y
        epsg=32600+int((lon+180)//6)+1
        projected=transform(Transformer.from_crs(4326,epsg,always_xy=True).transform,raw)
        parts=[raw] if raw.geom_type=='Polygon' else list(raw.geoms)
        rows.append({'file':str(file.relative_to(ROOT)),'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),
            'geometry_type':raw.geom_type,'valid':raw.is_valid,'epsg':epsg,
            'utm_anchor':[projected.centroid.x,projected.centroid.y],'area_m2':projected.area,
            'vertices':sum(len(p.exterior.coords) for p in parts),
            'holes':sum(len(p.interiors) for p in parts),'bounds_m':list(projected.bounds)})
    return {'seed':seed,'population_size':len(files),
        'sampling':'random.Random(seed).sample(sorted(WKT filenames),50); no outcome filtering or replacement',
        'fields':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,default=ROOT/'open_navigation_ws/src/fields2Benchmark_Dataset')
    p.add_argument('--count',type=int,default=50);p.add_argument('--seed',type=int,default=20261114)
    p.add_argument('--out',type=Path,default=ROOT/'.diagnostics/fields50_20261114')
    args=p.parse_args();value=prepare(args.dataset,args.count,args.seed)
    args.out.mkdir(parents=True,exist_ok=True);target=args.out/'manifest.json'
    if target.exists():
        assert json.loads(target.read_text())==value,'Existing manifest differs; choose a new output directory'
        print('Existing manifest reproduced exactly; no file changed')
    else:target.write_text(json.dumps(value,indent=2));print('Saved',target)


if __name__=='__main__':main()
