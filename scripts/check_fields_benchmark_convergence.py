#!/usr/bin/env python3
"""Fixed-index disk polygon convergence checks, without choosing new paths."""
import argparse,gzip,json
from pathlib import Path
from shapely import wkt
from benchmark_coverage_metrics import swept_metrics
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=ROOT/'.diagnostics/fields50_20261114');args=p.parse_args()
    rows=json.loads((args.out/'results.json').read_text());lookup={(r['index'],r['width_m'],r['method']):r for r in rows};checks=[]
    for index in (0,10,20,30,40):
        for width in (.8,6.):
            for method in ('current','native'):
                row=lookup.get((index,width,method));item={'index':index,'width_m':width,'method':method,'state':row['state'] if row else 'MISSING'}
                if row and row['state']=='SUCCEEDED':
                    with gzip.open(args.out/'jobs'/f'{row["key"]}_path.json.gz','rt') as stream:data=json.load(stream)
                    finer=swept_metrics(data['points'],wkt.loads(data['field_wkt']),width,quad_segs=64)
                    item.update(coverage_difference_pp=finer['coverage_percent']-row['independent']['coverage_percent'],
                                overlap_difference_pp=finer['overlap_percent']-row['independent']['overlap_percent'])
                checks.append(item)
    (args.out/'convergence_checks.json').write_text(json.dumps({'fixed_sample_indices_zero_based':[0,10,20,30,40],
        'coarse_quadrant_segments':32,'fine_quadrant_segments':64,'checks':checks},indent=2))
    measured=[c for c in checks if c['state']=='SUCCEEDED'];print('checks',len(measured),
        'max_coverage_change_pp',max((abs(c['coverage_difference_pp']) for c in measured),default=None),
        'max_overlap_change_pp',max((abs(c['overlap_difference_pp']) for c in measured),default=None))
if __name__=='__main__':main()
