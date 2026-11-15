#!/usr/bin/env python3
"""Report ten fixed 6m inputs against the immutable v6/native archive."""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import tarfile

ROOT=Path(__file__).resolve().parents[1]
FIELDS=('coverage_percent','overlap_percent','length_m','point_turn_count','score')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=ROOT/'.diagnostics/layout_portfolio10_20261114')
    parser.add_argument('--reference',type=Path,default=ROOT/'.diagnostics/fields50_20261114')
    args=parser.parse_args();out=args.out;reference=args.reference
    selection=json.loads((out/'selection.json').read_text());indices=selection['original_indices']
    rows=json.loads((out/'results.json').read_text())
    assert len(rows)==10 and {r['index'] for r in rows}==set(indices)
    config=json.loads((out/'configuration.json').read_text())
    for name,expected in config['source_hashes'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    for source in selection['fields']:
        assert hashlib.sha256((ROOT/source['file']).read_bytes()).hexdigest()==source['sha256']
    previous=json.loads((reference/'results.json').read_text())
    lookup={(r['index'],r['method']):r for r in previous if r['width_m']==6.}
    comparisons=[]
    for after in sorted(rows,key=lambda r:indices.index(r['index'])):
        before=lookup[after['index'],'current'];native=lookup[after['index'],'native']
        item={'field':after['field'],'index':after['index'],'state':after['state'],
              'after':after,'before':before,'native_reference':native}
        if after['state']=='SUCCEEDED':
            with gzip.open(out/'jobs'/f"{after['key']}_optimization.json.gz",'rt') as stream:report=json.load(stream)
            feasible=[r for r in report['candidates'] if r.get('executable')]
            minima={family:min((r['score'] for r in feasible if r.get('layout_family')==family),default=None)
                    for family in ('EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL')}
            assert report['selected']['score']==min(r['score'] for r in feasible)
            assert minima['EDGE_GLOBAL'] is None or report['selected']['score']<=minima['EDGE_GLOBAL']+1e-12
            assert after['clearance_passed'] and after['outside_length_m']<1e-5
            item['family_min_grid_scores']=minima
            item['delta']={k:after['independent'][k]-before['independent'][k] for k in FIELDS}
        comparisons.append(item)
    paired=[r for r in comparisons if all(r[k]['state']=='SUCCEEDED' for k in ('after','before','native_reference'))]
    summary={'date':'2026-11-15','width_m':6.,'field_count':10,'paired_count':len(paired),
             'selection':selection['selection'],'hardware':False,'hashes_verified':True,
             'states':dict(Counter(r['state'] for r in rows)),
             'winners':dict(Counter(r['selected']['layout_family'] for r in rows if r['state']=='SUCCEEDED')),
             'means':{stage:{k:statistics.mean(r[stage]['independent'][k] for r in paired) for k in FIELDS}
                      for stage in ('before','after','native_reference')},
             'planning_seconds':{stage:statistics.mean(r[stage]['planning_seconds'] for r in paired)
                                 for stage in ('before','after','native_reference')},
             'quality_change_counts':{k:dict(Counter('decreased' if r['delta'][k]<-1e-8 else 'increased' if r['delta'][k]>1e-8 else 'equal' for r in paired)) for k in FIELDS},
             'comparisons':comparisons,
             'limitations':['Three diagnostic cases plus seven random valid fields; not an unbiased held-out sample.',
                'Before and native rows are immutable historical runs; time comparisons are uncontrolled.',
                'No physical 6m implement rotation envelope, agricultural truth or field execution measured.']}
    guards=[]
    for file in sorted((out/'attempts').glob('**/*.json')):
        record=json.loads(file.read_text())
        if str(record.get('state','')).startswith('BENCHMARK_'):
            guards.append({'field':record.get('field'),'state':record['state'],
                           'archive':str(file.relative_to(out)),'peak_rss_mb':record.get('peak_rss_mb')})
    summary['prior_external_guard_attempts']=guards
    (out/'comparison.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    with (out/'comparison.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        names=['field','state','winner']+[f'{stage}_{key}' for stage in ('before','after','native_reference') for key in FIELDS]
        writer=csv.DictWriter(stream,fieldnames=names);writer.writeheader()
        for row in comparisons:
            flat={k:row[k] for k in ('field','state')};flat['winner']=row['after'].get('selected',{}).get('layout_family')
            flat.update({f'{stage}_{key}':row[stage].get('independent',{}).get(key) for stage in ('before','after','native_reference') for key in FIELDS})
            writer.writerow(flat)
    means=summary['means'];lines=['# 三类布局升级：10地块、6m验证','',
        '2026-11-15完成。仅重跑选定的10个地块，未重跑50块；不启动ROS硬件或车辆。',
        '样本为3个既有诊断案例＋固定随机种子20261115抽取的7个其他有效地块，不作为无偏留出集或普遍优越性证明。',
        '原v6和原生参考来自上轮不可变归档，本轮只运行v7；耗时跨批次且受主机负载影响，不作严格速度增量估计。','',
        f'成功：{len(paired)}/10；选定布局：{json.dumps(summary["winners"],ensure_ascii=False)}。',
        f'外部实验保护中断历史：{json.dumps(guards,ensure_ascii=False)}。中断单项已在其他检查结束后保持原保护重试，最终状态见上；中途中断不当作完整结果。' if guards else '本轮没有额外实验保护中断。','',
        '| 指标 | 升级前v6 | 升级后v7 | 原生参考 |','| --- | ---: | ---: | ---: |']
    labels=dict(zip(FIELDS,['覆盖率 %','重复率 %','路径长度 m','停车转向次数','四目标J']))
    for key in FIELDS:lines.append('| '+labels[key]+' | '+' | '.join(f'{means[stage][key]:.6f}' for stage in ('before','after','native_reference'))+' |')
    lines += ['', '覆盖率下降是固定四目标权衡的实际结果，不隐藏，也不把综合改善说成全部指标改善。未设置最低覆盖门槛。',
        '所有最终方案均通过完整旋转包络检查，最终网格评分不高于本轮保留的沿边候选；独立矢量指标单列。','',
        '| 地块 | 最终布局 | 升级前J | 升级后J | 覆盖变化，百分点 | 转向变化 |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for row in paired:
        lines.append(f'| {row["field"]} | {row["after"]["selected"]["layout_family"]} | {row["before"]["independent"]["score"]:.6f} | {row["after"]["independent"]["score"]:.6f} | {row["delta"]["coverage_percent"]:+.4f} | {row["delta"]["point_turn_count"]:+d} |')
    lines += ['', '本组没有最终选中分区独立方向方案，不能声称该新增分支在这10块上已经体现额外收益。',
        'L形合成用例另验证不同分区方向、完整连接和不重复内缩；软件与ROS验证日志见同目录。',
        '完整三类候选、拒绝/去重原因见jobs/*_optimization.json.gz；输入名单见selection.json；路径和独立复核见results.json。','',
        '[10页路径对照图册](atlas_10_fields.pdf) · [逐地块CSV](comparison.csv) · [完整比较JSON](comparison.json)']
    (out/'report.md').write_text('\n'.join(lines)+'\n')
    plot(out,reference,comparisons)
    with tarfile.open(out/'after_source_snapshot.tar.gz','w:gz') as archive:
        for name in config['source_hashes']:archive.add(ROOT/name,arcname=name)
    print(json.dumps({k:summary[k] for k in ('states','winners','means','planning_seconds','quality_change_counts')},ensure_ascii=False))


def plot(out,reference,comparisons):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager
    from matplotlib.backends.backend_pdf import PdfPages
    from shapely import wkt
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name()
    plt.rcParams['axes.unicode_minus']=False
    figures=out/'figures';figures.mkdir(exist_ok=True)
    with PdfPages(out/'atlas_10_fields.pdf') as pdf:
        for index,row in enumerate(comparisons):
            fig,axes=plt.subplots(1,3,figsize=(15,6),sharex=True,sharey=True)
            for ax,stage,title in zip(axes,('before','after','native_reference'),('升级前v6','升级后v7','原生参考')):
                result=row[stage];base=out if stage=='after' else reference
                if result['state']!='SUCCEEDED':ax.set_title(title+' '+result['state']);continue
                with gzip.open(base/'jobs'/f"{result['key']}_path.json.gz",'rt') as stream:data=json.load(stream)
                assert data['frame_id']=='benchmark_local_utm' and data['epsg']==row['after']['source']['epsg']
                assert data['utm_anchor']==row['after']['source']['utm_anchor']
                field=wkt.loads(data['field_wkt']);ax.fill(*field.exterior.xy,color='#eef4e8');ax.plot(*field.exterior.xy,c='#333',lw=.9)
                for hole in field.interiors:ax.fill(*hole.xy,color='white');ax.plot(*hole.xy,c='#9b3535',lw=1.)
                x,y=zip(*data['vertices']);ax.plot(x,y,c='#287c99',lw=.7)
                ax.scatter(x[0],y[0],c='#198b3b',s=18,zorder=5);ax.scatter(x[-1],y[-1],c='#d9443b',s=18,zorder=5)
                m=result['independent'];ax.set_title(f'{title}\nC {m["coverage_percent"]:.3f}% / O {m["overlap_percent"]:.3f}%\nL {m["length_m"]:.1f}m / N {m["point_turn_count"]} / J {m["score"]:.5f}')
                ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('共同局部UTM X / m')
            axes[0].set_ylabel('Y / m')
            fig.suptitle(f'{index+1}/10 · {row["field"]} · 6m幅宽\n同一地块/投影/净空；绿色起点、红色终点；抽象几何预测，未建模真实6m机具外伸',fontsize=12)
            fig.tight_layout(rect=(0,0,1,.88));pdf.savefig(fig);fig.savefig(figures/f'{index+1:02d}_{row["field"]}.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
