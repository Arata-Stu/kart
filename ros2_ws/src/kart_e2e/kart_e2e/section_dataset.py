"""Attach registered offline poses and section/head labels to an existing teacher dataset."""
import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from .sections import Router, load


def section_filter(routing, sections, *, physical_only=False):
    available={row['id'] for row in routing['sections'] if not physical_only or row['id']!='*'}
    if sections is None: return available
    if '*' in sections:
        if list(sections) != ['*']: raise ValueError('Use wildcard alone, not mixed with section IDs')
        return available
    chosen=set(sections)
    if not chosen or chosen-available:
        raise ValueError('Choose nonempty known section IDs; unknown: '+', '.join(sorted(chosen-available)))
    return chosen


def split(dataset, trajectory, routing, output, sections=None):
    dataset, trajectory, output = Path(dataset).resolve(strict=True), Path(trajectory).resolve(strict=True), Path(output)
    route = Router(routing)
    chosen = section_filter(routing, sections, physical_only=True)
    if output.exists(): raise FileExistsError(output)
    meta=json.loads((dataset/'metadata.json').read_text())
    if meta.get('clock') != 'header':
        raise ValueError('Section association requires a dataset made with --clock header')
    poses=[json.loads(line) for line in trajectory.read_text().splitlines() if line.strip()]
    stamps=[p['stamp_ns'] for p in poses]
    if not stamps or any(type(t) is not int or t<=0 for t in stamps) or any(a>=b for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Trajectory timestamps must be positive, strictly increasing; split clock epochs first')
    if any(p.get('routing_sha256') != route.sha for p in poses):
        raise ValueError('Trajectory and dataset routing mismatch')
    rows=[json.loads(line) for line in (dataset/'samples.jsonl').read_text().splitlines()]
    if any(a['stamp_ns']>=b['stamp_ns'] for a,b in zip(rows,rows[1:])):
        raise ValueError('Dataset timestamps must be strictly increasing')
    output.parent.mkdir(parents=True, exist_ok=True)
    counts, skipped=Counter(),Counter()
    with tempfile.TemporaryDirectory(prefix='.section-', dir=output.parent) as tmp:
        stage=Path(tmp)/'output'; stage.mkdir()
        accepted=[]
        for row in rows:
            stamp=row['stamp_ns']; i=bisect_right(stamps, stamp)-1
            if i<0 or stamp-stamps[i]>routing['max_pose_skew_s']*1e9:
                skipped['missing_or_old_pose']+=1; continue
            pose=poses[i]
            if not pose.get('valid') or pose.get('frame')!=routing['frame'] or pose.get('base_frame')!=routing['base_frame']:
                route.reset(); skipped['unlocalized']+=1; continue
            selected=route.select(pose['x'],pose['y'],pose['yaw'])
            if selected is None:
                skipped['unassigned']+=1; continue
            if selected['section_id'] not in chosen:
                skipped['section_filter']+=1; continue
            item=dict(row, **selected, pose_stamp_ns=pose['stamp_ns'])
            key='counts' if meta.get('modality')=='evs' else 'image'
            source=(dataset/row[key]).resolve(strict=True)
            if not source.is_relative_to(dataset): raise ValueError('Sample outside dataset')
            relative=Path('inputs')/(str(len(accepted)).zfill(8)+source.suffix)
            target=stage/relative; target.parent.mkdir(exist_ok=True)
            try: os.link(source,target)
            except OSError: shutil.copyfile(source,target)
            item[key]=str(relative);accepted.append(item);counts[selected['section_id']]+=1
        if not accepted: raise ValueError('No assigned samples; inspect map registration/poses/clock')
        details=dict(meta, source_dataset=str(dataset), source_manifest_sha256=hashlib.sha256((dataset/'samples.jsonl').read_bytes()).hexdigest(),
                     trajectory=str(trajectory), trajectory_sha256=hashlib.sha256(trajectory.read_bytes()).hexdigest(),
                     routing=routing, routing_sha256=route.sha, samples=len(accepted), section_counts=dict(counts), section_skipped=dict(skipped),
                     selected_sections=sorted(chosen), missing_selected_sections=sorted(chosen-set(counts)))
        (stage/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in accepted))
        (stage/'metadata.json').write_text(json.dumps(details,indent=2))
        # Each section is a directly trainable dataset. Hard links share storage.
        for section in counts:
            folder=stage/'sections'/section;folder.mkdir(parents=True)
            local=[]
            for row in accepted:
                if row['section_id']!=section: continue
                key='counts' if meta.get('modality')=='evs' else 'image'
                target=folder/row[key];target.parent.mkdir(exist_ok=True)
                os.link(stage/row[key],target)
                local.append(row)
            (folder/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in local))
            (folder/'metadata.json').write_text(json.dumps(dict(details,samples=len(local)),indent=2))
        os.rename(stage,output)
    return dict(samples=len(accepted),sections=dict(counts),skipped=dict(skipped))


def extract(dataset, output, sections, *, assign_routing=None):
    """Select existing section labels without VSLAM replay; preserve teacher labels."""
    from .sections import digest
    dataset, output=Path(dataset).resolve(strict=True),Path(output)
    if output.exists(): raise FileExistsError(output)
    meta=json.loads((dataset/'metadata.json').read_text())
    routing=assign_routing if assign_routing is not None else meta.get('routing')
    if not routing: raise ValueError('Run split first: source dataset has no section routing')
    router=Router(routing)
    assigning=assign_routing is not None
    if assigning:
        if list(sections)!=['*'] or router.wildcard is None:
            raise ValueError('Whole-lap assignment requires --sections "*" and a wildcard head in routing')
        chosen={'*'}
    else:
        if meta.get('routing_sha256')!=digest(routing): raise ValueError('Source routing checksum mismatch')
        chosen=section_filter(routing,sections)
    rows=[json.loads(line) for line in (dataset/'samples.jsonl').read_text().splitlines()]
    mapping={s['id']:s['head'] for s in routing['sections']}
    if assigning:
        selected=[]
        for row in rows:
            item=dict(row)
            for key in ('pose_stamp_ns','station_m','distance_m'):
                item.pop(key,None)
            item.update(section_id='*', head_id=router.wildcard['head'],
                        head_index=routing['heads'].index(router.wildcard['head']),
                        assignment_source='wildcard_no_localization')
            selected.append(item)
    else:
        for row in rows:
            if row.get('section_id') not in mapping or mapping[row['section_id']]!=row.get('head_id'):
                raise ValueError('Source section/head label mismatch')
        selected=[r for r in rows if r['section_id'] in chosen]
    if not selected: raise ValueError('No samples in selected sections')
    output.parent.mkdir(parents=True,exist_ok=True)
    counts=Counter(r['section_id'] for r in selected)
    with tempfile.TemporaryDirectory(prefix='.section-',dir=output.parent) as tmp:
        stage=Path(tmp)/'output';stage.mkdir()
        written=[]
        for row in selected:
            item=dict(row)
            key='counts' if meta.get('modality')=='evs' else 'image'
            source=(dataset/row[key]).resolve(strict=True)
            if not source.is_relative_to(dataset): raise ValueError('Sample outside dataset')
            relative=Path('inputs')/(str(len(written)).zfill(8)+source.suffix)
            target=stage/relative;target.parent.mkdir(exist_ok=True)
            try:os.link(source,target)
            except OSError:shutil.copyfile(source,target)
            item[key]=str(relative);written.append(item)
        details=dict(meta,source_dataset=str(dataset),source_manifest_sha256=hashlib.sha256((dataset/'samples.jsonl').read_bytes()).hexdigest(),
                     routing=routing,routing_sha256=router.sha,
                     samples=len(written),section_counts=dict(counts),selected_sections=sorted(chosen),
                     missing_selected_sections=sorted(chosen-set(counts)),
                     section_skipped=dict(section_filter=len(rows)-len(selected)),section_extraction_only=not assigning)
        if assigning:
            details.update(section_assignment='wildcard_no_localization')
            for key in ('trajectory','trajectory_sha256'):
                details.pop(key,None)
        (stage/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in written))
        (stage/'metadata.json').write_text(json.dumps(details,indent=2))
        os.rename(stage,output)
    return dict(samples=len(written),sections=dict(counts),skipped=dict(section_filter=len(rows)-len(selected)))


def bind_map_sections(assignments, document, lane_id, line_type):
    """Reference existing customline speed sections without inferring head semantics."""
    lane=next(row for row in document['lanes'] if row['id']==lane_id)
    existing=lane.get('custom_speeds', [])
    if not existing:
        existing=lane.get('lines',{}).get('customline',{}).get('speed_sections',[])
    result=[]
    for row in assignments:
        item=dict(row)
        if 'map_section' in item:
            if line_type!='customline':
                raise ValueError('Map speed sections refer to customline distances only')
            index=item['map_section']
            if type(index) is not int or not 0<=index<len(existing):
                raise ValueError('Unknown map_section index (zero-based)')
            if 'start_m' in item or 'end_m' in item:
                raise ValueError('Choose map_section or explicit distances, not both')
            item.update(start_m=existing[index]['start'], end_m=existing[index]['end'])
        result.append(item)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    build=sub.add_parser('routing',help='Freeze map geometry and section-to-head assignments')
    for key in ('map','localization-map','lane','line','assignments','output'): build.add_argument('--'+key,required=True)
    label=sub.add_parser('split')
    for key in ('dataset','trajectory','routing','output'): label.add_argument('--'+key,type=Path,required=True)
    label.add_argument('--sections',nargs='+',help='Only these section IDs; omitted selects all')
    select=sub.add_parser('extract',help='Extract sections from a labeled dataset without replay')
    for key in ('dataset','output'): select.add_argument('--'+key,type=Path,required=True)
    select.add_argument('--sections',nargs='+',required=True)
    assign=sub.add_parser('assign',help='Assign all teacher samples to wildcard head, without poses')
    for key in ('dataset','routing','output'): assign.add_argument('--'+key,type=Path,required=True)
    assign.add_argument('--sections',nargs='+',default=['*'],help='Whole-lap assignment: wildcard only')
    a=p.parse_args()
    if a.command=='assign':
        print(json.dumps(extract(a.dataset,a.output,a.sections,assign_routing=load(a.routing)),indent=2));return
    if a.command=='extract':
        print(json.dumps(extract(a.dataset,a.output,a.sections),indent=2));return
    if a.command=='split':
        print(json.dumps(split(a.dataset,a.trajectory,load(a.routing),a.output,a.sections),indent=2));return
    from .sections import map_fingerprint
    from kart_hdmap.document import load as load_map, select, read_document
    path=Path(a.map).resolve(strict=True)
    doc=load_map(path);lane=select(doc,a.lane)
    config=json.loads(Path(a.assignments).read_text())
    config['sections']=bind_map_sections(config['sections'],read_document(path),a.lane,a.line)
    config.update(schema='kart.section_heads.v1', frame=doc['frame_id'], points=lane['lines'][a.line]['points'], closed=lane['closed'],
                  localization_map_sha256=map_fingerprint(a.localization_map),map_file=str(path),map_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),lane_id=a.lane,line_type=a.line)
    Router(config)
    with Path(a.output).open('x') as f: json.dump(config,f,indent=2)

if __name__=='__main__': main()
