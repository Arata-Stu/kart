"""Confined UI plans for section/head datasets; no vehicle launches."""
import shutil
import sys
from pathlib import Path
from .storage import read_json, within, name


def modules(learning):
    for package in ('kart_e2e','kart_hdmap'):
        path=str(learning.repo/'ros2_ws/src'/package)
        if path not in sys.path:sys.path.insert(0,path)
    from kart_e2e.sections import Router, digest, validate_maps
    return Router,digest,validate_maps


def prepare(learning, action, body):
    Router,digest,validate_maps=modules(learning)
    python,cfg,env=learning.runtime()
    env['PYTHONPATH']=str(learning.repo/'ros2_ws/src/kart_hdmap')+':'+env['PYTHONPATH']
    key=name(body.get('name'))
    kind='routings' if action=='section-routing' else 'trajectories' if action=='section-offline' else 'datasets'
    target=learning.target(kind,key)
    plan=dict(key=target.relative_to(learning.root/kind).as_posix(),kind=kind,target=target,python=python,env=env,
              module='section_workflow',args=['--request','@REQUEST@','--output','@OUTPUT@'],
              resources=[],provenance={})
    if action=='section-routing':
        from kart_hdmap.document import load as load_map, select, read_document
        from kart_e2e.section_dataset import bind_map_sections
        from kart_e2e.sections import map_fingerprint, file_digest
        folder=within(learning.root.parent/'map',name(body.get('map')))
        path=within(folder,'map.json');doc=load_map(path)
        lane_id=body.get('lane');line_type=body.get('line','customline')
        lane=select(doc,lane_id)
        if lane is None or line_type not in lane['lines']:raise ValueError('生成済みlane / ラインを選択してください')
        if read_document(path).get('snapshot_status')=='pending':raise ValueError('準備済みのHDMapを選択してください')
        assignments=body.get('sections')
        if not isinstance(assignments,list) or not 1<=len(assignments)<=100:raise ValueError('section割当を1〜100行で指定してください')
        heads=[]
        for row in assignments:
            if not isinstance(row,dict):raise ValueError('section割当が不正です')
            if row.get('head') not in heads:heads.append(row.get('head'))
        routing=dict(schema='kart.section_heads.v1',frame=doc['frame_id'],base_frame='base_link',
                     heads=heads,sections=bind_map_sections(assignments,read_document(path),lane_id,line_type),
                     points=lane['lines'][line_type]['points'],closed=lane['closed'],lane_id=lane_id,line_type=line_type,
                     map_file=str(path),map_sha256=file_digest(path),localization_map_sha256=map_fingerprint(folder),
                     max_distance_m=1.,max_heading_rad=1.2,ambiguity_distance_m=.15,max_jump_m=3.,
                     max_pose_age_s=.3,max_pose_skew_s=.05)
        Router(routing)
        plan['request']=dict(action='routing',routing=routing)
        plan['provenance']=dict(map=body['map'],routing=routing,routing_sha256=digest(routing))
        plan['resources']+=['read:map:'+body['map']]
        return plan
    source=learning.folder('routings',body.get('routing'))
    route_file=within(source,'routing.json');routing=read_json(route_file);Router(routing)
    plan['resources']+=['read:e2e:routings:'+body['routing']]
    plan['provenance'].update(routing_id=body['routing'],routing_sha256=digest(routing),routing=routing)
    if action=='section-offline':
        if not shutil.which('ros2'):raise ValueError('offline VSLAMはROS/CUDAコンテナ内で実行してください')
        bag=within(learning.records,body.get('bag',''));within(bag,'metadata.yaml')
        map_id=name(body.get('map'));folder=within(learning.root.parent/'map',map_id)
        map_file=within(folder,'map.json');validate_maps(routing,map_file,folder)
        mode=str(body.get('tracking_mode','0'))
        if mode not in ('0','1'):raise ValueError('追跡方式はVOまたはVIOです')
        plan['request']=dict(action='offline',bag=str(bag),map_file=str(map_file),map_dir=str(folder),
                             routing_file=str(route_file),tracking_mode=mode)
        plan['provenance'].update(bag=str(bag),map=map_id)
        plan['resources']+=['read:record:'+str(bag.relative_to(learning.records.resolve())),'read:map:'+map_id,'ros:'+os_domain()]
        return plan
    dataset=learning.folder('datasets',body.get('dataset'));meta=read_json(within(dataset,'metadata.json'))
    within(dataset,'samples.jsonl')
    operation=body.get('operation','split')
    if operation not in ('split','extract','assign'):raise ValueError('抽出方式が不正です')
    sections=body.get('sections',[])
    if not isinstance(sections,list) or not sections or not all(isinstance(s,str) for s in sections):raise ValueError('採用するsectionを選択してください')
    from kart_e2e.section_dataset import section_filter
    if operation=='assign':
        if sections!=['*'] or Router(routing).wildcard is None:raise ValueError('全周割当には「*」のheadが必要です')
    else:section_filter(routing,sections,physical_only=operation=='split')
    plan['module']='section_dataset';plan.pop('request',None)
    args=[operation,'--dataset',str(dataset),'--output','@OUTPUT@','--sections',*sections]
    if operation in ('split','assign'):args+=['--routing',str(route_file)]
    if operation=='split':
        if meta.get('clock')!='header':raise ValueError('専門head用の元datasetは時刻基準headerで作成してください')
        trajectory=learning.folder('trajectories',body.get('trajectory'))
        info=read_json(within(trajectory,'metadata.json'))
        if info.get('routing_sha256')!=digest(routing):raise ValueError('poseと割当が一致しません')
        origin=meta.get('bag')
        if not origin or Path(origin).resolve()!=Path(info.get('bag','')).resolve():raise ValueError('datasetとposeは同じbagから作成してください')
        args+=['--trajectory',str(within(trajectory,'poses.jsonl'))]
        plan['resources']+=['read:e2e:trajectories:'+body['trajectory']]
    if operation=='extract' and meta.get('routing_sha256')!=digest(routing):raise ValueError('元datasetの割当が一致しません')
    plan['args']=args
    plan['provenance'].update(source_dataset=body['dataset'],bag=meta.get('bag',''),modality=meta.get('modality','rgb'),operation=operation)
    # Bag already passed immutable teacher extraction: guard the dataset, not an optional bag path.
    if not plan['provenance']['bag']:plan['provenance'].pop('bag')
    plan['resources']+=['read:e2e:datasets:'+body['dataset']]
    return plan


def os_domain():
    import os
    return os.environ.get('ROS_DOMAIN_ID','0')


def training_routing(learning, groups, action):
    Router,digest,_=modules(learning)
    entries=[[read_json(within(d,'metadata.json')) for d in group] for group in groups]
    marked=[m for group in entries for m in group if m.get('routing_sha256')]
    if not marked:return None
    if action!='train':raise ValueError('section付きdatasetは制御head学習で使用してください')
    routing=marked[0].get('routing');Router(routing);sha=digest(routing)
    for group in entries:
        present=set()
        for meta in group:
            if meta.get('routing_sha256')!=sha or meta.get('routing')!=routing:
                raise ValueError('専門・全周datasetは同じ割当を使ってください。未割当の全周データは「*」を割り当ててください')
            mapping={r['id']:r['head'] for r in routing['sections']}
            present.update(mapping[s] for s,count in meta.get('section_counts',{}).items() if count>0 and s in mapping)
        if present!=set(routing['heads']):raise ValueError('学習用・検証用のそれぞれで全headのdatasetを揃えてください')
    return routing
