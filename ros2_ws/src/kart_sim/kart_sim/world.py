"""JSON indoor map -> MJCF. All coordinates are metres; no ROS dependency."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

COLORS = {'red': '0.85 0.06 0.04 1', 'white': '0.92 0.92 0.88 1',
          'green': '0.04 0.4 0.12 1', 'blue': '0.04 0.08 0.75 1',
          'gray': '0.7 0.7 0.7 1', 'black': '0.015 0.015 0.015 1'}

def nums(v):
    return ' '.join(str(float(x)) for x in v)

def load_map(path):
    doc = json.loads(Path(path).read_text())
    if doc.get('schema_version') != 1 or doc.get('units') != 'm':
        raise ValueError('Expected schema_version=1 and units=m')
    def finite(value):
        if isinstance(value, dict):
            for v in value.values(): finite(v)
        elif isinstance(value, list):
            for v in value: finite(v)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('Non-finite map coordinate')
    finite(doc)
    x0,y0,x1,y1 = doc['room']['bounds']
    if x1 <= x0 or y1 <= y0 or doc['room']['height'] <= 0:
        raise ValueError('Invalid room dimensions')
    sx,sy,_ = doc['spawn']
    if not x0 < sx < x1 or not y0 < sy < y1:
        raise ValueError('Spawn outside room')
    for key in ('wall_height', 'wall_thickness'):
        if doc[key] <= 0: raise ValueError('Invalid wall dimensions')
    gap=doc.get('wall_joint_gap',0)
    if gap<0: raise ValueError('Wall joint gap must be nonnegative')
    for line in doc['polylines']:
        if line['color'] not in COLORS or len(line['points']) < 2:
            raise ValueError('Invalid wall polyline')
        for a,b in zip(line['points'],line['points'][1:]):
            if len(a)!=2 or len(b)!=2 or np.linalg.norm(np.array(b)-a) <= max(1e-6,gap):
                raise ValueError('Invalid wall segment')
    return doc

def box(world, name, bounds, height, color, bottom=0, **attrs):
    x0,y0,x1,y1 = bounds
    if x1<=x0 or y1<=y0 or height<=0: raise ValueError('Invalid box dimensions')
    return ET.SubElement(world,'geom',name=name,type='box',
        pos=nums([(x0+x1)/2,(y0+y1)/2,bottom+height/2]),
        size=nums([(x1-x0)/2,(y1-y0)/2,height/2]),rgba=COLORS.get(color,color),**attrs)

def wall(world, name, a, b, height, thickness, color, bottom=0, **attrs):
    a,b = np.asarray(a),np.asarray(b)
    d=b-a; length=float(np.linalg.norm(d))
    if length<1e-6: raise ValueError('Zero-length wall')
    return ET.SubElement(world,'geom',name=name,type='box',
        pos=nums([*((a+b)/2),bottom+height/2]),size=nums([length/2,thickness/2,height/2]),
        euler=nums([0,0,math.atan2(d[1],d[0])]),rgba=COLORS.get(color,color),**attrs)

def parking_label(world, name, label, origin, color):
    """Flat tape strokes: no font files, textures, or collision geometry."""
    glyphs={
        'P': [[(0,0),(0,.25),(.15,.25),(.2,.20),(.2,.15),(.15,.125),(0,.125)]],
        '1': [[(.06,.22),(.10,.25),(.10,0)],[(.025,0),(.175,0)]],
        '2': [[(0,.25),(.2,.25),(.2,.125),(0,.125),(0,0),(.2,0)]],
        '3': [[(0,.25),(.2,.25),(.2,0),(0,0)],[(0,.125),(.2,.125)]],
    }
    offset=np.asarray(origin,dtype=float)
    for k,char in enumerate(label):
        if char not in glyphs: raise ValueError('Parking label supports P and digits 1..3')
        for i,path in enumerate(glyphs[char]):
            for j,(a,b) in enumerate(zip(path,path[1:])):
                wall(world,f'{name}_{k}_{i}_{j}',offset+a,offset+b,.001,.05,color,
                     bottom=.0005,contype='0',conaffinity='0')
            # Fill the outside wedge at bends so diagonal tape strokes stay connected.
            for j,point in enumerate(path[1:-1]):
                ET.SubElement(world,'geom',name=f'{name}_join_{k}_{i}_{j}',type='cylinder',
                    pos=nums([*(offset+point),.001]),size='.025 .0005',rgba=COLORS[color],
                    contype='0',conaffinity='0')
        offset[0]+=.35 if label=='P1' else .30

def add_world(root, doc):
    world=root.find('worldbody')
    for geom in list(world.findall('geom')): world.remove(geom)
    for light in list(world.findall('light')): world.remove(light)
    x0,y0,x1,y1=doc['room']['bounds']; h=doc['room']['height']
    box(world,'room_floor',[x0,y0,x1,y1],.05,'0.22 0.22 0.23 1',bottom=-.05,friction='0.8 0.002 0.0001')
    corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]
    for i,(a,b) in enumerate(zip(corners,corners[1:])):
        wall(world,f'room_wall_{i}',a,b,h,.1,'0.75 0.72 0.65 1')
    box(world,'ceiling',[x0,y0,x1,y1],.06,'0.7 0.7 0.7 1',bottom=h,group='4')
    # Point lights, no sun or sky. Fixtures are approximate room lighting.
    for i,(x,y) in enumerate((x,y) for x in np.linspace(x0+1,x1-1,3) for y in np.linspace(y0+1,y1-1,3)):
        ET.SubElement(world,'light',name=f'indoor_light_{i}',pos=nums([x,y,h-.15]),
            dir='0 0 -1',directional='false',diffuse='0.15 0.15 0.15',ambient='0.02 0.02 0.02',attenuation='1 0 0',castshadow='false',cutoff='90',exponent='0')
        box(world,f'fixture_{i}',[x-.6,y-.08,x+.6,y+.08],.03,'0.95 0.95 0.9 1',bottom=h-.05,contype='0',conaffinity='0',group='4')
    gap=doc.get('wall_joint_gap',0)
    joints={}
    for i,line in enumerate(doc['polylines']):
        for j,(a,b) in enumerate(zip(line['points'],line['points'][1:])):
            a,b=np.asarray(a,float),np.asarray(b,float)
            direction=(b-a)/np.linalg.norm(b-a)
            wall(world,f'barrier_{i}_{j}',a+direction*gap/2,b-direction*gap/2,
                 line.get('height',doc['wall_height']),doc['wall_thickness'],line['color'])
            for point in (a,b): joints.setdefault(tuple(point),direction)
    if doc.get('joint_supports',False):
        for i,(point,direction) in enumerate(joints.items()):
            # Nominal post beside each joint; the post does not fill the board gap.
            normal=np.array([-direction[1],direction[0]])
            x,y=np.asarray(point)+normal*(doc['wall_thickness']/2+.012)
            ET.SubElement(world,'geom',name=f'joint_post_{i}',type='cylinder',
                pos=nums([x,y,.055]),size='.008 .055',rgba=COLORS['black'])
            wall(world,f'joint_clamp_{i}',[x,y]-.015*direction,[x,y]+.015*direction,
                 .012,.006,'gray',bottom=.075,contype='0',conaffinity='0')
    for i,line in enumerate(doc.get('black_walls',[])):
        for j,(a,b) in enumerate(zip(line,line[1:])):
            wall(world,f'black_curtain_{i}_{j}',a,b,1.33,.01,'black')
    for i,p in enumerate(doc.get('parking',[])):
        x,y,L,W=p['x'],p['y'],p['length'],p['width']
        points=[(x,y+W),(x,y),(x+L,y),(x+L,y+W)]
        for j,(a,b) in enumerate(zip(points,points[1:])):
            if p.get('walls', True):
                wall(world,f'parking_wall_{i}_{j}',a,b,doc['wall_height'],doc['wall_thickness'],'white')
        # Keep the course-facing edge open physically, but mark it in white.
        wall(world,f'parking_opening_line_{i}',[x,y+W],[x+L,y+W],.001,.05,'white',
             bottom=.0005,contype='0',conaffinity='0')
        inset=.05
        outline=[(x+inset,y+inset),(x+L-inset,y+inset),
                 (x+L-inset,y+W-inset),(x+inset,y+W-inset),(x+inset,y+inset)]
        for j,(a,b) in enumerate(zip(outline,outline[1:])):
            wall(world,f'parking_tape_{i}_{j}',a,b,.001,.05,p['color'],bottom=.0005,contype='0',conaffinity='0')
        if p.get('label'):
            parking_label(world,f'parking_label_{i}',p['label'],[x+.275,y+.125],p['color'])
    for i,x in enumerate(doc.get('start_lines',[])):
        wall(world,f'start_line_{i}',[x,0],[x,1.4],.001,.05,'white',contype='0',conaffinity='0')
    for p in doc.get('patches',[]):
        height=p.get('height',.001)
        attrs={'friction':f"{p['friction']} 0.002 0.0001",'priority':str(p.get('priority',1))}
        if 'polygon' in p:
            points=np.asarray(p['polygon'],dtype=float)
            if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or height<=0:
                raise ValueError('Invalid surface polygon')
            edges=np.roll(points,-1,axis=0)-points
            following=np.roll(edges,-1,axis=0)
            turns=edges[:,0]*following[:,1]-edges[:,1]*following[:,0]
            if not (np.all(turns>0) or np.all(turns<0)):
                raise ValueError('Surface polygon must be strictly convex and ordered')
            vertices=[[x,y,z] for z in (0,height) for x,y in points]
            ET.SubElement(root.find('asset'),'mesh',name=p['name']+'_mesh',vertex=nums(np.array(vertices).ravel()))
            ET.SubElement(world,'geom',name=p['name'],type='mesh',mesh=p['name']+'_mesh',rgba=COLORS[p['color']],**attrs)
        else:
            box(world,p['name'],p['bounds'],height,p['color'],**attrs)
    if 'ramp' in doc:
        p=doc['ramp']; a,b,c,d=p['bounds']; z=p['height']; slope=p['slope_length']
        if not 0 < slope < min(c-a,d-b)/2 or z <= 0:
            raise ValueError('Ramp slope must fit inside bounds; height must be positive')
        color=p.get('color','gray')
        box(world,'shortcut_platform',[a+slope,b+slope,c-slope,d-slope],z,color)

        def ramp_mesh(name, vertices):
            # Convex pieces share seams; no vertical lip on any outer edge.
            ET.SubElement(root.find('asset'),'mesh',name=name+'_mesh',vertex=nums(np.array(vertices).ravel()))
            ET.SubElement(world,'geom',name=name,type='mesh',mesh=name+'_mesh',rgba=COLORS[color])

        # Four triangular-prism slopes, 5 cm horizontally / 1 cm vertically by default.
        for name,outer0,outer1,inner0,inner1 in (
            ('west',(a,b+slope),(a,d-slope),(a+slope,b+slope),(a+slope,d-slope)),
            ('east',(c,b+slope),(c,d-slope),(c-slope,b+slope),(c-slope,d-slope)),
            ('south',(a+slope,b),(c-slope,b),(a+slope,b+slope),(c-slope,b+slope)),
            ('north',(a+slope,d),(c-slope,d),(a+slope,d-slope),(c-slope,d-slope)),
        ):
            ramp_mesh('ramp_'+name,[[*outer0,0],[*outer1,0],[*inner0,0],[*inner1,0],
                                     [*inner0,z],[*inner1,z]])
        # Each 5x5 cm corner has two triangular top faces joined on the 45-degree diagonal.
        # The inner vertex reaches platform height; both external edges remain at floor height.
        for i,(x,y,sx,sy) in enumerate(((a,b,1,1),(c,b,-1,1),(c,d,-1,-1),(a,d,1,-1))):
            inner=[x+sx*slope,y+sy*slope]
            ramp_mesh(f'ramp_corner_{i}',[[x,y,0],[x+sx*slope,y,0],[x,y+sy*slope,0],
                                        [*inner,0],[*inner,z]])
    if 'bumps' in doc:
        p=doc['bumps']; a,b,c,d=p['bounds']
        box(world,'bump_mat',[a,b,c,d],.001,'0.06 0.5 0.65 1')
        for i,x in enumerate(np.arange(a+.04,c,.08)):
            for j,y in enumerate(np.arange(b+.04,d,p['spacing'])):
                ET.SubElement(world,'geom',name=f'bump_{i}_{j}',type='ellipsoid',
                    pos=nums([x,y,.001]),size=nums([.018,.018,p['height']]),rgba='0.06 0.5 0.65 1')
    if 'gate' in doc:
        g=doc['gate']; x,y,w,z=g['x'],g['y'],g['width'],g['clearance']
        for sign in (-1,1):
            wall(world,f'gate_post_{sign}',[x,y+sign*w/2-.01],[x,y+sign*w/2+.01],z,.02,'white')
        wall(world,'arrow_panel',[x,y-w/2],[x,y+w/2],.19,.03,'black',bottom=z)
    # Hide ceiling in overview viewer only; it remains visible to sensor cameras.
    visual=root.find('visual')
    if visual is None: visual=ET.SubElement(root,'visual')
    if visual.find('global') is None: ET.SubElement(visual,'global',offwidth='960',offheight='720')
    ET.SubElement(visual,'headlight',ambient='0.2 0.2 0.2',diffuse='0 0 0',specular='0 0 0')
