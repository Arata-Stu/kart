#!/usr/bin/env python3
"""Rebuild deterministic multi-scale, nonperiodic visual textures (stdlib + NumPy)."""
import json
from pathlib import Path
import struct
import zlib
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def png(path, pixels):
    height, width = pixels.shape[:2]
    def chunk(tag, data):
        return struct.pack('!I',len(data))+tag+data+struct.pack('!I',zlib.crc32(tag+data)&0xffffffff)
    raw = b''.join(b'\0'+row.tobytes() for row in pixels)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw,9))+chunk(b'IEND',b''))


def pattern(width, height, rng):
    # Random overlapping rectangles and asymmetric L shapes, with several sizes.
    # Never a repeated chessboard or a frame-varying random pattern.
    image = np.full((height,width),210,dtype=np.uint8)
    for size in (5,12,26):
        for _ in range(max(30,width*height//(size*size*12))):
            x,y = int(rng.integers(width)),int(rng.integers(height))
            w,h = rng.integers(max(2,size//2),size*2,size=2)
            value = int(rng.choice([35,65,110,160,245]))
            image[y:min(height,y+h),x:min(width,x+w)] = value
            if rng.random()<.5:
                image[y:min(height,y+max(2,h//3)),x:min(width,x+w+size)] = value
    return np.repeat(image[:,:,None],3,axis=2)


def main():
    source = ROOT/'maps/sim/minicar_2026.json'
    doc = json.loads(source.read_text())
    doc['name'] = 'minicar_2026_features'
    doc['assumptions'].append('Synthetic VSLAM diagnostic textures on room/course walls and floor; not surveyed venue appearance. Geometry, collision, friction and route match minicar_2026. Fixed seed 20261011, no runtime image randomness.')
    targets = [('room_floor',1536,1024)] + [(f'room_wall_{i}',512,512) for i in range(4)]
    targets += [(f'barrier_{i}_{j}',256,256) for i,line in enumerate(doc['polylines']) for j in range(len(line['points'])-1)]
    targets += [(f'parking_wall_{i}_{j}',256,256) for i,p in enumerate(doc.get('parking',[])) if p.get('walls',True) for j in range(3)]
    directory = ROOT/'maps/sim/textures'
    directory.mkdir(exist_ok=True)
    rng = np.random.default_rng(20261011)
    textures = []
    for name,width,height in targets:
        file = f'textures/features_{name}.png'
        png(ROOT/'maps/sim'/file,pattern(width,height,rng))
        spec = {'geom':name,'file':file,'type':'2d' if name=='room_floor' else 'cube'}
        if name=='room_floor':
            spec['rgba'] = [0.45,0.45,0.46,1]
        textures.append(spec)
    doc['visual_textures'] = textures
    (ROOT/'maps/sim/minicar_2026_features.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n')
    print(f'Created {doc["name"]}: {len(textures)} texture assets')


if __name__=='__main__':main()
