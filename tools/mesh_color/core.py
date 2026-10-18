"""RGB projection without changing mesh geometry; optical coordinates are x right/y down/z forward."""
import numpy as np
from scipy.spatial.transform import Rotation


def transform(value):
    a, t = value['axis_angle'], value['translation']
    axis = np.array([a.get(k, 0.) for k in 'xyz'], float)
    angle = np.deg2rad(float(a.get('angle_degrees', 0.)))
    if not np.isfinite([*axis, angle, *t.values()]).all():
        raise ValueError('Nonfinite camera transform')
    if abs(angle) > 1e-12 and np.linalg.norm(axis) < 1e-12:
        raise ValueError('Invalid rotation axis')
    matrix = np.eye(4)
    if abs(angle) > 1e-12:
        matrix[:3,:3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()
    matrix[:3,3] = [t.get(k,0.) for k in 'xyz']
    return matrix


def project(vertices, world_from_camera, k, width, height):
    camera = (vertices-world_from_camera[:3,3]) @ world_from_camera[:3,:3]
    z = camera[:,2]
    uv = camera @ k.T
    uv = uv[:,:2]/np.maximum(z[:,None], 1e-10)
    good = (z>.05)&np.isfinite(uv).all(axis=1)&(uv[:,0]>=1)&(uv[:,0]<width-1)&(uv[:,1]>=1)&(uv[:,1]<height-1)
    return camera, uv, np.flatnonzero(good)


def paint(vertices, ray_mesh, image, world_from_camera, k, colors, best, tolerance=.02):
    _, uv, ids = project(vertices,world_from_camera,k,image.shape[1],image.shape[0])
    origin = world_from_camera[:3,3]
    assigned = 0
    for start in range(0,len(ids),20000):
        selected = ids[start:start+20000]
        directions = vertices[selected]-origin
        distances = np.linalg.norm(directions,axis=1)
        locations, rays, _ = ray_mesh.ray.intersects_location(
            np.broadcast_to(origin,directions.shape),directions/distances[:,None],multiple_hits=False)
        if not len(rays): continue
        hit = np.linalg.norm(locations-origin,axis=1)
        visible = abs(hit-distances[rays]) <= tolerance
        idx = selected[rays[visible]]
        score = distances[rays[visible]]
        better = score < best[idx]
        idx, score = idx[better],score[better]
        pixels = np.rint(uv[idx]).astype(int)
        colors[idx] = image[pixels[:,1],pixels[:,0]]
        best[idx] = score
        assigned += len(idx)
    return assigned
