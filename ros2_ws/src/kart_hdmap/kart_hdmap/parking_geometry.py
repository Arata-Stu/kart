"""ROS-independent conservative polygon and swept rectangle checks (meters)."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def edges(p):
    return zip(p, p[1:] + p[:1])


def point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = dx * dx + dy * dy
    t = max(0., min(1., ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/d)) if d else 0.
    q = (a[0]+t*dx, a[1]+t*dy)
    return distance(p, q), t, q


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def intersect(a, b, c, d):
    if min(point_segment(p, u, v)[0] for p, u, v in
           ((a,c,d), (b,c,d), (c,a,b), (d,a,b))) <= 1e-9:
        return True
    return cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0


def inside(p, poly):
    if any(point_segment(p,a,b)[0] <= 1e-9 for a,b in edges(poly)):
        return False  # Boundary contact is never free.
    result = False
    for a,b in edges(poly):
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                result = not result
    return result


def polygon(raw):
    if not isinstance(raw, list) or not 3 <= len(raw) <= 200:
        raise ValueError('Polygon requires 3..200 vertices')
    p = [tuple(float(v) for v in xy) for xy in raw]
    if any(len(xy) != 2 or not all(math.isfinite(v) for v in xy) for xy in p):
        raise ValueError('Invalid polygon coordinate')
    e = list(edges(p))
    for i,(a,b) in enumerate(e):
        if distance(a,b) < 1e-6:
            raise ValueError('Duplicate polygon vertices')
        for j,(c,d) in enumerate(e):
            if j > i+1 and not (i == 0 and j == len(e)-1) and intersect(a,b,c,d):
                raise ValueError('Self-intersecting polygon')
    if abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in e)) < 1e-6:
        raise ValueError('Degenerate polygon')
    return p


def separated(a, b, margin):
    for u,v in edges(a):
        for x,y in edges(b):
            if intersect(u,v,x,y):
                return False
            if min(point_segment(p,q,r)[0] for p,q,r in
                   ((u,x,y),(v,x,y),(x,u,v),(y,u,v))) <= margin:
                return False
    return True


def footprint(pose, front, rear, width):
    x,y,yaw = pose
    c,s = math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a,b in
            ((front,width/2),(front,-width/2),(-rear,-width/2),(-rear,width/2))]


class World:
    def __init__(self, outer, obstacles, front, rear, width, margin):
        self.outer, self.obstacles = polygon(outer), [polygon(p) for p in obstacles]
        self.front, self.rear, self.width, self.margin = front,rear,width,margin
        self.radius = math.hypot(max(front,rear), width/2)

    def clear(self, pose, domain=None, extra=0.):
        body = footprint(pose,self.front,self.rear,self.width)
        for boundary in [self.outer] + ([domain] if domain is not None else []):
            if not all(inside(p,boundary) for p in body) or not separated(body,boundary,self.margin+extra):
                return False
        for obstacle in self.obstacles:
            if (any(inside(p,obstacle) for p in body) or inside(obstacle[0],body)
                    or not separated(body,obstacle,self.margin+extra)):
                return False
        return True

    def swept(self, poses, domain=None):
        for a,b in zip(poses,poses[1:]):
            motion = distance(a,b) + self.radius*abs(wrap(b[2]-a[2]))
            # Inflation bounds any point's displacement between sampled poses.
            if not self.clear(a,domain,motion/2) or not self.clear(b,domain,motion/2):
                return False
        return bool(poses) and self.clear(poses[-1],domain)


class SurfaceWorld(World):
    """Union surface retains interior holes; clearance applies only to real edges."""
    def __init__(self, surface, obstacles, front, rear, width, margin):
        self.surface = surface
        self.obstacles = [polygon(p) for p in obstacles]
        self.front, self.rear, self.width, self.margin = front, rear, width, margin
        self.radius = math.hypot(max(front, rear), width/2)

    def clear(self, pose, domain=None, extra=0.):
        from shapely.geometry import Polygon
        body = footprint(pose, self.front, self.rear, self.width)
        shape = Polygon(body)
        clearance = self.margin+extra
        if not self.surface.contains(shape) or self.surface.boundary.distance(shape) <= clearance:
            return False
        if domain is not None and (not all(inside(p, domain) for p in body) or not separated(body, domain, clearance)):
            return False
        for obstacle in self.obstacles:
            if (any(inside(p, obstacle) for p in body) or inside(obstacle[0], body)
                    or not separated(body, obstacle, clearance)):
                return False
        return True
