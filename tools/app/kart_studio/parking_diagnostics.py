"""Offline visualization of the same clearance predicates used by World.clear."""
from shapely.geometry import Polygon, mapping
from shapely.ops import nearest_points


def clearance_diagnostic(world, pose, domain=None):
    from kart_hdmap.parking_geometry import footprint, inside, separated
    body = footprint(pose, world.front, world.rear, world.width)
    shape = Polygon(body)
    surface = world.surface if hasattr(world, 'surface') else Polygon(world.outer)
    hits = []

    def hit(kind, geometry, boundary, outside):
        a, b = nearest_points(shape, boundary)
        hits.append(dict(kind=kind, geometry=mapping(geometry),
                         marker=list(b.coords[0]), closest=[list(a.coords[0]), list(b.coords[0])],
                         outside=mapping(outside), distance_m=shape.distance(boundary)))

    if not surface.contains(shape) or surface.boundary.distance(shape) <= world.margin:
        hit('走行可能領域の境界', surface.boundary, surface.boundary, shape.difference(surface))
    if domain is not None and (not all(inside(p, domain) for p in body) or not separated(body, domain, world.margin)):
        area = Polygon(domain)
        hit('駐車枠の境界', area.boundary, area.boundary, shape.difference(area))
    for index, obstacle in enumerate(world.obstacles):
        if (any(inside(p, obstacle) for p in body) or inside(obstacle[0], body)
                or not separated(body, obstacle, world.margin)):
            area = Polygon(obstacle)
            hit(f'障害物 {index+1}', area, area.boundary, shape.intersection(area))
    return dict(body=body, margin_m=world.margin,
                clearance=mapping(shape.buffer(world.margin)), surface=mapping(surface), hits=hits)
