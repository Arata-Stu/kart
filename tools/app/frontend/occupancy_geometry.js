// Pixel edge coordinates -> screen; YAML origin is the bottom-left grid edge.
export function rasterTransform(origin, resolution, height, project, scale) {
  const [x, y, yaw] = origin, [sx, sy] = project([x, y]);
  const c = Math.cos(yaw), s = Math.sin(yaw), k = resolution * scale;
  return [k * c, -k * s, k * s, k * c,
    sx - k * s * height, sy - k * c * height];
}
