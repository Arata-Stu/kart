// CPU-only inspection fallback. Sampling affects display only, never saved data.
export function projectPoint(p, center, yaw, pitch) {
  const x = p[0] - center[0], y = p[1] - center[1], z = p[2] - center[2];
  const u = Math.cos(yaw) * x - Math.sin(yaw) * y;
  const v = Math.sin(yaw) * x + Math.cos(yaw) * y;
  return [u, Math.sin(pitch) * v - Math.cos(pitch) * z,
    Math.cos(pitch) * v + Math.sin(pitch) * z];
}

export function createCanvasViewer(host, geometry, points, paths) {
  const canvas = document.createElement('canvas'), ctx = canvas.getContext('2d');
  if (!ctx) throw Error('Canvas 2Dも利用できません。外部PLYビューアで確認してください');
  host.append(canvas);
  const positions = geometry.attributes.position;
  const indices = geometry.index?.array;
  const faceCount = Math.floor((indices?.length ?? positions.count) / 3);
  const stride = Math.max(1, Math.ceil(faceCount / 12000));
  const triangles = [];
  const read = i => [positions.getX(i), positions.getY(i), positions.getZ(i)];
  for (let f = 0; f < faceCount; f += stride)
    triangles.push([0, 1, 2].map(k => read(indices ? indices[f * 3 + k] : f * 3 + k)));
  const cloudStride = Math.max(1, Math.ceil(points.length / 15000));
  const cloud = points.filter((_, i) => i % cloudStride === 0);
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.count; i++) read(i).forEach((v, k) => {
    min[k] = Math.min(min[k], v); max[k] = Math.max(max[k], v);
  });
  for (const p of [...cloud, ...paths.flatMap(path => path.points)]) p.forEach((v, k) => {
    min[k] = Math.min(min[k], v); max[k] = Math.max(max[k], v);
  });
  const center = min.map((v, k) => (v + max[k]) / 2);
  const diameter = Math.max(1, Math.hypot(...min.map((v, k) => max[k] - v)));
  let yaw = -0.7, pitch = 0.8, zoom = 1, pan = [0, 0], drag = null, raf = 0;
  let settings = { mesh: true, cloud: true, lines: true, opacity: 0.55, z: 0 };
  function render() {
    raf = 0;
    const w = Math.max(1, host.clientWidth), h = Math.max(1, host.clientHeight);
    const dpr = Math.min(devicePixelRatio || 1, 2);
    canvas.width = w * dpr; canvas.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#182331'; ctx.fillRect(0, 0, w, h);
    const scale = Math.min(w, h) * 0.85 / diameter * zoom;
    const screen = p => {
      const q = projectPoint(p, center, yaw, pitch);
      return [w / 2 + pan[0] + q[0] * scale, h / 2 + pan[1] - q[1] * scale, q[2]];
    };
    if (settings.mesh) {
      const faces = triangles.map(t => t.map(screen));
      faces.sort((a, b) => a.reduce((s, p) => s + p[2], 0) - b.reduce((s, p) => s + p[2], 0));
      ctx.globalAlpha = settings.opacity; ctx.fillStyle = '#aab7c3';
      ctx.strokeStyle = '#65788b'; ctx.lineWidth = 0.4;
      for (const t of faces) {
        ctx.beginPath(); t.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]));
        ctx.closePath(); ctx.fill(); ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }
    if (settings.cloud) {
      ctx.fillStyle = '#e88cff';
      for (const point of cloud) { const p = screen(point); ctx.fillRect(p[0] - 1, p[1] - 1, 2, 2); }
    }
    if (settings.lines) for (const path of paths) {
      ctx.strokeStyle = '#' + path.color.toString(16).padStart(6, '0'); ctx.lineWidth = 1.5;
      ctx.beginPath();
      path.points.forEach((p, i) => {
        const q = screen([p[0], p[1], settings.z]);
        i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]);
      }); ctx.stroke();
    }
  }
  const draw = () => { if (!raf) raf = requestAnimationFrame(render); };
  canvas.oncontextmenu = e => e.preventDefault();
  canvas.onpointerdown = e => { drag = { x: e.clientX, y: e.clientY, button: e.button }; canvas.setPointerCapture(e.pointerId); };
  canvas.onpointermove = e => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (drag.button === 0) { yaw += dx * 0.008; pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, pitch + dy * 0.008)); }
    else { pan[0] += dx; pan[1] += dy; }
    drag.x = e.clientX; drag.y = e.clientY; draw();
  };
  canvas.onpointerup = canvas.onpointercancel = canvas.onlostpointercapture = () => { drag = null; };
  canvas.addEventListener('wheel', e => {
    e.preventDefault(); zoom = Math.max(0.05, Math.min(100, zoom * Math.exp(-e.deltaY * 0.001))); draw();
  }, { passive: false });
  canvas.style.touchAction = 'none';
  const observer = new ResizeObserver(draw); observer.observe(host);
  draw();
  return {
    faceCount: triangles.length, cloudCount: cloud.length,
    fit(top) { zoom = 1; pan = [0, 0]; yaw = top ? 0 : -0.7; pitch = top ? Math.PI / 2 : 0.8; draw(); },
    update(value) { settings = value; draw(); },
    dispose() { observer.disconnect(); cancelAnimationFrame(raf); canvas.remove(); },
  };
}
