import { $, state, action, on } from "./api.js";
import { hdmapPaths } from "./mesh_layers.js";
import { createCanvasViewer } from "./mesh_canvas.js";

const dialog = $("mesh-dialog"), host = $("mesh-viewport");
let viewer = null, controller = null, owner = null, sequence = 0;
function disposeObject(object) {
  object.traverse(o => {
    o.geometry?.dispose();
    for (const material of [].concat(o.material || [])) material.dispose();
  });
}
function closeViewer() {
  sequence++;
  controller?.abort(); controller = null;
  if (viewer) {
    if (viewer.software) viewer.software.dispose();
    else {
    viewer.observer.disconnect(); viewer.controls.dispose();
    disposeObject(viewer.scene);
    viewer.renderer.dispose(); viewer.renderer.forceContextLoss();
    }
    viewer = null;
  }
  host.replaceChildren();
  owner = null;
}
dialog.addEventListener("close", closeViewer);
on("document", () => { if (dialog.open && owner !== state.id) dialog.close(); });

function draw() {
  if (viewer && dialog.open) viewer.renderer.render(viewer.scene, viewer.camera);
}
function fit(top = false) {
  if (!viewer) return;
  if (viewer.software) { viewer.software.fit(top); return; }
  const { THREE: T, scene, camera, controls } = viewer;
  const box = new T.Box3().setFromObject(scene);
  if (box.isEmpty()) return;
  const center = box.getCenter(new T.Vector3());
  const radius = Math.max(1, box.getSize(new T.Vector3()).length() / 2);
  const distance = radius / Math.sin(camera.fov * Math.PI / 360) * Math.max(1, 1 / camera.aspect) * 1.15;
  controls.target.copy(center);
  camera.position.copy(center).add(new T.Vector3(...(top ? [0, -0.0001, 1] : [1, -1, 0.85])).normalize().multiplyScalar(distance));
  camera.near = Math.max(0.001, distance / 10000); camera.far = distance * 100;
  camera.updateProjectionMatrix(); controls.update(); draw();
}
function updateLayers() {
  if (!viewer) return;
  if (viewer.software) {
    viewer.software.update({ mesh: $("mesh-visible").checked, cloud: $("mesh-cloud").checked,
      lines: $("mesh-hdmap").checked, opacity: Number($("mesh-opacity").value),
      z: Number.isFinite(Number($("mesh-line-z").value)) ? Number($("mesh-line-z").value) : 0 });
    return;
  }
  viewer.mesh.visible = $("mesh-visible").checked;
  viewer.mesh.material.opacity = Number($("mesh-opacity").value);
  viewer.cloud.visible = $("mesh-cloud").checked;
  viewer.lines.visible = $("mesh-hdmap").checked;
  const z = Number($("mesh-line-z").value);
  if (Number.isFinite(z)) viewer.lines.position.z = z;
  draw();
}
for (const id of ["mesh-visible", "mesh-opacity", "mesh-cloud", "mesh-hdmap", "mesh-line-z"])
  $(id).addEventListener("input", updateLayers);
action("mesh-fit", () => fit());
action("mesh-top", () => fit(true));

action("mesh-open", async () => {
  closeViewer();
  const ticket = sequence, id = state.id;
  if (!id) return;
  owner = id;
  const doc = structuredClone(state.doc), cloudData = state.cloud;
  const lo = state.zmin, hi = state.zmax;
  const abort = new AbortController(); controller = abort;
  dialog.showModal();
  $("mesh-info").textContent = "元map座標のmeshを読み込み中…";
  try {
    const response = await fetch('/api/mesh-overlay?id=' + encodeURIComponent(id), { signal: abort.signal });
    const info = await response.json();
    if (!response.ok) throw Error(info.error);
    if (info.frame !== 'map' || info.source !== 'before_adjustment' || cloudData?.frame !== 'map')
      throw Error('点群とmeshのmap座標を確認できません');
    const [T, { PLYLoader }, { OrbitControls }, file] = await Promise.all([
      import('./vendor/three/three.module.js'), import('./vendor/three/PLYLoader.js'),
      import('./vendor/three/OrbitControls.js'), fetch(info.url, { signal: abort.signal }),
    ]);
    if (!file.ok) throw Error('meshの取得に失敗しました');
    const buffer = await file.arrayBuffer();
    if (sequence !== ticket || !dialog.open) return;
    const geometry = new PLYLoader().parse(buffer);
    const positions = geometry.getAttribute('position');
    if (!positions || !positions.count || !positions.array.every(Number.isFinite)) {
      geometry.dispose(); throw Error('meshの座標が不正です');
    }
    geometry.computeVertexNormals();
    let paths;
    try { paths = hdmapPaths(doc); } catch (error) { geometry.dispose(); throw error; }
    const scene = new T.Scene(); scene.background = new T.Color(0x182331);
    const camera = new T.PerspectiveCamera(45, 1, 0.01, 10000); camera.up.set(0, 0, 1);
    let renderer;
    const points = cloudData.points.filter(p => p[2] >= lo && p[2] <= hi);
    try { renderer = new T.WebGLRenderer({ antialias: true }); }
    catch (error) {
      try {
        viewer = { software: createCanvasViewer(host, geometry, points, paths) };
        updateLayers(); fit();
        $("mesh-info").textContent = `${doc.title} · map / m · ${info.colored ? "RGB着色 " + (info.color_report.coverage * 100).toFixed(1) + "%" : "単色"} · 互換表示（WebGL2初期化不可） · 表示 ${viewer.software.faceCount.toLocaleString()} / ${info.face.toLocaleString()} 面、点群 ${viewer.software.cloudCount.toLocaleString()} 点。間引き・簡易重なり表示のため欠損や遮蔽の精密評価には使わないでください。元データは保持しています。`;
      } finally { geometry.dispose(); }
      return;
    }
    renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2)); host.append(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    const mesh = new T.Mesh(geometry, new T.MeshBasicMaterial({ vertexColors: !!geometry.getAttribute("color"), color: geometry.getAttribute("color") ? 0xffffff : 0xaaaaaa, side: T.DoubleSide, transparent: true, opacity: 0.55, depthWrite: false }));
    scene.add(mesh, new T.AmbientLight(0xffffff, 1.5));
    const light = new T.DirectionalLight(0xffffff, 2); light.position.set(3, -4, 8); scene.add(light);
    const pg = new T.BufferGeometry(); pg.setAttribute('position', new T.Float32BufferAttribute(points.flat(), 3));
    const cloud = new T.Points(pg, new T.PointsMaterial({ color: 0xe88cff, size: 2, sizeAttenuation: false }));
    scene.add(cloud);
    const lines = new T.Group();
    for (const path of paths) {
      const g = new T.BufferGeometry(); g.setAttribute('position', new T.Float32BufferAttribute(path.points.flat(), 3));
      const line = new T.Line(g, new T.LineBasicMaterial({ color: path.color }));
      lines.add(line);
    }
    scene.add(lines, new T.AxesHelper(1));
    const observer = new ResizeObserver(() => {
      if (!viewer) return;
      const width = Math.max(1, host.clientWidth), height = Math.max(1, host.clientHeight);
      renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); draw();
    });
    viewer = { THREE: T, renderer, scene, camera, controls, observer, mesh, cloud, lines };
    renderer.setSize(host.clientWidth, host.clientHeight, false); camera.aspect = host.clientWidth / host.clientHeight;
    observer.observe(host); controls.addEventListener('change', draw);
    updateLayers(); fit();
    $("mesh-info").textContent = `${doc.title} · map / m · ${info.colored ? "RGB着色 " + (info.color_report.coverage * 100).toFixed(1) + "%" : "単色"} · ${info.face.toLocaleString()} 面 · 点群 ${points.length.toLocaleString()} 点 · HDMap r${doc.revision}${state.dirty ? '（未保存編集を含む）' : ''}${info.stale ? ' · meshは旧深度の成果物' : ''} · 追加TFなし`;
  } catch (error) {
    if (ticket !== sequence || abort.signal.aborted) return;
    closeViewer();
    $("mesh-info").textContent = error.message;
  }
});
