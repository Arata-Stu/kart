# Vehicle assets

rc-simから2026-10-10に移植。実行時に元checkoutを参照しない。
`tt02_cad/manifest.json`は元FreeCAD形状の出典・export時の寸法/配置を保持する。
TT02Frame/MountBody/D455Envelope/SilkyEvCamOfficialCADのOBJは表示用、
接触・慣性はmanifestの外接箱で近似する。自由形状の穴は接触で再現しない。
`vehicle.json`の質量、Jetson形状、CAD配置は仮値。実測により更新する。
新しい実車CADとの差は自動追従しないため、再export時にmanifestも同時更新する。
