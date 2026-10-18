"""Validate optional NVIDIA dense reconstruction outputs without changing map TF."""

DENSE_FILES = (
    "mesh.ply", "mesh_before_adjustment.ply", "occupancy_map.png",
    "occupancy_map.yaml", "occupancy_map_before_adjustment.png",
    "occupancy_map_before_adjustment.yaml", "ground_plane.yaml", "T_world_to_z0.json",
)


def options(workflow):
    enabled = workflow.get("dense_reconstruction", False)
    resolution = workflow.get("fs_model_res", "low_res")
    if type(enabled) is not bool or resolution not in ("low_res", "high_res"):
        raise ValueError("dense_reconstruction must be boolean; fs_model_res must be low_res/high_res")
    return enabled, resolution


def collect(run_dir):
    required = ("mesh.ply", "occupancy_map.png", "occupancy_map.yaml", "ground_plane.yaml")
    for name in required:
        path = run_dir / name
        if path.is_symlink() or not path.is_file() or not path.stat().st_size:
            raise ValueError(f"公式dense成果物がありません: {path}")
    with (run_dir / "mesh.ply").open("rb") as stream:
        header = stream.read(65536).split(b"end_header", 1)
    if len(header) != 2 or header[0].splitlines()[:1] != [b"ply"]:
        raise ValueError("mesh.ply has no valid PLY header")
    counts = {}
    for line in header[0].splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[:1] == [b"element"] and fields[1] in (b"vertex", b"face"):
            counts[fields[1].decode()] = int(fields[2])
    if counts.get("vertex", 0) <= 0 or counts.get("face", 0) <= 0:
        raise ValueError("mesh.ply is empty (vertices/faces required)")
    return dict(
        frame="ground_aligned", vertices=counts["vertex"], faces=counts["face"],
        files=[name for name in DENSE_FILES if (run_dir / name).is_file() and not (run_dir / name).is_symlink()],
    )
