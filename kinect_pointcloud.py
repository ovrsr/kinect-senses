"""
kinect_pointcloud.py — colored 3D point cloud from the Kinect.

Unprojects the RGB-registered depth into metric 3D points, colors each from the
RGB frame, and exports a binary PLY (opens in MeshLab / CloudCompare / Blender).
Also renders the cloud from a rotated viewpoint so the 3D structure is visible
as a plain image (a PLY can't be viewed inline).

World convention in the PLY: X right, Y up, Z forward (metres), camera at origin.

CLI:
    python kinect_pointcloud.py out/cloud
"""

from __future__ import annotations
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import kinect

_W, _H = kinect._W, kinect._H
_FX, _FY, _CX, _CY = kinect._FX, kinect._FY, kinect._CX, kinect._CY


def capture(index: int = 0, n_denoise: int = 5, z_range=(0.4, 5.0)):
    """Return (points Nx3 metres [X right, Y up, Z fwd], colors Nx3 uint8)."""
    depth = kinect.get_depth_mm_denoised(index, n=n_denoise, registered=True)
    rgb = kinect.get_rgb(index)
    kinect.stop()

    z = depth.astype(np.float32) / 1000.0
    valid = (z >= z_range[0]) & (z <= z_range[1])
    vs, us = np.nonzero(valid)
    zz = z[vs, us]
    x = (us.astype(np.float32) - _CX) * zz / _FX
    y = (vs.astype(np.float32) - _CY) * zz / _FY
    # camera Y is down; flip so PLY is Y-up
    pts = np.stack([x, -y, zz], axis=-1)
    cols = rgb[vs, us]
    return pts, cols


def save_ply(path: str, pts: np.ndarray, cols: np.ndarray):
    """Write a binary_little_endian PLY with per-vertex color."""
    n = pts.shape[0]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n").encode("ascii")
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("r", "u1"), ("g", "u1"), ("b", "u1")])
    v = np.empty(n, dt)
    v["x"], v["y"], v["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    v["r"], v["g"], v["b"] = cols[:, 0], cols[:, 1], cols[:, 2]
    with open(path, "wb") as f:
        f.write(header)
        f.write(v.tobytes())
    return n


def _rot(yaw_deg, pitch_deg):
    a, b = np.radians(yaw_deg), np.radians(pitch_deg)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)], [0, np.sin(b), np.cos(b)]])
    return (Rx @ Ry).astype(np.float32)


def render(pts, cols, yaw=35.0, pitch=12.0, size=(760, 560), splat=2):
    """Orthographic render of the cloud from a rotated view (painter's algo)."""
    W, H = size
    c = pts.mean(axis=0)
    rot = (pts - c) @ _rot(yaw, pitch).T
    sx, sy, depth = rot[:, 0], rot[:, 1], rot[:, 2]
    span = max(np.percentile(np.abs(sx), 96), np.percentile(np.abs(sy), 96), 1e-3)
    scale = 0.45 * min(W, H) / span
    px = np.round(W / 2 + sx * scale).astype(np.int32)
    py = np.round(H / 2 - sy * scale).astype(np.int32)

    canvas = np.zeros((H, W, 3), np.uint8)
    order = np.argsort(depth)[::-1]           # far first; near overwrites
    px, py, colz = px[order], py[order], cols[order]
    for dx in range(splat):
        for dy in range(splat):
            qx, qy = px + dx, py + dy
            m = (qx >= 0) & (qx < W) & (qy >= 0) & (qy < H)
            canvas[qy[m], qx[m]] = colz[m]
    return Image.fromarray(canvas, "RGB")


def _stats(pts):
    return {
        "points": int(pts.shape[0]),
        "bounds_m": {
            "x": [round(float(pts[:, 0].min()), 2), round(float(pts[:, 0].max()), 2)],
            "y": [round(float(pts[:, 1].min()), 2), round(float(pts[:, 1].max()), 2)],
            "z": [round(float(pts[:, 2].min()), 2), round(float(pts[:, 2].max()), 2)]},
    }


def _main(argv):
    prefix = argv[0] if argv else "out/cloud"
    Path(prefix).parent.mkdir(parents=True, exist_ok=True)
    pts, cols = capture()
    if pts.shape[0] == 0:
        print("no depth points captured"); return 1
    n = save_ply(f"{prefix}.ply", pts, cols)
    render(pts, cols, yaw=35, pitch=12).save(f"{prefix}_view1.png")
    render(pts, cols, yaw=-35, pitch=8).save(f"{prefix}_view2.png")
    import json
    print(json.dumps(_stats(pts), indent=2))
    print(f"saved {prefix}.ply ({n:,} points), {prefix}_view1.png, {prefix}_view2.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
