"""
kinect_pose.py — 3D body skeleton by fusing RGB pose estimation with depth.

The original Kinect skeleton lived in Microsoft's SDK, not the hardware. Here we
run MediaPipe BlazePose on the RGB frame (33 landmarks), then lift each joint to
real 3D by sampling the RGB-registered depth at its pixel and unprojecting with
the camera intrinsics — a metric skeleton in millimetres.

Needs the pose model (auto-downloaded on first run) and:  pip install mediapipe

CLI:
    python kinect_pose.py out/pose
"""

from __future__ import annotations
import json
import sys
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import kinect

_ROOT = Path(__file__).resolve().parent
MODEL_PATH = _ROOT / "models" / "pose_landmarker_full.task"
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_full/float16/latest/pose_landmarker_full.task")

_W, _H = kinect._W, kinect._H
_FX, _FY, _CX, _CY = kinect._FX, kinect._FY, kinect._CX, kinect._CY

LANDMARK_NAMES = [
    "nose", "left_eye_inner", "left_eye", "left_eye_outer", "right_eye_inner",
    "right_eye", "right_eye_outer", "left_ear", "right_ear", "mouth_left",
    "mouth_right", "left_shoulder", "right_shoulder", "left_elbow",
    "right_elbow", "left_wrist", "right_wrist", "left_pinky", "right_pinky",
    "left_index", "right_index", "left_thumb", "right_thumb", "left_hip",
    "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle",
    "left_heel", "right_heel", "left_foot_index", "right_foot_index"]

# BlazePose skeleton edges (body + light face)
POSE_CONNECTIONS = [
    (0, 2), (0, 5), (2, 7), (5, 8), (9, 10),                    # face
    (11, 12), (11, 23), (12, 24), (23, 24),                     # torso
    (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),  # left arm
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),  # right arm
    (23, 25), (25, 27), (27, 29), (29, 31), (27, 31),           # left leg
    (24, 26), (26, 28), (28, 30), (30, 32), (28, 32)]           # right leg

# "major" joints reported in the compact summary
MAJOR = ["nose", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
         "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee",
         "right_knee", "left_ankle", "right_ankle"]

_LM = None


def _ensure_model() -> str:
    if not MODEL_PATH.exists():
        MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return str(MODEL_PATH)


def _landmarker():
    global _LM
    if _LM is None:
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        opts = vision.PoseLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=_ensure_model()),
            running_mode=vision.RunningMode.IMAGE, num_poses=1)
        _LM = vision.PoseLandmarker.create_from_options(opts)
    return _LM


def _depth_at(depth_mm, u, v, half=4):
    lo_u, hi_u = max(0, u - half), min(_W, u + half + 1)
    lo_v, hi_v = max(0, v - half), min(_H, v + half + 1)
    win = depth_mm[lo_v:hi_v, lo_u:hi_u]
    vals = win[win > 0]
    return float(np.median(vals)) if vals.size else None


def detect(index: int = 0, n_denoise: int = 3):
    """Return (annotated PIL image, result dict). Requires a person in view."""
    import mediapipe as mp
    rgb = kinect.get_rgb(index)
    depth = kinect.get_depth_mm_denoised(index, n=n_denoise, registered=True)
    kinect.stop()

    mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                      data=np.ascontiguousarray(rgb))
    res = _landmarker().detect(mp_img)

    base = kinect.auto_normalize_rgb(rgb)[0]
    img = Image.fromarray(base, "RGB")
    dr = ImageDraw.Draw(img)
    fnt = kinect._font(14)

    if not res.pose_landmarks:
        dr.text((8, 8), "no person detected", font=fnt, fill=(255, 200, 200))
        return img, {"person": False}

    lms = res.pose_landmarks[0]
    joints = {}
    dmin, dmax = 400.0, 4500.0
    for i, lm in enumerate(lms):
        u = int(round(lm.x * _W))
        v = int(round(lm.y * _H))
        vis = float(lm.visibility)
        z = _depth_at(depth, u, v) if (0 <= u < _W and 0 <= v < _H) else None
        xyz = None
        if z:
            zz = z / 1000.0
            xyz = [round((u - _CX) * zz / _FX, 3),
                   round((v - _CY) * zz / _FY, 3), round(zz, 3)]
        joints[LANDMARK_NAMES[i]] = {
            "px": [u, v], "visibility": round(vis, 2),
            "depth_m": round(z / 1000.0, 3) if z else None, "xyz_m": xyz}

    # draw skeleton: edges green (dim if a joint is occluded), joints colored by depth
    for a, b in POSE_CONNECTIONS:
        ja, jb = joints[LANDMARK_NAMES[a]], joints[LANDMARK_NAMES[b]]
        good = ja["visibility"] > 0.5 and jb["visibility"] > 0.5
        col = (60, 220, 90) if good else (70, 90, 70)
        dr.line([tuple(ja["px"]), tuple(jb["px"])], fill=col, width=3 if good else 1)
    for name, j in joints.items():
        if j["visibility"] < 0.4:
            continue
        u, v = j["px"]
        if j["depth_m"]:
            t = np.clip((j["depth_m"] * 1000 - dmin) / (dmax - dmin), 0, 1)
            c = tuple(int(x) for x in kinect._cmap(np.array([t]))[0])
        else:
            c = (150, 150, 150)
        dr.ellipse([u - 5, v - 5, u + 5, v + 5], fill=c, outline=(0, 0, 0))

    # summary: torso distance = median depth of shoulders+hips
    torso = [joints[k]["depth_m"] for k in
             ("left_shoulder", "right_shoulder", "left_hip", "right_hip")
             if joints[k]["depth_m"]]
    dist = round(float(np.median(torso)), 2) if torso else None
    vis_pts = [j["px"] for j in joints.values() if j["visibility"] > 0.4]
    bbox = None
    if vis_pts:
        xs = [p[0] for p in vis_pts]
        ys = [p[1] for p in vis_pts]
        bbox = [min(xs), min(ys), max(xs), max(ys)]

    dr.text((8, 8), f"person @ {dist} m" if dist else "person (no torso depth)",
            font=kinect._font(16), fill=(255, 230, 120))

    summary = {
        "person": True,
        "torso_distance_m": dist,
        "bbox_xyxy": bbox,
        "joints_3d": int(sum(1 for j in joints.values() if j["xyz_m"])),
        "major_joints": {k: joints[k] for k in MAJOR},
        "all_joints": joints,
    }
    return img, summary


def _main(argv):
    prefix = argv[0] if argv else "out/pose"
    Path(prefix).parent.mkdir(parents=True, exist_ok=True)
    img, summary = detect()
    img.save(f"{prefix}_pose.png")
    with open(f"{prefix}_pose.json", "w") as fh:
        json.dump(summary, fh, indent=2)
    compact = {k: v for k, v in summary.items() if k != "all_joints"}
    print(json.dumps(compact, indent=2))
    print(f"saved {prefix}_pose.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
