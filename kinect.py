"""
kinect.py — ctypes access + perception toolkit for a Kinect v1 via libfreenect.

We load the mingw-built libfreenect_sync.dll (+ siblings in ./dist) and call the
C sync API directly — no compiled Python extension, so it is Python-3.14 safe.

Everything Claude "sees" comes out as files it can Read: PNG images and JSON.

Capabilities / CLI:
    python kinect.py info                 # load libs, report exports
    python kinect.py tilt [deg]           # level/aim the motor (-30..30), default 0
    python kinect.py capture <prefix>     # legacy: raw RGB + 11-bit depth PNGs
    python kinect.py look <prefix>        # the "glance": normalized RGB (+IR if dark),
                                          #   denoised metric depth heatmap w/ colorbar,
                                          #   a composite montage, and a JSON scene summary
    python kinect.py birdseye <prefix>    # top-down occupancy map (metric axes)
    python kinect.py ref <prefix>         # store a reference frame for motion detection
    python kinect.py motion <prefix>      # diff vs reference -> change overlay + stats
    python kinect.py audio <prefix> [sec] # record mic array -> WAV + waveform + spectrogram

Library API (import kinect): get_rgb, get_ir, get_depth (raw 11-bit),
    get_depth_mm, get_depth_mm_denoised, set_tilt, get_tilt, look, birdseye,
    set_reference, detect_motion, record_audio.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage

# --- native library location ------------------------------------------------

_ROOT = Path(__file__).resolve().parent
_DIST = _ROOT / "dist"

# freenect_video_format
FREENECT_VIDEO_RGB = 0
FREENECT_VIDEO_IR_8BIT = 2
# freenect_depth_format
FREENECT_DEPTH_11BIT = 0        # raw disparity, 0..2047 (2047 = no data)
FREENECT_DEPTH_REGISTERED = 4   # mm, aligned to the 640x480 RGB image (0 = no data)
FREENECT_DEPTH_MM = 5           # mm, not aligned to RGB

_W, _H = 640, 480
_NO_DISP = 2047                 # 11-bit "no data" sentinel

# Kinect v1 RGB-camera intrinsics (standard libfreenect/OpenNI defaults) used to
# unproject FREENECT_DEPTH_REGISTERED (which is aligned to the RGB frame).
_FX = _FY = 525.0
_CX, _CY = 319.5, 239.5

TILT_MIN, TILT_MAX = -30, 30
_TILT_STATUS = {0: "stopped", 1: "limit", 4: "moving"}


class RawTiltState(ctypes.Structure):
    _fields_ = [
        ("accelerometer_x", ctypes.c_int16),
        ("accelerometer_y", ctypes.c_int16),
        ("accelerometer_z", ctypes.c_int16),
        ("tilt_angle", ctypes.c_int8),
        ("tilt_status", ctypes.c_int),
    ]


# --- library loading --------------------------------------------------------

def _load():
    if not _DIST.is_dir():
        raise FileNotFoundError(f"dist directory not found: {_DIST}")
    os.add_dll_directory(str(_DIST))
    dll = _DIST / "libfreenect_sync.dll"
    if not dll.is_file():
        raise FileNotFoundError(f"missing {dll}")
    lib = ctypes.CDLL(str(dll))
    p_void_p = ctypes.POINTER(ctypes.c_void_p)
    p_u32 = ctypes.POINTER(ctypes.c_uint32)
    lib.freenect_sync_get_video.restype = ctypes.c_int
    lib.freenect_sync_get_video.argtypes = [p_void_p, p_u32, ctypes.c_int, ctypes.c_int]
    lib.freenect_sync_get_depth.restype = ctypes.c_int
    lib.freenect_sync_get_depth.argtypes = [p_void_p, p_u32, ctypes.c_int, ctypes.c_int]
    lib.freenect_sync_stop.restype = None
    lib.freenect_sync_stop.argtypes = []
    lib.freenect_sync_set_tilt_degs.restype = ctypes.c_int
    lib.freenect_sync_set_tilt_degs.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.freenect_sync_get_tilt_state.restype = ctypes.c_int
    lib.freenect_sync_get_tilt_state.argtypes = [
        ctypes.POINTER(ctypes.POINTER(RawTiltState)), ctypes.c_int]
    return lib


_LIB = None


def _lib():
    global _LIB
    if _LIB is None:
        _LIB = _load()
    return _LIB


# --- raw frame grabbing -----------------------------------------------------

def _grab_video(fmt: int, nbytes: int, index: int) -> bytes:
    lib = _lib()
    buf = ctypes.c_void_p()
    ts = ctypes.c_uint32()
    rc = lib.freenect_sync_get_video(ctypes.byref(buf), ctypes.byref(ts), index, fmt)
    if rc != 0 or not buf.value:
        raise RuntimeError(f"freenect_sync_get_video(fmt={fmt}) failed (rc={rc}); "
                           "is the Kinect connected and bound to libusbK/WinUSB?")
    return ctypes.string_at(buf, nbytes)


def _grab_depth(fmt: int, index: int) -> np.ndarray:
    lib = _lib()
    buf = ctypes.c_void_p()
    ts = ctypes.c_uint32()
    rc = lib.freenect_sync_get_depth(ctypes.byref(buf), ctypes.byref(ts), index, fmt)
    if rc != 0 or not buf.value:
        raise RuntimeError(f"freenect_sync_get_depth(fmt={fmt}) failed (rc={rc}); "
                           "is the Kinect connected and bound to libusbK/WinUSB?")
    raw = ctypes.string_at(buf, _W * _H * 2)
    return np.frombuffer(raw, dtype=np.uint16, count=_W * _H).reshape(_H, _W).copy()


def get_rgb(index: int = 0) -> np.ndarray:
    """RGB frame -> (480, 640, 3) uint8."""
    raw = _grab_video(FREENECT_VIDEO_RGB, _W * _H * 3, index)
    return np.frombuffer(raw, dtype=np.uint8, count=_W * _H * 3).reshape(_H, _W, 3).copy()


def get_ir(index: int = 0) -> np.ndarray:
    """8-bit IR frame -> (480, 640) uint8. Works in the dark (active IR illuminator)."""
    raw = _grab_video(FREENECT_VIDEO_IR_8BIT, _W * _H, index)
    return np.frombuffer(raw, dtype=np.uint8, count=_W * _H).reshape(_H, _W).copy()


def get_depth(index: int = 0) -> np.ndarray:
    """Legacy raw 11-bit depth -> (480, 640) uint16 (0..2047, 2047 = no data)."""
    return _grab_depth(FREENECT_DEPTH_11BIT, index)


def get_depth_mm(index: int = 0, registered: bool = True) -> np.ndarray:
    """Metric depth in millimetres -> (480, 640) uint16 (0 = no data).

    registered=True aligns depth to the RGB image (FREENECT_DEPTH_REGISTERED)."""
    fmt = FREENECT_DEPTH_REGISTERED if registered else FREENECT_DEPTH_MM
    return _grab_depth(fmt, index)


def stop() -> None:
    """Stop streams and release the device."""
    if _LIB is not None:
        _LIB.freenect_sync_stop()


# --- motor / tilt -----------------------------------------------------------

def get_tilt(index: int = 0) -> dict:
    lib = _lib()
    ptr = ctypes.POINTER(RawTiltState)()
    rc = lib.freenect_sync_get_tilt_state(ctypes.byref(ptr), index)
    if rc != 0 or not ptr:
        raise RuntimeError(f"freenect_sync_get_tilt_state failed (rc={rc})")
    s = ptr.contents
    return {
        "angle_deg": s.tilt_angle / 2.0,
        "accel": (s.accelerometer_x, s.accelerometer_y, s.accelerometer_z),
        "status": _TILT_STATUS.get(s.tilt_status, s.tilt_status),
    }


def set_tilt(degrees: float, index: int = 0) -> int:
    angle = max(TILT_MIN, min(TILT_MAX, int(round(degrees))))
    rc = _lib().freenect_sync_set_tilt_degs(angle, index)
    if rc != 0:
        raise RuntimeError(f"freenect_sync_set_tilt_degs({angle}) failed (rc={rc})")
    return angle


# --- depth denoise + visualization -----------------------------------------

def get_depth_mm_denoised(index: int = 0, n: int = 5, registered: bool = True) -> np.ndarray:
    """Median of n metric-depth frames (ignoring no-data) + small-hole fill."""
    frames = []
    for _ in range(max(1, n)):
        frames.append(get_depth_mm(index, registered).astype(np.float32))
    stack = np.stack(frames, axis=0)
    stack[stack == 0] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(stack, axis=0)
    med = np.nan_to_num(med, nan=0.0)
    return _holefill(med.astype(np.uint16))


def _holefill(depth_mm: np.ndarray, iterations: int = 3, min_valid: int = 6) -> np.ndarray:
    """Fill zero pixels with the local mean of valid neighbours (small holes only)."""
    d = depth_mm.astype(np.float32)
    k = np.ones((5, 5), np.float32)
    for _ in range(iterations):
        holes = d == 0
        if not holes.any():
            break
        valid = (~holes).astype(np.float32)
        ssum = ndimage.convolve(d, k, mode="constant")
        scnt = ndimage.convolve(valid, k, mode="constant")
        fillable = holes & (scnt >= min_valid)
        d[fillable] = ssum[fillable] / scnt[fillable]
    return d.astype(np.uint16)


# 5-stop ramp: near = red -> far = blue
_CMAP_POS = [0.0, 0.25, 0.5, 0.75, 1.0]
_CMAP_R = [255, 255, 0, 0, 0]
_CMAP_G = [0, 255, 255, 255, 0]
_CMAP_B = [0, 0, 0, 255, 255]


def _cmap(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    r = np.interp(t, _CMAP_POS, _CMAP_R)
    g = np.interp(t, _CMAP_POS, _CMAP_G)
    b = np.interp(t, _CMAP_POS, _CMAP_B)
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


def colorize_depth_mm(depth_mm: np.ndarray, dmin=None, dmax=None):
    """Colorize metric depth. Returns (rgb uint8, dmin_mm, dmax_mm)."""
    valid = depth_mm > 0
    out = np.zeros((*depth_mm.shape, 3), np.uint8)
    if not valid.any():
        return out, 0, 0
    v = depth_mm[valid].astype(np.float32)
    if dmin is None:
        dmin = float(np.percentile(v, 2))
    if dmax is None:
        dmax = float(np.percentile(v, 98))
    span = max(dmax - dmin, 1.0)
    t = (depth_mm.astype(np.float32) - dmin) / span
    rgb = _cmap(t)
    out[valid] = rgb[valid]
    return out, int(dmin), int(dmax)


def auto_normalize_rgb(rgb: np.ndarray):
    """Low-light brighten: percentile stretch + adaptive gamma. Returns (uint8, gamma)."""
    f = rgb.astype(np.float32)
    lum = f.mean()
    lo, hi = np.percentile(f, 1), np.percentile(f, 99)
    if hi - lo < 1:
        hi = lo + 1
    s = np.clip((f - lo) / (hi - lo), 0, 1)
    # darker scene -> stronger gamma lift
    gamma = 1.0 if lum > 90 else (0.45 if lum < 35 else 0.65)
    s = s ** gamma
    return (s * 255).astype(np.uint8), gamma


# --- fonts / drawing helpers ------------------------------------------------

def _font(size: int):
    for name in ("segoeui.ttf", "arial.ttf"):
        p = Path("C:/Windows/Fonts") / name
        if p.exists():
            try:
                return ImageFont.truetype(str(p), size)
            except Exception:
                pass
    return ImageFont.load_default()


def _depth_colorbar(height: int, dmin_mm: int, dmax_mm: int, width: int = 26):
    """Vertical colorbar image (near at top) with mm/m tick labels to its right."""
    strip = np.zeros((height, width, 3), np.uint8)
    t = np.linspace(0, 1, height)          # 0 at top = near
    strip[:] = _cmap(t)[:, None, :]
    pad = 78
    img = Image.new("RGB", (width + pad, height), (16, 16, 16))
    img.paste(Image.fromarray(strip), (0, 0))
    d = ImageDraw.Draw(img)
    fnt = _font(13)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = int(frac * (height - 1))
        mm = dmin_mm + frac * (dmax_mm - dmin_mm)
        d.line([(0, y), (width, y)], fill=(255, 255, 255), width=1)
        ty = min(max(y - 7, 0), height - 14)
        d.text((width + 4, ty), f"{mm/1000:.2f} m", font=fnt, fill=(230, 230, 230))
    return img


# --- scene understanding ----------------------------------------------------

def scene_summary(depth_mm: np.ndarray) -> dict:
    valid = depth_mm > 0
    total = depth_mm.size
    out = {
        "coverage_pct": round(100 * valid.sum() / total, 1),
        "valid_px": int(valid.sum()),
    }
    if not valid.any():
        return out
    v = depth_mm[valid].astype(np.float32)
    out["nearest_m"] = round(float(np.percentile(v, 1)) / 1000, 3)
    out["farthest_m"] = round(float(np.percentile(v, 99)) / 1000, 3)
    out["median_m"] = round(float(np.median(v)) / 1000, 3)
    # left / center / right thirds
    thirds = {}
    for name, sl in (("left", slice(0, _W // 3)),
                     ("center", slice(_W // 3, 2 * _W // 3)),
                     ("right", slice(2 * _W // 3, _W))):
        seg = depth_mm[:, sl]
        sv = seg[seg > 0]
        thirds[name] = {
            "coverage_pct": round(100 * sv.size / seg.size, 1),
            "nearest_m": round(float(np.percentile(sv, 1)) / 1000, 3) if sv.size else None,
        }
    out["thirds"] = thirds
    # foreground objects: near band = within 0.5 m of the nearest surface
    near = float(np.percentile(v, 1))
    band = valid & (depth_mm.astype(np.float32) < near + 500)
    lbl, ncomp = ndimage.label(band)
    if ncomp:
        sizes = ndimage.sum(np.ones_like(lbl), lbl, range(1, ncomp + 1))
        big = int((sizes > 800).sum())          # ignore specks < 800 px
        out["foreground_objects"] = big
    else:
        out["foreground_objects"] = 0
    return out


# --- the "glance" -----------------------------------------------------------

def look(prefix: str, n_denoise: int = 5, index: int = 0) -> dict:
    """Capture a rich, self-describing snapshot. Returns the scene summary dict
    and writes <prefix>_look.png (montage), <prefix>.json, and component PNGs."""
    pp = Path(prefix)
    pp.parent.mkdir(parents=True, exist_ok=True)

    rgb = get_rgb(index)
    rgb_norm, gamma = auto_normalize_rgb(rgb)
    dark = rgb.astype(np.float32).mean() < 35
    ir = get_ir(index) if dark else None
    depth = get_depth_mm_denoised(index, n=n_denoise, registered=True)

    dcolor, dmin, dmax = colorize_depth_mm(depth)
    summary = scene_summary(depth)
    summary["low_light"] = bool(dark)
    summary["rgb_gamma"] = gamma

    # left panel: normalized RGB (or IR if too dark to see anything in RGB)
    left_img = Image.fromarray(rgb_norm, "RGB")
    left_label = "RGB (auto-normalized)"
    if ir is not None:
        left_img = Image.fromarray(np.stack([ir] * 3, -1), "RGB")
        left_label = "IR (low light)"

    cbar = _depth_colorbar(_H, dmin, dmax)
    # compose montage
    gap, top, bottom = 16, 30, 96
    W = _W + gap + _W + gap + cbar.width
    H = top + _H + bottom
    canvas = Image.new("RGB", (W, H), (16, 16, 16))
    d = ImageDraw.Draw(canvas)
    fnt = _font(15)
    fnt_s = _font(13)
    canvas.paste(left_img, (0, top))
    canvas.paste(Image.fromarray(dcolor, "RGB"), (_W + gap, top))
    canvas.paste(cbar, (_W + gap + _W + gap, top))
    d.text((0, 6), left_label, font=fnt, fill=(230, 230, 230))
    d.text((_W + gap, 6), "Depth (metric, denoised) — near=red, far=blue",
           font=fnt, fill=(230, 230, 230))

    th = summary.get("thirds", {})
    def _t(n):
        x = th.get(n, {})
        return f"{n}: {x.get('nearest_m')}m/{x.get('coverage_pct')}%"
    lines = [
        f"coverage {summary['coverage_pct']}%   "
        f"nearest {summary.get('nearest_m')} m   median {summary.get('median_m')} m   "
        f"farthest {summary.get('farthest_m')} m",
        f"foreground objects: {summary.get('foreground_objects')}    "
        f"{_t('left')}   {_t('center')}   {_t('right')}",
    ]
    for i, ln in enumerate(lines):
        d.text((0, top + _H + 8 + i * 20), ln, font=fnt_s, fill=(200, 220, 200))

    look_png = f"{prefix}_look.png"
    canvas.save(look_png)
    Image.fromarray(rgb_norm, "RGB").save(f"{prefix}_rgb.png")
    Image.fromarray(dcolor, "RGB").save(f"{prefix}_depth.png")
    if ir is not None:
        Image.fromarray(ir).save(f"{prefix}_ir.png")
    with open(f"{prefix}.json", "w") as fh:
        json.dump(summary, fh, indent=2)
    summary["_montage"] = look_png
    return summary


# --- bird's-eye occupancy ---------------------------------------------------

def _unproject(depth_mm: np.ndarray):
    """Registered depth -> Nx3 metric points (X right, Y down, Z forward), meters."""
    v, u = np.nonzero(depth_mm)
    z = depth_mm[v, u].astype(np.float32) / 1000.0
    x = (u.astype(np.float32) - _CX) * z / _FX
    y = (v.astype(np.float32) - _CY) * z / _FY
    return np.stack([x, y, z], axis=-1)


def birdseye(prefix: str, x_range=(-2.0, 2.0), z_range=(0.3, 4.0),
             cell=0.02, n_denoise: int = 5, index: int = 0) -> dict:
    """Top-down occupancy map from registered depth. Camera at bottom-center,
    forward = up. Writes <prefix>_birdseye.png; returns basic stats."""
    pp = Path(prefix)
    pp.parent.mkdir(parents=True, exist_ok=True)
    depth = get_depth_mm_denoised(index, n=n_denoise, registered=True)
    pts = _unproject(depth)
    m = ((pts[:, 0] >= x_range[0]) & (pts[:, 0] < x_range[1]) &
         (pts[:, 2] >= z_range[0]) & (pts[:, 2] < z_range[1]))
    pts = pts[m]
    nx = int((x_range[1] - x_range[0]) / cell)
    nz = int((z_range[1] - z_range[0]) / cell)
    hist, _, _ = np.histogram2d(pts[:, 0], pts[:, 2],
                                bins=[nx, nz], range=[list(x_range), list(z_range)])
    dens = np.log1p(hist)
    if dens.max() > 0:
        dens = dens / dens.max()
    # rows = Z (forward), flip so far is up; cols = X
    grid = dens.T[::-1, :]
    img_rgb = _cmap(grid)
    img_rgb[grid == 0] = (18, 18, 22)
    img = Image.fromarray(img_rgb, "RGB").resize((nx * 2, nz * 2), Image.NEAREST)

    pad_l, pad_b, pad_t = 52, 34, 26
    canvas = Image.new("RGB", (img.width + pad_l, img.height + pad_b + pad_t), (16, 16, 16))
    canvas.paste(img, (pad_l, pad_t))
    d = ImageDraw.Draw(canvas)
    fnt = _font(13)
    d.text((pad_l, 6), "Bird's-eye occupancy (top-down)", font=fnt, fill=(230, 230, 230))
    # Z ticks (forward distance) up the left axis
    for zf in np.arange(math.ceil(z_range[0]), z_range[1] + 0.001, 1.0):
        yy = pad_t + img.height - int((zf - z_range[0]) / (z_range[1] - z_range[0]) * img.height)
        d.line([(pad_l - 5, yy), (pad_l, yy)], fill=(200, 200, 200))
        d.text((4, yy - 7), f"{zf:.0f} m", font=fnt, fill=(210, 210, 210))
    # X ticks along the bottom
    for xf in np.arange(math.ceil(x_range[0]), x_range[1] + 0.001, 1.0):
        xx = pad_l + int((xf - x_range[0]) / (x_range[1] - x_range[0]) * img.width)
        d.line([(xx, pad_t + img.height), (xx, pad_t + img.height + 5)], fill=(200, 200, 200))
        d.text((xx - 8, pad_t + img.height + 8), f"{xf:+.0f}", font=fnt, fill=(210, 210, 210))
    d.text((pad_l, pad_t + img.height + 8), "^cam", font=fnt, fill=(120, 200, 120))
    out = f"{prefix}_birdseye.png"
    canvas.save(out)
    return {"points_plotted": int(m.sum()), "grid": [nx, nz], "image": out}


# --- change / motion detection ---------------------------------------------

def set_reference(prefix: str, n_denoise: int = 5, index: int = 0) -> dict:
    pp = Path(prefix)
    pp.parent.mkdir(parents=True, exist_ok=True)
    depth = get_depth_mm_denoised(index, n=n_denoise, registered=True)
    rgb = get_rgb(index)
    np.save(f"{prefix}_ref_depth.npy", depth)
    np.save(f"{prefix}_ref_rgb.npy", rgb)
    Image.fromarray(auto_normalize_rgb(rgb)[0], "RGB").save(f"{prefix}_ref_rgb.png")
    return {"reference_saved": f"{prefix}_ref_depth.npy",
            "coverage_pct": scene_summary(depth)["coverage_pct"]}


def detect_motion(prefix: str, thresh_mm: int = 120, n_denoise: int = 5,
                  index: int = 0) -> dict:
    ref_path = f"{prefix}_ref_depth.npy"
    if not os.path.exists(ref_path):
        raise FileNotFoundError(f"no reference; run: python kinect.py ref {prefix}")
    ref = np.load(ref_path).astype(np.float32)
    cur = get_depth_mm_denoised(index, n=n_denoise, registered=True).astype(np.float32)
    rgb_norm = auto_normalize_rgb(get_rgb(index))[0]

    both = (ref > 0) & (cur > 0)
    diff = np.abs(cur - ref)
    # Depth flickers at object boundaries; exclude high-gradient (edge) pixels
    # of the reference so they don't masquerade as motion.
    grad = np.hypot(ndimage.sobel(ref, axis=1), ndimage.sobel(ref, axis=0))
    edge = ndimage.binary_dilation(grad > 400, iterations=3)
    changed = both & (diff > thresh_mm) & (~edge)
    changed = ndimage.binary_opening(changed, iterations=2)
    # keep only sizeable coherent regions (drop residual speckle)
    lbl, n = ndimage.label(changed)
    min_size = 500
    if n:
        sizes = ndimage.sum(np.ones(1), lbl, range(1, n + 1))
        keep = {i + 1 for i, s in enumerate(sizes) if s >= min_size}
        changed = np.isin(lbl, list(keep)) if keep else np.zeros_like(changed)

    overlay = rgb_norm.copy()
    overlay[changed] = (overlay[changed] * 0.35 + np.array([255, 40, 40]) * 0.65).astype(np.uint8)
    Image.fromarray(overlay, "RGB").save(f"{prefix}_motion.png")

    stats = {"changed_pct": round(100 * changed.sum() / changed.size, 2),
             "threshold_mm": thresh_mm, "image": f"{prefix}_motion.png"}
    if changed.any():
        cd = cur[changed & (cur > 0)]
        stats["nearest_change_m"] = round(float(cd.min()) / 1000, 3) if cd.size else None
        ys, xs = np.nonzero(changed)
        stats["bbox_xyxy"] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        lbl2, n2 = ndimage.label(changed)
        stats["change_regions"] = int(n2)
    else:
        stats["nearest_change_m"] = None
        stats["change_regions"] = 0
    with open(f"{prefix}_motion.json", "w") as fh:
        json.dump(stats, fh, indent=2)
    return stats


# --- audio (mic array) ------------------------------------------------------

def _probe_open(i: int, sr: int, ch: int) -> bool:
    """Return True if a tiny real recording actually opens on this device."""
    import sounddevice as sd
    try:
        sd.rec(int(0.15 * sr), samplerate=sr, channels=ch, dtype="float32", device=i)
        sd.wait()
        return True
    except Exception:
        return False


def _pick_input(substr: str):
    """Find an input device whose name contains `substr` that genuinely opens.
    Returns (index, samplerate, channels, name, hostapi) or None. WDM-KS first."""
    import sounddevice as sd
    hostapis = sd.query_hostapis()
    prio = {"Windows WDM-KS": 0, "Windows WASAPI": 1, "Windows DirectSound": 2, "MME": 3}
    rows = []
    for i, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] < 1:
            continue
        if substr and substr.lower() not in dev["name"].lower():
            continue
        api = hostapis[dev["hostapi"]]["name"]
        rows.append((prio.get(api, 9), -min(4, dev["max_input_channels"]), i, dev, api))
    rows.sort(key=lambda r: (r[0], r[1]))
    for _, _, i, dev, api in rows:
        sr = int(dev["default_samplerate"])
        for ch in sorted({min(4, dev["max_input_channels"]), 2, 1}, reverse=True):
            if ch >= 1 and _probe_open(i, sr, ch):
                return i, sr, ch, dev["name"], api
    return None


def record_audio(prefix: str, seconds: float = 5.0, device: str = None) -> dict:
    """Record an input device to WAV; render waveform + spectrogram PNGs.

    device: substring of the device name (e.g. 'Astro', 'Realtek'). Defaults to
    the Kinect mic array — which on a Kinect v1 only streams after its audio DSP
    firmware is uploaded (a separate step from the camera), so it may be
    unavailable and raise a clear error pointing you at a working mic."""
    import soundfile as sf
    from scipy import signal

    pp = Path(prefix)
    pp.parent.mkdir(parents=True, exist_ok=True)
    target = device if device else "kinect"
    pick = _pick_input(target)
    if pick is None:
        if not device:
            raise RuntimeError(
                "The Kinect v1 mic array won't open: its audio DSP firmware must be "
                "uploaded before the microphones stream (Microsoft's Kinect Runtime "
                "normally does this; our libusbK camera rebind does not). Other mics "
                "work — pass a device substring, e.g.: "
                "python kinect.py audio out\\voice 5 Astro")
        raise RuntimeError(f"no openable input device matching '{device}'")
    import sounddevice as sd
    dev_idx, sr, ch, name, api = pick
    frames = int(seconds * sr)
    audio = sd.rec(frames, samplerate=sr, channels=ch, dtype="float32", device=dev_idx)
    sd.wait()

    wav = f"{prefix}.wav"
    sf.write(wav, audio, sr, subtype="PCM_16")
    mono = audio.mean(axis=1)

    rms = float(np.sqrt(np.mean(mono ** 2)) + 1e-12)
    peak = float(np.max(np.abs(mono)) + 1e-12)
    stats = {
        "device": name, "hostapi": api, "seconds": seconds, "samplerate": sr, "channels": ch,
        "rms_dbfs": round(20 * math.log10(rms), 1),
        "peak_dbfs": round(20 * math.log10(peak), 1),
        "wav": wav,
    }

    # waveform PNG
    wf_w, wf_h = 900, 160
    wf = Image.new("RGB", (wf_w, wf_h), (16, 16, 16))
    dr = ImageDraw.Draw(wf)
    step = max(1, len(mono) // wf_w)
    env = np.abs(mono[:step * wf_w]).reshape(wf_w, step).max(axis=1)
    env = env / (env.max() + 1e-9)
    for x in range(wf_w):
        h = int(env[x] * (wf_h // 2 - 4))
        dr.line([(x, wf_h // 2 - h), (x, wf_h // 2 + h)], fill=(90, 200, 120))
    dr.text((4, 2), f"waveform  rms {stats['rms_dbfs']} dBFS  peak {stats['peak_dbfs']} dBFS",
            font=_font(13), fill=(220, 220, 220))
    wf.save(f"{prefix}_waveform.png")

    # spectrogram PNG (log-power)
    f, t, Sxx = signal.spectrogram(mono, fs=sr, nperseg=512, noverlap=384)
    S = 10 * np.log10(Sxx + 1e-10)
    S = np.clip((S - S.min()) / (S.max() - S.min() + 1e-9), 0, 1)
    spec_rgb = _cmap(1.0 - S[::-1, :])          # low freq at bottom, warm = loud
    spec = Image.fromarray(spec_rgb, "RGB").resize((900, 260), Image.BILINEAR)
    canvas = Image.new("RGB", (960, 300), (16, 16, 16))
    canvas.paste(spec, (52, 8))
    dr = ImageDraw.Draw(canvas)
    fnt = _font(12)
    dr.text((52, 284), f"spectrogram  0..{seconds:.0f}s   (device: {name})",
            font=fnt, fill=(220, 220, 220))
    for frac in (0.0, 0.5, 1.0):
        fy = 8 + int((1 - frac) * 260)
        khz = frac * (sr / 2) / 1000
        dr.text((4, fy - 7), f"{khz:.0f}k", font=fnt, fill=(210, 210, 210))
    canvas.save(f"{prefix}_spectrogram.png")

    stats["waveform"] = f"{prefix}_waveform.png"
    stats["spectrogram"] = f"{prefix}_spectrogram.png"
    with open(f"{prefix}_audio.json", "w") as fh:
        json.dump(stats, fh, indent=2)
    return stats


# --- CLI --------------------------------------------------------------------

def _print(d):
    print(json.dumps(d, indent=2))


def _main(argv):
    if not argv or argv[0] == "info":
        lib = _lib()
        print(f"loaded: {_DIST / 'libfreenect_sync.dll'}")
        for n in ("freenect_sync_get_video", "freenect_sync_get_depth",
                  "freenect_sync_set_tilt_degs", "freenect_sync_get_tilt_state"):
            print(f"  export {n}: {'ok' if hasattr(lib, n) else 'MISSING'}")
        return 0
    cmd = argv[0]
    try:
        if cmd == "tilt":
            target = float(argv[1]) if len(argv) > 1 else 0.0
            before = get_tilt()
            angle = set_tilt(target)
            print(f"before: {before['angle_deg']:+.1f} deg  accel={before['accel']}")
            for _ in range(10):
                time.sleep(0.7)
                st = get_tilt()
                if st["status"] != "moving":
                    break
            print(f"after:  {st['angle_deg']:+.1f} deg  accel={st['accel']}  status={st['status']}")
            return 0
        if cmd == "capture":
            prefix = argv[1] if len(argv) > 1 else "shot"
            rgb, depth = get_rgb(), get_depth()
            Image.fromarray(rgb, "RGB").save(f"{prefix}_rgb.png")
            dc, _, _ = colorize_depth_mm((depth.astype(np.uint32) *
                       (depth < _NO_DISP)).astype(np.uint16))  # rough
            Image.fromarray(dc, "RGB").save(f"{prefix}_depth.png")
            print(f"saved {prefix}_rgb.png, {prefix}_depth.png")
            return 0
        if cmd == "look":
            _print(look(argv[1] if len(argv) > 1 else "look"))
            return 0
        if cmd == "birdseye":
            _print(birdseye(argv[1] if len(argv) > 1 else "scene"))
            return 0
        if cmd == "ref":
            _print(set_reference(argv[1] if len(argv) > 1 else "scene"))
            return 0
        if cmd == "motion":
            _print(detect_motion(argv[1] if len(argv) > 1 else "scene"))
            return 0
        if cmd == "audio":
            prefix = argv[1] if len(argv) > 1 else "audio"
            secs = float(argv[2]) if len(argv) > 2 else 5.0
            dev = argv[3] if len(argv) > 3 else None
            _print(record_audio(prefix, secs, dev))
            return 0
    finally:
        if cmd in ("tilt", "capture", "look", "birdseye", "ref", "motion"):
            stop()
    print(__doc__)
    return 1


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
