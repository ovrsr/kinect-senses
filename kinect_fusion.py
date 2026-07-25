"""
kinect_fusion.py — fuse mic-array direction with depth for a 3D fix on a sound.

Direction-of-arrival (kinect_doa) gives the horizontal azimuth of a sound; the
depth camera gives distance per pixel. We map the azimuth to a depth-image
column, read the nearest object's distance there, and report where the sound
source is in 3D — plus an annotated image with a crosshair on the source.

Frame alignment: the mic array and the RGB/depth camera both look forward but
(a) have opposite left/right handedness (the camera faces you, so your right is
image-left) and (b) sit a few cm apart on the bar (parallax, negligible >1 m).
AZ_SIGN captures the handedness and is calibrated live, like the DOA sign was.

CLI:
    python kinect_fusion.py locate out/fix 5
"""

from __future__ import annotations
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import kinect
import kinect_audio
import kinect_doa

# camera intrinsics (registered depth is aligned to the 640x480 RGB frame)
FX, CX = kinect._FX, kinect._CX
FY, CY = kinect._FY, kinect._CY
W, H = kinect._W, kinect._H

# +1 => camera column increases with DOA azimuth; -1 => inverted (mirror).
# The camera faces the user, so the user's right (DOA +deg) is image-left,
# which means AZ_SIGN = -1. Verified live 2026-07-25.
AZ_SIGN = -1


def azimuth_to_column(az_deg: float) -> int:
    """Map a DOA azimuth (+deg = user's right) to a depth-image column u."""
    u = CX + AZ_SIGN * FX * np.tan(np.radians(az_deg))
    return int(np.clip(round(u), 0, W - 1))


def _distance_in_column(depth_mm, u, half_w=10):
    """Nearest coherent object distance (mm) + its row within column u +/- half_w."""
    lo, hi = max(0, u - half_w), min(W, u + half_w + 1)
    strip = depth_mm[:, lo:hi]
    valid = strip[strip > 0]
    if valid.size < 30:
        return None, None
    znear = float(np.percentile(valid, 15))          # robust "nearest surface"
    band = (strip > 0) & (np.abs(strip.astype(np.float32) - znear) < 200)
    rows = np.nonzero(band.any(axis=1))[0]
    v = int(np.median(rows)) if rows.size else H // 2
    # distance = median depth of the near band (steadier than the percentile)
    z = float(np.median(strip[band])) if band.any() else znear
    return z, v


def locate(prefix: str, seconds: float = 5.0) -> dict:
    Path(prefix).parent.mkdir(parents=True, exist_ok=True)

    # 1) sound direction
    audio, _ = kinect_audio.capture(seconds, verbose=False)
    if audio.shape[0] == 0:
        raise RuntimeError("no audio captured")
    doa = kinect_doa.estimate(audio, kinect_audio.RATE)
    az = doa["overall_azimuth_deg"]

    # 2) depth + rgb snapshot (source assumed ~static over the few-second window)
    depth = kinect.get_depth_mm_denoised(n=5, registered=True)
    rgb_norm, _ = kinect.auto_normalize_rgb(kinect.get_rgb())
    kinect.stop()

    out = {"azimuth_deg": az, "direction": kinect_doa.label_azimuth(az),
           "doa_confidence": doa["confidence"]}

    img = Image.fromarray(rgb_norm.copy(), "RGB")
    dr = ImageDraw.Draw(img)
    fnt = kinect._font(15)

    if az is None:
        dr.text((10, 10), "no active sound (too quiet to localize)",
                font=fnt, fill=(255, 200, 200))
        img.save(f"{prefix}_fix.png")
        out["located"] = False
    else:
        u = azimuth_to_column(az)
        z_mm, v = _distance_in_column(depth, u)
        out["column_u"] = u
        if z_mm is None:
            out["located"] = False
            out["note"] = "no depth in that column (source out of frame or too near/far)"
            dr.line([(u, 0), (u, H)], fill=(255, 90, 90), width=2)
            dr.text((10, 10), f"{out['direction']}  (no depth in column {u})",
                    font=fnt, fill=(255, 200, 200))
        else:
            z = z_mm / 1000.0
            x = (u - CX) * z / FX
            y = (v - CY) * z / FY
            out.update({"located": True, "distance_m": round(z, 2),
                        "xyz_m": [round(x, 2), round(y, 2), round(z, 2)],
                        "row_v": v})
            # draw azimuth line + crosshair on the source
            dr.line([(u, 0), (u, H)], fill=(255, 150, 60), width=1)
            r = 16
            dr.ellipse([u - r, v - r, u + r, v + r], outline=(255, 60, 60), width=3)
            dr.line([(u - r - 6, v), (u + r + 6, v)], fill=(255, 60, 60), width=2)
            dr.line([(u, v - r - 6), (u, v + r + 6)], fill=(255, 60, 60), width=2)
            label = f"sound source: {z:.2f} m, {out['direction']}"
            dr.rectangle([6, 6, 6 + 9 * len(label), 28], fill=(0, 0, 0))
            dr.text((10, 8), label, font=fnt, fill=(255, 230, 120))
        img.save(f"{prefix}_fix.png")

    out["image"] = f"{prefix}_fix.png"
    with open(f"{prefix}_fix.json", "w") as fh:
        json.dump(out, fh, indent=2)
    return out


if __name__ == "__main__":
    prefix = sys.argv[1] if len(sys.argv) > 1 else "out/fix"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    print(json.dumps(locate(prefix, secs), indent=2))
