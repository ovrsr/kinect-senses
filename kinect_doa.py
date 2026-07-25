"""
kinect_doa.py — direction-of-arrival (azimuth) from the Kinect 4-mic array.

Uses SRP-PHAT: for each candidate azimuth, sum the GCC-PHAT (phase-transform
cross-correlation) of all 6 mic pairs evaluated at the inter-mic delays that
azimuth implies. The peak azimuth is the estimated sound direction.

The 4 mics are (approximately) a horizontal line, so we resolve azimuth only
(left..right), with front/back ambiguity (assume the source is in front of the
sensor). Sign convention (calibrated 2026-07-25 against a known source):
    +deg = your RIGHT, -deg = your LEFT (as you face the sensor), 0 = ahead.

CLI:
    python kinect_doa.py listen out/doa 5     # capture 5s, estimate DOA over time
    python kinect_doa.py calib 4              # capture 4s, print instantaneous DOA
"""

from __future__ import annotations
import json
import sys
from pathlib import Path

import numpy as np
from scipy import signal
from PIL import Image, ImageDraw

import kinect            # _cmap, _font
import kinect_audio      # capture(), RATE

C_SOUND = 343.0          # m/s


def label_azimuth(az):
    """Human-readable direction for an azimuth in degrees (+ = right, - = left)."""
    if az is None:
        return "unknown"
    if abs(az) <= 8:
        return "straight ahead"
    side = "right" if az > 0 else "left"
    return f"{abs(az):.0f} deg to your {side}"
# Approximate Kinect v1 mic x-positions (m) along the array, channels 0..3.
# Geometry is approximate; sign of the result may need calibration (see `calib`).
MIC_X = np.array([0.113, -0.036, -0.076, -0.113], dtype=np.float64)
PAIRS = [(i, j) for i in range(4) for j in range(i + 1, 4)]
ANGLES = np.arange(-90, 91, 2, dtype=np.float64)


def _gcc_phat(a, b, fs, interp=16):
    """Upsampled GCC-PHAT cross-correlation. Returns (cc, max_shift, interp)."""
    n = len(a) + len(b)
    N = 1 << int(np.ceil(np.log2(max(n, 2))))
    A = np.fft.rfft(a, N)
    B = np.fft.rfft(b, N)
    R = A * np.conj(B)
    R /= np.abs(R) + 1e-9
    cc = np.fft.irfft(R, N * interp)
    max_shift = N * interp // 2
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
    return cc, max_shift, interp


def srp_over_angles(frame, fs, interp=16):
    """SRP-PHAT response across ANGLES for one multichannel frame (samples, 4)."""
    ccs = {}
    for (i, j) in PAIRS:
        ccs[(i, j)] = _gcc_phat(frame[:, i], frame[:, j], fs, interp)
    srp = np.zeros(len(ANGLES))
    for a_idx, th in enumerate(ANGLES):
        s = 0.0
        sin_t = np.sin(np.radians(th))
        for (i, j) in PAIRS:
            cc, max_shift, itp = ccs[(i, j)]
            tau = (MIC_X[i] - MIC_X[j]) * sin_t / C_SOUND      # seconds
            k = int(round(tau * fs * itp)) + max_shift
            if 0 <= k < len(cc):
                s += cc[k]
        srp[a_idx] = s
    return srp


def estimate(audio, fs, frame_s=0.4, hop_s=0.15, active_db=-55.0):
    """Per-frame SRP map + overall azimuth. Returns dict."""
    x = audio.astype(np.float64) / 2**31
    # high-pass ~150 Hz to suppress rumble/HVAC that biases the estimate
    sos = signal.butter(4, 150, btype="high", fs=fs, output="sos")
    x = signal.sosfilt(sos, x, axis=0)

    fl = int(frame_s * fs)
    hop = int(hop_s * fs)
    frames = []
    peaks = []
    energies = []
    for start in range(0, max(1, len(x) - fl), hop):
        fr = x[start:start + fl]
        if fr.shape[0] < fl:
            break
        rms = np.sqrt(np.mean(fr ** 2) + 1e-18)
        srp = srp_over_angles(fr, fs)
        frames.append(srp)
        peaks.append(float(ANGLES[int(np.argmax(srp))]))
        energies.append(20 * np.log10(rms + 1e-12))

    frames = np.array(frames) if frames else np.zeros((1, len(ANGLES)))
    energies = np.array(energies) if energies else np.array([-120.0])
    peaks = np.array(peaks) if len(peaks) else np.array([0.0])

    active = energies > active_db
    if active.any():
        # weight the overall SRP by frame energy, over active frames only
        w = np.clip(energies[active] - active_db, 0, None)
        overall_srp = (frames[active] * w[:, None]).sum(axis=0)
        overall = float(ANGLES[int(np.argmax(overall_srp))])
        conf = float(overall_srp.max() / (overall_srp.mean() + 1e-9))
    else:
        overall_srp = frames.mean(axis=0)
        overall = None
        conf = 0.0

    return {
        "overall_azimuth_deg": overall,
        "confidence": round(conf, 2),
        "active_frames": int(active.sum()),
        "total_frames": int(len(peaks)),
        "per_frame_deg": [round(p, 1) for p in peaks],
        "per_frame_db": [round(e, 1) for e in energies],
        "_frames": frames, "_energies": energies, "_peaks": peaks,
        "_overall_srp": overall_srp, "fs": fs,
    }


def _render(prefix, res, seconds):
    fnt = kinect._font(13)
    ang = ANGLES
    frames = res["_frames"]
    peaks = res["_peaks"]
    active = res["_energies"] > -55.0

    # 1) DOA-vs-time heatmap: x=time, y=azimuth, color=SRP; peak overlaid
    nT = frames.shape[0]
    norm = frames - frames.min(axis=1, keepdims=True)
    norm /= norm.max(axis=1, keepdims=True) + 1e-9
    img = kinect._cmap(1.0 - norm.T[::-1, :])    # rows=angle (top=+90); hot=strong
    im = Image.fromarray(img, "RGB").resize((max(nT * 6, 300), 300), Image.NEAREST)
    padL, padB, padT = 54, 30, 26
    cv = Image.new("RGB", (im.width + padL, im.height + padB + padT), (16, 16, 16))
    cv.paste(im, (padL, padT))
    d = ImageDraw.Draw(cv)
    d.text((padL, 6), "Direction of arrival over time", font=fnt, fill=(230, 230, 230))
    for a in (-90, -45, 0, 45, 90):
        yy = padT + int((90 - a) / 180 * im.height)
        d.line([(padL - 5, yy), (padL, yy)], fill=(200, 200, 200))
        d.text((4, yy - 7), f"{a:+d}", font=fnt, fill=(210, 210, 210))
    d.text((6, padT - 20), "az", font=fnt, fill=(150, 150, 150))
    # overlay per-frame peak (only where active) as white dots
    for t in range(nT):
        if t < len(active) and active[t]:
            yy = padT + int((90 - peaks[t]) / 180 * im.height)
            xx = padL + int((t + 0.5) / max(nT, 1) * im.width)
            d.ellipse([xx - 2, yy - 2, xx + 2, yy + 2], fill=(255, 255, 255))
    for s in range(0, int(seconds) + 1):
        xx = padL + int(s / seconds * im.width)
        d.text((xx - 4, padT + im.height + 6), f"{s}", font=fnt, fill=(200, 200, 200))
    cv.save(f"{prefix}_doa_time.png")

    # 2) overall SRP vs azimuth (fan/line) with the estimate marked
    W, H = 640, 300
    cv2 = Image.new("RGB", (W, H), (16, 16, 16))
    d2 = ImageDraw.Draw(cv2)
    srp = res["_overall_srp"].copy()
    srp -= srp.min()
    srp /= srp.max() + 1e-9
    ox, oy, rad = W // 2, H - 30, 230
    d2.text((10, 8), "Acoustic direction (overall)", font=fnt, fill=(230, 230, 230))
    prev = None
    for k, a in enumerate(ang):
        rr = 30 + srp[k] * rad
        px = ox + rr * np.sin(np.radians(a))
        py = oy - rr * np.cos(np.radians(a))
        col = tuple(int(c) for c in kinect._cmap(np.array([srp[k]]))[0])
        if prev:
            d2.line([prev, (px, py)], fill=col, width=2)
        prev = (px, py)
    for a in (-90, -45, 0, 45, 90):
        px = ox + (rad + 34) * np.sin(np.radians(a))
        py = oy - (rad + 34) * np.cos(np.radians(a))
        d2.text((px - 10, py - 6), f"{a:+d}", font=fnt, fill=(180, 180, 180))
    est = res["overall_azimuth_deg"]
    if est is not None:
        px = ox + (rad + 30) * np.sin(np.radians(est))
        py = oy - (rad + 30) * np.cos(np.radians(est))
        d2.line([(ox, oy), (px, py)], fill=(255, 90, 90), width=3)
        d2.text((10, 28), f"DOA = {est:+.0f} deg  ({label_azimuth(est)})  "
                f"conf {res['confidence']}",
                font=kinect._font(15), fill=(255, 200, 200))
    else:
        d2.text((10, 28), "no active sound (too quiet)", font=fnt, fill=(220, 180, 180))
    cv2.save(f"{prefix}_doa_polar.png")
    return f"{prefix}_doa_time.png", f"{prefix}_doa_polar.png"


def listen(prefix: str, seconds: float = 5.0) -> dict:
    Path(prefix).parent.mkdir(parents=True, exist_ok=True)
    audio, _ = kinect_audio.capture(seconds, verbose=False)
    if audio.shape[0] == 0:
        raise RuntimeError("no audio captured")
    res = estimate(audio, kinect_audio.RATE)
    t_png, p_png = _render(prefix, res, seconds)
    out = {k: v for k, v in res.items() if not k.startswith("_")}
    out["direction"] = label_azimuth(res["overall_azimuth_deg"])
    out["doa_time"] = t_png
    out["doa_polar"] = p_png
    with open(f"{prefix}_doa.json", "w") as fh:
        json.dump(out, fh, indent=2)
    return out


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "listen"
    if cmd == "listen":
        prefix = sys.argv[2] if len(sys.argv) > 2 else "out\\doa"
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
        print(json.dumps(listen(prefix, secs), indent=2))
    elif cmd == "calib":
        secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
        audio, _ = kinect_audio.capture(secs, verbose=False)
        res = estimate(audio, kinect_audio.RATE)
        az = res["overall_azimuth_deg"]
        print(f"overall azimuth: {az} deg -> {label_azimuth(az)} "
              f"(conf {res['confidence']}, {res['active_frames']} active frames)")
