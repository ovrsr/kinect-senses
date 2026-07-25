"""
mcp_server.py — Model Context Protocol bridge for kinect-senses.

Exposes the Kinect as MCP tools so any MCP client (Claude Desktop, Claude Code,
etc.) can see through it and drive its status LED / motor directly. Tool results
return PNG images (which vision models see) alongside JSON/text.

Run (stdio transport):
    python mcp_server.py

Register with Claude Code:
    claude mcp add kinect -- python C:/path/to/kinect-senses/mcp_server.py

Only one process can own the Kinect over USB, so every hardware call is
serialized behind a lock. Nothing here prints to stdout (it would corrupt the
stdio protocol) — diagnostics go to stderr.

Requires:  pip install mcp   (plus the kinect-senses runtime: see README)
"""

from __future__ import annotations

import io
import sys
import threading

import numpy as np
from PIL import Image as PILImage

from mcp.server.fastmcp import FastMCP, Image

import kinect
import kinect_audio
import kinect_doa
import kinect_fusion

mcp = FastMCP("kinect-senses")

# The Kinect can only be claimed by one caller at a time; serialize everything.
_LOCK = threading.Lock()


def _png(arr_or_pil) -> Image:
    """Encode a numpy RGB array or PIL image as an MCP PNG image block."""
    img = arr_or_pil if isinstance(arr_or_pil, PILImage.Image) else PILImage.fromarray(arr_or_pil)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return Image(data=buf.getvalue(), format="png")


def _err(where, e) -> str:
    return (f"{where} failed: {e}\n"
            "Is the Kinect connected and its drivers bound? "
            "Run doctor.ps1 / bind-drivers.ps1.")


@mcp.tool(description="Look through the Kinect: returns an annotated RGB+depth "
          "montage image (IR fallback in the dark) and a scene summary with "
          "nearest/median distances (metres), left/center/right occupancy, and "
          "foreground object count.")
def kinect_look() -> list:
    with _LOCK:
        try:
            rgb = kinect.get_rgb()
            rgb_norm, gamma = kinect.auto_normalize_rgb(rgb)
            dark = rgb.astype(np.float32).mean() < 35
            left = np.stack([kinect.get_ir()] * 3, -1) if dark else rgb_norm
            depth = kinect.get_depth_mm_denoised(n=5, registered=True)
            dcolor, dmin, dmax = kinect.colorize_depth_mm(depth)
            summary = kinect.scene_summary(depth)
            summary["low_light"] = bool(dark)
            montage = _compose(left, dcolor, dmin, dmax,
                               "IR (low light)" if dark else "RGB")
            return [_png(montage), _json(summary)]
        except Exception as e:
            return [_err("kinect_look", e)]
        finally:
            _safe_stop()


@mcp.tool(description="Detect a person and return a 3D skeleton: an annotated "
          "image with the pose overlaid, plus per-joint 3D coordinates (metres, "
          "camera frame) and the torso distance. RGB pose (MediaPipe) lifted to "
          "3D via the Kinect depth. Returns {person:false} if nobody is in view.")
def kinect_pose() -> list:
    with _LOCK:
        try:
            import kinect_pose as kp
            img, summary = kp.detect()
            compact = {k: v for k, v in summary.items() if k != "all_joints"}
            return [_png(img), _json(compact)]
        except Exception as e:
            return [_err("kinect_pose", e)]
        finally:
            _safe_stop()


@mcp.tool(description="Capture a colored 3D point cloud and return a rendered "
          "oblique view showing the scene's 3D structure, plus stats (point "
          "count and metric bounds). For the PLY export use kinect_pointcloud.py.")
def kinect_pointcloud() -> list:
    with _LOCK:
        try:
            import kinect_pointcloud as kpc
            pts, cols = kpc.capture()
            if pts.shape[0] == 0:
                return ["no depth points captured (device/scene?)."]
            return [_png(kpc.render(pts, cols, yaw=35, pitch=12)), _json(kpc._stats(pts))]
        except Exception as e:
            return [_err("kinect_pointcloud", e)]
        finally:
            _safe_stop()


@mcp.tool(description="Read the room's audio via the 4-mic array for `seconds` "
          "(1-15). Returns a spectrogram image + per-channel levels (dBFS). "
          "PRIVACY: this records the microphone.")
def kinect_hear(seconds: float = 5.0) -> list:
    seconds = float(max(1.0, min(15.0, seconds)))
    with _LOCK:
        try:
            audio, _ = kinect_audio.capture(seconds, verbose=False)
            if audio.shape[0] == 0:
                return ["No audio captured (mic array claimed / unbound?)."]
            spec = _spectrogram(audio)
            f32 = audio.astype(np.float64) / 2**31
            rms = np.sqrt(np.mean(f32 ** 2, axis=0))
            info = {"seconds": round(audio.shape[0] / kinect_audio.RATE, 2),
                    "channels": int(audio.shape[1]), "samplerate": kinect_audio.RATE,
                    "per_channel_dbfs": [round(20 * np.log10(r + 1e-12), 1) for r in rms]}
            return [_png(spec), _json(info)]
        except Exception as e:
            return [_err("kinect_hear", e)]


@mcp.tool(description="Locate a sound in 3D: listen for `seconds`, estimate its "
          "direction (azimuth), and read its distance from the depth frame. "
          "Returns an annotated image + {direction, distance_m, xyz}. PRIVACY: "
          "records the microphone.")
def kinect_locate_sound(seconds: float = 5.0) -> list:
    seconds = float(max(2.0, min(15.0, seconds)))
    with _LOCK:
        try:
            audio, _ = kinect_audio.capture(seconds, verbose=False)
            doa = kinect_doa.estimate(audio, kinect_audio.RATE)
            az = doa["overall_azimuth_deg"]
            depth = kinect.get_depth_mm_denoised(n=5, registered=True)
            rgb_norm = kinect.auto_normalize_rgb(kinect.get_rgb())[0]
            out = {"azimuth_deg": az, "direction": kinect_doa.label_azimuth(az),
                   "confidence": doa["confidence"]}
            if az is not None:
                u = kinect_fusion.azimuth_to_column(az)
                z_mm, v = kinect_fusion._distance_in_column(depth, u)
                if z_mm:
                    z = z_mm / 1000.0
                    out.update({"distance_m": round(z, 2),
                                "xyz_m": [round((u - kinect_fusion.CX) * z / kinect_fusion.FX, 2),
                                          round((v - kinect_fusion.CY) * z / kinect_fusion.FY, 2),
                                          round(z, 2)]})
            return [_png(rgb_norm), _json(out)]
        except Exception as e:
            return [_err("kinect_locate_sound", e)]
        finally:
            _safe_stop()


@mcp.tool(description="Set the Kinect status LED as a notifier. state is one of: "
          "off, green, red, orange, blink-green, blink-orange-red.")
def kinect_set_led(state: str = "green") -> str:
    with _LOCK:
        try:
            if state not in kinect._LED_NAMES:
                return f"unknown LED state '{state}'. Options: {', '.join(sorted(kinect._LED_NAMES))}"
            kinect.set_led(state)
            return f"LED set to {state}."
        except Exception as e:
            return _err("kinect_set_led", e)
        finally:
            _safe_stop()


@mcp.tool(description="Aim the Kinect: set the tilt motor to `degrees` (-30..30, "
          "0 = level). Returns the resulting motor state.")
def kinect_tilt(degrees: float = 0.0) -> str:
    with _LOCK:
        try:
            angle = kinect.set_tilt(degrees)
            return f"motor commanded to {angle:+d} deg (settling)."
        except Exception as e:
            return _err("kinect_tilt", e)
        finally:
            _safe_stop()


@mcp.tool(description="Report Kinect health: whether the device streams and the "
          "current tilt/accelerometer state.")
def kinect_health() -> str:
    with _LOCK:
        try:
            t = kinect.get_tilt()
            depth = kinect.get_depth_mm(registered=True)
            cov = round(100 * (depth > 0).sum() / depth.size, 1)
            return _json({"streaming": True, "depth_coverage_pct": cov,
                          "tilt_deg": t["angle_deg"], "accel": t["accel"]})
        except Exception as e:
            return _err("kinect_health", e)
        finally:
            _safe_stop()


# --- helpers ----------------------------------------------------------------

def _safe_stop():
    try:
        kinect.stop()
    except Exception:
        pass


def _json(d) -> str:
    import json
    return json.dumps(d, indent=2)


def _compose(left_rgb, depth_rgb, dmin, dmax, left_label):
    from PIL import ImageDraw
    W, H = kinect._W, kinect._H
    cbar = kinect._depth_colorbar(H, dmin, dmax)
    gap, top = 12, 26
    canvas = PILImage.new("RGB", (W + gap + W + gap + cbar.width, top + H), (16, 16, 16))
    canvas.paste(PILImage.fromarray(left_rgb.astype(np.uint8), "RGB"), (0, top))
    canvas.paste(PILImage.fromarray(depth_rgb, "RGB"), (W + gap, top))
    canvas.paste(cbar, (W + gap + W + gap, top))
    d = ImageDraw.Draw(canvas)
    d.text((0, 6), left_label, font=kinect._font(14), fill=(230, 230, 230))
    d.text((W + gap, 6), "Depth (m) near=red far=blue", font=kinect._font(14), fill=(230, 230, 230))
    return canvas


def _spectrogram(audio):
    from scipy import signal
    mono = (audio.astype(np.float64) / 2**31).mean(axis=1)
    f, t, Sxx = signal.spectrogram(mono, fs=kinect_audio.RATE, nperseg=512, noverlap=384)
    S = 10 * np.log10(Sxx + 1e-12)
    S = np.clip((S - S.min()) / (S.max() - S.min() + 1e-9), 0, 1)
    rgb = kinect._cmap(1.0 - S[::-1, :])
    return PILImage.fromarray(rgb, "RGB").resize((900, 260), PILImage.BILINEAR)


if __name__ == "__main__":
    print("kinect-senses MCP server starting (stdio)", file=sys.stderr)
    mcp.run()
