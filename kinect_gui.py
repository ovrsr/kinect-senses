"""
kinect_gui.py — a Tkinter control panel for manually driving and observing the
Kinect v1.

    python kinect_gui.py          (or: python kinect.py gui)

Left  : live view — RGB / IR / colorized metric depth, with a pixel probe that
        reports depth in mm and the unprojected XYZ under the cursor.
Right : device controls (tilt, LED), stream settings, and one-shot actions that
        call the same functions the CLI uses (look, birdseye, ref/motion,
        point cloud, pose, depth HQ/dither, sweep, audio). Results are logged as
        JSON and the produced PNG is previewed.

Threading note: the libfreenect *sync* API keeps a single global device handle,
so every call into kinect.* happens on one dedicated worker thread. The Tk main
thread only ever touches queues.
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import kinect

_ROOT = Path(__file__).resolve().parent
_OUT = _ROOT / "out"

# stream modes: which frames the worker grabs each tick. RGB and IR come from
# different cameras, and IR shares the sensor with depth, so IR is exclusive.
MODES = ["RGB + Depth", "RGB only", "Depth only", "IR only"]

LEDS = [("off", "off"), ("green", "green"), ("red", "red"), ("orange", "orange"),
        ("blink", "blink-green"), ("alarm", "blink-orange-red")]

# kinect.colorize_depth_mm interpolates the ramp per pixel (~37 ms/frame), which
# is far too slow to sit in a live loop. For the on-screen view we precompute the
# same ramp as a 256-entry LUT and index into it (~2 ms). Saved outputs still go
# through the exact-precision function.
_LUT = kinect._cmap(np.linspace(0.0, 1.0, 256))


def colorize_fast(depth_mm: np.ndarray, dmin: float, dmax: float) -> np.ndarray:
    span = max(float(dmax) - float(dmin), 1.0)
    t = (depth_mm.astype(np.float32) - float(dmin)) * (255.0 / span)
    idx = np.clip(t, 0, 255).astype(np.uint8)
    out = _LUT[idx]
    out[depth_mm == 0] = 0
    return out


# --- device worker ----------------------------------------------------------

class Worker(threading.Thread):
    """Owns the Kinect. Consumes commands, publishes frames/logs."""

    def __init__(self, cmds: queue.Queue, events: queue.Queue):
        super().__init__(daemon=True)
        self.cmds = cmds
        self.events = events
        self.stop_flag = threading.Event()
        self.mode = "RGB + Depth"
        self.streaming = True
        self.denoise = 1          # frames medianed for the live depth view
        self.cfg = {"normalize": False, "autorange": True,
                    "dmin": 500.0, "dmax": 4000.0}
        # one-slot latest frame: the UI renders the newest frame and drops any
        # it could not keep up with, instead of working through a backlog
        self._slot = None
        self._slot_lock = threading.Lock()
        self._stat_tick = 0

    def take_frame(self):
        with self._slot_lock:
            f, self._slot = self._slot, None
        return f

    # -- helpers
    def emit(self, kind, payload):
        self.events.put((kind, payload))

    def log(self, msg):
        self.emit("log", msg)

    def run(self):
        try:
            kinect._lib()
            self.log("libfreenect_sync loaded")
        except Exception as exc:
            self.emit("error", f"cannot load libfreenect: {exc}")
            return
        next_tilt = 0.0
        while not self.stop_flag.is_set():
            try:
                self._drain_commands()
            except Exception:
                self.emit("error", traceback.format_exc(limit=3))
            now = time.time()
            if now >= next_tilt:
                next_tilt = now + 1.0
                try:
                    self.emit("tilt", kinect.get_tilt())
                except Exception as exc:
                    self.emit("status", f"tilt read failed: {exc}")
            if self.streaming:
                try:
                    self._grab()
                except Exception as exc:
                    self.emit("status", f"capture failed: {exc}")
                    time.sleep(0.5)
            else:
                time.sleep(0.05)
        try:
            kinect.stop()
        except Exception:
            pass

    def _drain_commands(self):
        while True:
            try:
                cmd = self.cmds.get_nowait()
            except queue.Empty:
                return
            self._handle(cmd)

    def _grab(self):
        """Grab and fully prepare a frame for display. All pixel work happens
        here so the Tk main thread only has to blit."""
        t0 = time.time()
        out = {}
        video = depth = None
        if self.mode == "IR only":
            ir = kinect.get_ir()
            video = np.repeat(ir[:, :, None], 3, axis=2)
            out["video_label"] = "IR"
        elif self.mode in ("RGB + Depth", "RGB only"):
            video = kinect.get_rgb()
            out["video_label"] = "RGB"
        if self.mode in ("RGB + Depth", "Depth only"):
            if self.denoise > 1:
                depth = kinect.get_depth_mm_denoised(n=self.denoise, registered=True)
            else:
                depth = kinect.get_depth_mm(registered=True)
        grab_s = time.time() - t0

        if video is not None and self.cfg["normalize"]:
            video = kinect.auto_normalize_rgb(video)[0]
        out["video"] = video

        if depth is not None:
            valid = depth > 0
            if self.cfg["autorange"] and valid.any():
                v = depth[valid]
                # subsample for the percentiles: full-frame percentiles cost more
                # than the colorize itself and the range barely differs
                s = v[::17]
                dmin, dmax = float(np.percentile(s, 2)), float(np.percentile(s, 98))
            else:
                dmin, dmax = self.cfg["dmin"], self.cfg["dmax"]
            out["depth"] = depth
            out["depth_rgb"] = colorize_fast(depth, dmin, dmax)
            out["range"] = (dmin, dmax)
            self._stat_tick += 1
            if self._stat_tick % 5 == 0 and valid.any():
                v = depth[valid]
                out["stats"] = (100.0 * valid.sum() / depth.size,
                                float(v.min()) / 1000.0,
                                float(np.median(v[::7])) / 1000.0)
        out["grab_ms"] = grab_s * 1000.0
        out["prep_ms"] = (time.time() - t0) * 1000.0 - out["grab_ms"]
        with self._slot_lock:
            self._slot = out

    # -- commands
    def _handle(self, cmd):
        op = cmd["op"]
        if op == "mode":
            self.mode = cmd["value"]
            kinect.stop()               # force the stream to re-open in the new format
            self.log(f"mode -> {self.mode}")
        elif op == "stream":
            self.streaming = bool(cmd["value"])
            self.log("streaming " + ("on" if self.streaming else "paused"))
        elif op == "denoise":
            self.denoise = int(cmd["value"])
        elif op == "cfg":
            self.cfg.update(cmd["value"])
        elif op == "tilt":
            deg = kinect.set_tilt(cmd["value"])
            self.log(f"tilt -> {deg:+d} deg")
        elif op == "led":
            kinect.set_led(cmd["value"])
            self.log(f"LED -> {cmd['value']}")
        elif op == "action":
            self._action(cmd["name"], cmd.get("prefix"), cmd.get("args", {}))
        else:
            self.log(f"unknown command {op!r}")

    def _action(self, name, prefix, args):
        was = self.streaming
        self.streaming = False
        self.emit("status", f"running {name} ...")
        t0 = time.time()
        try:
            result, images = self._run_action(name, prefix, args)
            result = result if isinstance(result, dict) else {"ok": True}
            result["_seconds"] = round(time.time() - t0, 2)
            self.emit("result", {"name": name, "data": result, "images": images})
        except Exception as exc:
            self.emit("error", f"{name} failed: {exc}\n{traceback.format_exc(limit=4)}")
        finally:
            self.emit("status", "ready")
            self.streaming = was

    def _run_action(self, name, prefix, args):
        """Return (result_dict, [image paths]). Runs on the worker thread."""
        _OUT.mkdir(parents=True, exist_ok=True)
        if name == "look":
            r = kinect.look(prefix)
            return r, [r.get("_montage")]
        if name == "birdseye":
            r = kinect.birdseye(prefix)
            return r, [r.get("image")]
        if name == "ref":
            r = kinect.set_reference(prefix)
            return r, [f"{prefix}_ref_rgb.png"]
        if name == "motion":
            r = kinect.detect_motion(prefix, thresh_mm=int(args.get("thresh_mm", 120)))
            return r, [r.get("image")]
        if name == "depthhq":
            depth = kinect.get_depth_mm_hq(n=5)
            img, dmin, dmax = kinect.colorize_depth_mm(depth)
            path = f"{prefix}_depthhq.png"
            Image.fromarray(img, "RGB").save(path)
            r = kinect.scene_summary(depth)
            r.update({"range_mm": [dmin, dmax], "image": path})
            return r, [path]
        if name == "dither":
            depth = kinect.get_depth_mm_dither()
            img, dmin, dmax = kinect.colorize_depth_mm(depth)
            path = f"{prefix}_dither.png"
            Image.fromarray(img, "RGB").save(path)
            r = kinect.scene_summary(depth)
            r.update({"range_mm": [dmin, dmax], "image": path})
            return r, [path]
        if name == "sweep":
            dpano, rpano, vfov = kinect.sweep_fov()
            cd = kinect.colorize_depth_mm(dpano)[0]
            dpath, rpath = f"{prefix}_sweep_depth.png", f"{prefix}_sweep_rgb.png"
            Image.fromarray(cd, "RGB").save(dpath)
            Image.fromarray(rpano, "RGB").save(rpath)
            return ({"vertical_fov_deg": vfov,
                     "panorama": [int(rpano.shape[1]), int(rpano.shape[0])],
                     "coverage_pct": round(100 * float((dpano > 0).sum()) / dpano.size, 1)},
                    [dpath, rpath])
        if name == "pointcloud":
            import kinect_pointcloud as kpc
            pts, cols = kpc.capture()
            if pts.shape[0] == 0:
                raise RuntimeError("no depth points in range")
            n = kpc.save_ply(f"{prefix}.ply", pts, cols)
            v1, v2 = f"{prefix}_view1.png", f"{prefix}_view2.png"
            kpc.render(pts, cols, yaw=35, pitch=12).save(v1)
            kpc.render(pts, cols, yaw=-35, pitch=8).save(v2)
            r = kpc._stats(pts)
            r["ply"] = f"{prefix}.ply ({n:,} points)"
            return r, [v1, v2]
        if name == "pose":
            import kinect_pose as kp
            img, summary = kp.detect()
            path = f"{prefix}_pose.png"
            img.save(path)
            with open(f"{prefix}_pose.json", "w") as fh:
                json.dump(summary, fh, indent=2)
            compact = {k: v for k, v in summary.items() if k != "all_joints"}
            return compact, [path]
        if name == "audio":
            r = kinect.record_audio(prefix, float(args.get("seconds", 5.0)),
                                    args.get("device") or None)
            return r, [r.get("waveform"), r.get("spectrogram")]
        if name == "doa":
            import kinect_doa
            r = kinect_doa.listen(prefix, float(args.get("seconds", 5.0)))
            return r, [v for k, v in r.items()
                       if isinstance(v, str) and v.endswith(".png")]
        raise ValueError(f"unknown action {name!r}")


# --- GUI --------------------------------------------------------------------

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Kinect v1 — manual control")
        root.configure(bg="#101014")

        self.cmds: queue.Queue = queue.Queue()
        self.events: queue.Queue = queue.Queue()
        self.worker = Worker(self.cmds, self.events)

        self.depth_mm = None            # latest depth frame, for the pixel probe
        self.fps_t0 = time.time()
        self.fps_n = 0
        self._photo = {}                # keep PhotoImage refs alive
        self.preview_paths = []
        self.preview_i = 0

        self._build()
        # typing in the range spinboxes doesn't fire their command, so watch the vars
        for var in (self.dmin_var, self.dmax_var):
            var.trace_add("write", lambda *_: self._send_cfg())
        self.worker.start()
        self.root.after(30, self._pump)
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    # -- layout
    def _build(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background="#101014", foreground="#dcdce4")
        style.configure("TLabelframe", background="#101014", foreground="#9fb0c0")
        style.configure("TLabelframe.Label", background="#101014", foreground="#9fb0c0")
        style.configure("TButton", padding=3)
        style.configure("TCheckbutton", background="#101014", foreground="#dcdce4")

        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        # ---- left: live views
        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nw")

        views = ttk.Frame(left)
        views.pack(anchor="nw")
        self.video_canvas = tk.Canvas(views, width=640, height=480, bg="#000000",
                                      highlightthickness=1, highlightbackground="#303040")
        self.video_canvas.grid(row=1, column=0, padx=(0, 6))
        self.depth_canvas = tk.Canvas(views, width=640, height=480, bg="#000000",
                                      highlightthickness=1, highlightbackground="#303040")
        self.depth_canvas.grid(row=1, column=1)
        self.video_title = ttk.Label(views, text="RGB")
        self.video_title.grid(row=0, column=0, sticky="w")
        ttk.Label(views, text="Depth (near=red, far=blue)").grid(row=0, column=1, sticky="w")

        self.probe = ttk.Label(left, text="hover the depth view to probe a pixel",
                               foreground="#87d7a0")
        self.probe.pack(anchor="w", pady=(4, 0))
        self.depth_canvas.bind("<Motion>", self._on_probe)
        self.depth_canvas.bind("<Leave>",
                               lambda e: self.probe.config(text="—"))

        self.scene = ttk.Label(left, text="", foreground="#b9c4d0")
        self.scene.pack(anchor="w")

        # ---- right: controls
        right = ttk.Frame(main, padding=(10, 0, 0, 0))
        right.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # stream
        gs = ttk.Labelframe(right, text="stream", padding=6)
        gs.pack(fill="x")
        self.mode_var = tk.StringVar(value=MODES[0])
        mode = ttk.Combobox(gs, values=MODES, textvariable=self.mode_var,
                            state="readonly", width=14)
        mode.grid(row=0, column=0, columnspan=2, sticky="w")
        mode.bind("<<ComboboxSelected>>",
                  lambda e: self.send(op="mode", value=self.mode_var.get()))
        self.live_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(gs, text="live", variable=self.live_var,
                        command=lambda: self.send(op="stream", value=self.live_var.get())
                        ).grid(row=0, column=2, padx=(8, 0))

        ttk.Label(gs, text="median frames").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.denoise_var = tk.IntVar(value=1)
        ttk.Spinbox(gs, from_=1, to=9, width=4, textvariable=self.denoise_var,
                    command=lambda: self.send(op="denoise", value=self.denoise_var.get())
                    ).grid(row=1, column=1, sticky="w", pady=(4, 0))

        self.autorange_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(gs, text="auto depth range", variable=self.autorange_var,
                        command=self._send_cfg
                        ).grid(row=2, column=0, columnspan=2, sticky="w")
        rng = ttk.Frame(gs)
        rng.grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Label(rng, text="near mm").pack(side="left")
        self.dmin_var = tk.IntVar(value=500)
        ttk.Spinbox(rng, from_=0, to=8000, increment=100, width=6,
                    textvariable=self.dmin_var, command=self._send_cfg
                    ).pack(side="left", padx=(2, 8))
        ttk.Label(rng, text="far mm").pack(side="left")
        self.dmax_var = tk.IntVar(value=4000)
        ttk.Spinbox(rng, from_=100, to=10000, increment=100, width=6,
                    textvariable=self.dmax_var, command=self._send_cfg
                    ).pack(side="left", padx=2)

        self.norm_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(gs, text="auto-normalize RGB (low light)",
                        variable=self.norm_var, command=self._send_cfg
                        ).grid(row=4, column=0, columnspan=3, sticky="w")

        # motor + LED
        gd = ttk.Labelframe(right, text="device", padding=6)
        gd.pack(fill="x", pady=(8, 0))
        self.tilt_label = ttk.Label(gd, text="tilt —   accel —")
        self.tilt_label.grid(row=0, column=0, columnspan=3, sticky="w")
        self.tilt_var = tk.DoubleVar(value=0.0)
        scale = tk.Scale(gd, from_=kinect.TILT_MIN, to=kinect.TILT_MAX,
                         orient="horizontal", variable=self.tilt_var, length=220,
                         resolution=1, bg="#101014", fg="#dcdce4",
                         troughcolor="#25252e", highlightthickness=0)
        scale.grid(row=1, column=0, columnspan=3, sticky="w")
        # apply on release, not on every drag step, so the motor gets one target
        scale.bind("<ButtonRelease-1>",
                   lambda e: self.send(op="tilt", value=float(self.tilt_var.get())))
        ttk.Button(gd, text="level (0)", command=lambda: self._set_tilt(0)
                   ).grid(row=2, column=0, sticky="w")
        ttk.Button(gd, text="up +5", command=lambda: self._nudge(+5)
                   ).grid(row=2, column=1, sticky="w")
        ttk.Button(gd, text="down -5", command=lambda: self._nudge(-5)
                   ).grid(row=2, column=2, sticky="w")

        led = ttk.Frame(gd)
        led.grid(row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Label(led, text="LED").pack(side="left", padx=(0, 4))
        for label, value in LEDS:
            ttk.Button(led, text=label, width=6,
                       command=lambda v=value: self.send(op="led", value=v)
                       ).pack(side="left", padx=1)

        # actions
        ga = ttk.Labelframe(right, text="actions", padding=6)
        ga.pack(fill="x", pady=(8, 0))
        pf = ttk.Frame(ga)
        pf.pack(fill="x")
        ttk.Label(pf, text="prefix").pack(side="left")
        self.prefix_var = tk.StringVar(value=str(_OUT / "gui"))
        ttk.Entry(pf, textvariable=self.prefix_var, width=28).pack(side="left", padx=4)

        grid = ttk.Frame(ga)
        grid.pack(fill="x", pady=(4, 0))
        buttons = [
            ("look", "look"), ("bird's-eye", "birdseye"),
            ("set reference", "ref"), ("detect motion", "motion"),
            ("depth HQ", "depthhq"), ("depth dither", "dither"),
            ("tilt sweep", "sweep"), ("point cloud", "pointcloud"),
            ("pose", "pose"), ("audio 5s", "audio"),
            ("sound DOA", "doa"), ("save frames", "_save"),
        ]
        for i, (label, name) in enumerate(buttons):
            ttk.Button(grid, text=label, width=14,
                       command=lambda n=name: self._action(n)
                       ).grid(row=i // 2, column=i % 2, padx=2, pady=2, sticky="we")

        opt = ttk.Frame(ga)
        opt.pack(fill="x", pady=(4, 0))
        ttk.Label(opt, text="motion thresh mm").pack(side="left")
        self.thresh_var = tk.IntVar(value=120)
        ttk.Spinbox(opt, from_=20, to=1000, increment=10, width=6,
                    textvariable=self.thresh_var).pack(side="left", padx=(2, 10))
        ttk.Label(opt, text="mic").pack(side="left")
        self.mic_var = tk.StringVar(value="")
        ttk.Entry(opt, textvariable=self.mic_var, width=10).pack(side="left", padx=2)

        # preview + log
        gp = ttk.Labelframe(right, text="last result", padding=6)
        gp.pack(fill="both", expand=True, pady=(8, 0))
        self.preview = tk.Canvas(gp, width=440, height=200, bg="#000000",
                                 highlightthickness=1, highlightbackground="#303040")
        self.preview.pack()
        prow = ttk.Frame(gp)
        prow.pack(fill="x", pady=2)
        ttk.Button(prow, text="< prev", width=7,
                   command=lambda: self._cycle(-1)).pack(side="left")
        ttk.Button(prow, text="next >", width=7,
                   command=lambda: self._cycle(+1)).pack(side="left", padx=2)
        ttk.Button(prow, text="open file", command=self._open_preview).pack(side="left")
        self.preview_label = ttk.Label(prow, text="")
        self.preview_label.pack(side="left", padx=6)

        self.log = tk.Text(gp, height=12, width=54, bg="#16161c", fg="#c8d0dc",
                           insertbackground="#c8d0dc", relief="flat", wrap="none")
        self.log.pack(fill="both", expand=True, pady=(4, 0))

        self.status = ttk.Label(self.root, text="starting ...", anchor="w",
                                foreground="#87d7a0")
        self.status.pack(fill="x", padx=8, pady=(0, 4))

    # -- helpers
    def send(self, **cmd):
        self.cmds.put(cmd)

    def _send_cfg(self):
        """Push display settings to the worker, which does all the pixel work."""
        try:
            dmin, dmax = float(self.dmin_var.get()), float(self.dmax_var.get())
        except (tk.TclError, ValueError):
            return                            # mid-edit spinbox text
        self.send(op="cfg", value={"normalize": bool(self.norm_var.get()),
                                   "autorange": bool(self.autorange_var.get()),
                                   "dmin": dmin, "dmax": max(dmax, dmin + 1)})

    def _set_tilt(self, deg):
        self.tilt_var.set(deg)
        self.send(op="tilt", value=float(deg))

    def _nudge(self, delta):
        self._set_tilt(max(kinect.TILT_MIN,
                           min(kinect.TILT_MAX, self.tilt_var.get() + delta)))

    def _action(self, name):
        prefix = self.prefix_var.get().strip() or str(_OUT / "gui")
        Path(prefix).parent.mkdir(parents=True, exist_ok=True)
        if name == "_save":
            self._save_frames(prefix)
            return
        args = {"thresh_mm": self.thresh_var.get(),
                "seconds": 5.0, "device": self.mic_var.get().strip()}
        self.status.config(text=f"queued {name} ...")
        self.send(op="action", name=name, prefix=prefix, args=args)

    def _save_frames(self, prefix):
        """Save exactly what is on screen right now (main thread, no device I/O)."""
        saved = []
        if getattr(self, "_last_video", None) is not None:
            p = f"{prefix}_live_video.png"
            Image.fromarray(self._last_video, "RGB").save(p)
            saved.append(p)
        if getattr(self, "_last_depth_rgb", None) is not None:
            p = f"{prefix}_live_depth.png"
            Image.fromarray(self._last_depth_rgb, "RGB").save(p)
            saved.append(p)
        if self.depth_mm is not None:
            p = f"{prefix}_live_depth_mm.npy"
            np.save(p, self.depth_mm)
            saved.append(p)
        if saved:
            self._log("saved:\n  " + "\n  ".join(saved))
            self._set_preview([s for s in saved if s.endswith(".png")])
        else:
            self._log("nothing to save yet")

    def _log(self, text):
        self.log.insert("end", text.rstrip() + "\n")
        self.log.see("end")

    # -- rendering
    def _show(self, canvas, key, arr):
        img = ImageTk.PhotoImage(Image.fromarray(arr, "RGB"))
        self._photo[key] = img
        canvas.delete("img")
        canvas.create_image(0, 0, anchor="nw", image=img, tags="img")

    def _on_probe(self, event):
        d = self.depth_mm
        if d is None:
            return
        x, y = int(event.x), int(event.y)
        if not (0 <= x < d.shape[1] and 0 <= y < d.shape[0]):
            return
        mm = int(d[y, x])
        if mm == 0:
            self.probe.config(text=f"({x},{y})  no data")
            return
        z = mm / 1000.0
        X = (x - kinect._CX) * z / kinect._FX
        Y = (y - kinect._CY) * z / kinect._FY
        self.probe.config(
            text=f"({x},{y})  {mm} mm = {z:.3f} m   XYZ = "
                 f"{X:+.2f}, {Y:+.2f}, {z:.2f} m  (X right, Y down, Z fwd)")

    def _set_preview(self, paths):
        self.preview_paths = [p for p in (paths or []) if p and os.path.exists(p)]
        self.preview_i = 0
        self._draw_preview()

    def _cycle(self, delta):
        if self.preview_paths:
            self.preview_i = (self.preview_i + delta) % len(self.preview_paths)
            self._draw_preview()

    def _draw_preview(self):
        self.preview.delete("all")
        if not self.preview_paths:
            self.preview_label.config(text="")
            return
        path = self.preview_paths[self.preview_i]
        try:
            im = Image.open(path)
        except Exception as exc:
            self.preview_label.config(text=f"cannot open: {exc}")
            return
        im.thumbnail((440, 200))
        photo = ImageTk.PhotoImage(im)
        self._photo["preview"] = photo
        self.preview.create_image(220, 100, image=photo)
        self.preview_label.config(
            text=f"{self.preview_i + 1}/{len(self.preview_paths)}  {Path(path).name}")

    def _open_preview(self):
        if not self.preview_paths:
            return
        path = os.path.abspath(self.preview_paths[self.preview_i])
        try:
            if sys.platform == "win32":
                os.startfile(path)          # noqa: S606 - user-initiated
            else:
                subprocess.Popen(["xdg-open", path])
        except Exception as exc:
            self._log(f"open failed: {exc}")

    # -- event pump
    def _pump(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                self._on_event(kind, payload)
        except queue.Empty:
            pass
        frame = self.worker.take_frame()      # newest only; stale frames dropped
        if frame is not None:
            self._on_frame(frame)
        self.root.after(15, self._pump)

    def _on_event(self, kind, payload):
        if kind == "tilt":
            self.tilt_label.config(
                text=f"tilt {payload['angle_deg']:+.1f} deg   "
                     f"accel {payload['accel']}   {payload['status']}")
        elif kind == "status":
            self.status.config(text=payload)
        elif kind == "log":
            self._log(payload)
        elif kind == "error":
            self.status.config(text=payload.splitlines()[0])
            self._log("ERROR: " + payload)
        elif kind == "result":
            self._log(f"--- {payload['name']} ---\n"
                      + json.dumps(payload["data"], indent=2, default=str))
            self._set_preview(payload.get("images"))

    def _on_frame(self, f):
        video = f.get("video")
        if video is not None:
            self._last_video = video
            self.video_title.config(text=f.get("video_label", "video"))
            self._show(self.video_canvas, "video", video)
        else:
            self._last_video = None
            self.video_canvas.delete("img")

        depth_rgb = f.get("depth_rgb")
        if depth_rgb is not None:
            self.depth_mm = f["depth"]
            self._last_depth_rgb = depth_rgb
            self._show(self.depth_canvas, "depth", depth_rgb)
            if "stats" in f:
                cov, near, med = f["stats"]
                dmin, dmax = f["range"]
                self.scene.config(
                    text=f"coverage {cov:.1f}%   nearest {near:.2f} m   "
                         f"median {med:.2f} m   "
                         f"scale {dmin/1000:.2f}–{dmax/1000:.2f} m")
        else:
            self.depth_mm = None
            self._last_depth_rgb = None
            self.depth_canvas.delete("img")

        self.fps_n += 1
        now = time.time()
        if now - self.fps_t0 >= 1.0:
            fps = self.fps_n / (now - self.fps_t0)
            self.fps_t0, self.fps_n = now, 0
            cur = self.status.cget("text")
            if cur in ("ready", "starting ...") or cur.startswith("live"):
                msg = (f"live  {fps:.1f} fps   grab {f.get('grab_ms', 0):.0f} ms   "
                       f"prep {f.get('prep_ms', 0):.0f} ms")
                if self.mode_var.get() == "RGB + Depth" and fps < 18:
                    msg += "   — dual-stream USB loss; try a single-stream mode"
                self.status.config(text=msg)

    def _quit(self):
        self.status.config(text="closing ...")
        self.worker.stop_flag.set()
        self.worker.join(timeout=3.0)
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
