# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project aims
to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Live GUI** (`kinect_gui.py` / `python kinect.py gui`): a Tkinter control
  panel for driving and observing the sensor by hand. Side-by-side live RGB (or
  IR) and colorized metric depth, with a depth pixel probe that reports mm and
  the unprojected XYZ under the cursor; tilt slider + LED buttons with a live
  angle/accelerometer readout; selectable stream mode, temporal-median depth,
  fixed or auto depth colour range, and low-light RGB normalization; one-click
  buttons for every capability (look, birdseye, ref/motion, depth HQ/dither,
  sweep, point cloud, pose, audio, DOA) with JSON results and image previews,
  plus a "save frames" button that writes exactly what is on screen. All
  libfreenect calls run on one dedicated worker thread — the sync API keeps a
  single global device handle — while Tk only touches queues.
- **`streambench.py`**: measures per-stream USB health (achieved fps and
  libfreenect's dropped-packet/resync rate) so USB ports and host controllers
  can be compared objectively.

### Fixed

- Live-view rendering no longer stalls the UI thread: depth colorization for the
  on-screen view now uses a 256-entry LUT (~6 ms/frame vs ~37 ms for the
  per-pixel `colorize_depth_mm`, max 4/255 channel difference), all pixel work
  moved off the Tk main thread, and the frame pump renders only the newest frame
  instead of working through a backlog. Saved outputs still use the exact
  full-precision colorizer.

## [0.2.0] - 2026-07-25

### Added

- **Tilt-sweep FOV panorama** (`kinect.sweep_fov` / `python kinect.py sweep`):
  sweeps the motor across its range and stitches depth + RGB into a taller
  vertical field of view (~95° vs ~49° single-frame), each frame offset by its
  measured tilt (`fy·Δθ`) and overlaps averaged.
- **Motor-dither depth** (`kinect.get_depth_mm_dither` / `python kinect.py
  depthdither`): captures depth at several small tilt offsets to decorrelate the
  fixed laser speckle, aligns each frame by its induced vertical shift, and
  merges — recovering *real* depth (not interpolation) in speckle/edge gaps and
  denoising the result. Honest, modest gain (~+3% real coverage on a test scene)
  because the motor is pitch-only; opt-in and slow (it moves the motor).
- **Edge-aware depth hole-fill** (`kinect.holefill_guided` / `get_depth_mm_hq`):
  a joint-bilateral fill that closes structured-light gaps using RGB colour
  similarity as guidance — filling *along* surfaces but not *across* object
  edges (crisper boundaries, ~+6% coverage vs the old isotropic fill). `look`
  now uses it, and `python kinect.py depthhq` renders a raw/simple/guided
  comparison.
- **Colored 3D point cloud** (`kinect_pointcloud.py` + `kinect_pointcloud` MCP
  tool): unprojects the registered depth + RGB into metric 3D points, exports a
  binary PLY (MeshLab / CloudCompare / Blender), and renders the cloud from a
  rotated viewpoint so the 3D structure is viewable as an image.
- **3D body skeleton** (`kinect_pose.py` + `kinect_pose` MCP tool): MediaPipe
  BlazePose on the RGB frame, with each of the 33 joints lifted to metric 3D by
  sampling the registered depth — an annotated skeleton image + per-joint `xyz`
  and torso distance. Optional dep in `requirements-pose.txt`; the pose model
  auto-downloads on first run.
- Status **LED** control (`python kinect.py led <state>` / `kinect.set_led`):
  off / green / red / orange / blink-green / blink-orange-red — a hardware
  status notifier for agents.
- **MCP bridge** (`mcp_server.py`): exposes the Kinect to Model Context Protocol
  clients (Claude Desktop, Claude Code, …) with tools `kinect_look`,
  `kinect_hear`, `kinect_locate_sound`, `kinect_set_led`, `kinect_tilt`, and
  `kinect_health`. Tool results return PNG images + JSON; hardware access is
  serialized behind a lock. Optional dep in `requirements-mcp.txt`.

## [0.1.0] - 2026-07-25

Initial release: a Windows toolkit turning a first-generation Kinect into a
multi-modal sensor via libfreenect + libusb (no Microsoft Kinect Runtime; works
with Memory Integrity / HVCI enabled).

### Added

- **Vision** (`kinect.py`): RGB, IR, and metric (RGB-registered, millimetre)
  depth over `libfreenect_sync` via ctypes; `look` composite with a scene
  summary; top-down `birdseye` occupancy map; reference/`motion` change
  detection; motor `tilt`; multi-frame depth denoise + hole-fill.
- **Mic array** (`kinect_audio.py`): direct 4-channel / 32-bit / 16 kHz capture
  of the Kinect mic array over libusb isochronous transfers (hand-rolled async
  iso via ctypes), with WAV + waveform + spectrogram output. No `audios.bin`
  firmware or security handshake required on the Kinect for Windows.
- **Direction of arrival** (`kinect_doa.py`): SRP-PHAT azimuth estimation with
  a DOA-over-time heatmap and polar plot; calibrated left/right convention.
- **Fusion** (`kinect_fusion.py`): map DOA azimuth into the depth frame for a
  3D fix (distance + `xyz`) on a sound source, with an annotated image.
- **USB tooling** (`usbprobe.py`): libusb descriptor-tree dumper.
- **Setup automation**: idempotent `setup.ps1`, guided/verified
  `bind-drivers.ps1`, and a `doctor.ps1` health check.
- Docs: README (scripted quick start + manual build + driver setup + caveats),
  CONTRIBUTING, CODE_OF_CONDUCT, SECURITY, NOTICE, and issue/PR templates.

### Known limitations

- Windows x64 only for the driver/`dist` build; the Python is largely portable.
- Direction of arrival is azimuth-only (single horizontal mic row) with a
  front/back ambiguity (assumes the source is in front of the sensor).
- Driver binding is guided (a few Zadig clicks), not fully unattended.

[Unreleased]: https://github.com/ovrsr/kinect-senses/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/ovrsr/kinect-senses/releases/tag/v0.2.0
[0.1.0]: https://github.com/ovrsr/kinect-senses/releases/tag/v0.1.0
