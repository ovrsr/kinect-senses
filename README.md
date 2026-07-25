# kinect-senses

Turn a first-generation Kinect (Kinect for Windows 1517, or the Xbox 1414/1473)
into a multi-modal sensor on Windows 11 — RGB, IR, metric depth, spatial maps,
motion, the 4-mic array, sound direction, and audio↔depth fusion — built
entirely on open-source **libfreenect** + **libusb**.

No Microsoft Kinect Runtime is required, and **Windows Memory Integrity (HVCI)
can stay enabled**. The stock Microsoft camera driver fails under HVCI with
*Code 39 / "Bad Image"* (it's a 2013 driver blocked by hypervisor-enforced code
integrity); this project sidesteps it by talking to the sensor through libusb.

Every capability writes **PNG images and JSON** — designed so an LLM/agent (or a
human) can consume the sensor by reading files.

## Capabilities

| Command | Output |
|---|---|
| `python kinect.py look out/g` | Auto-normalized RGB (IR fallback in the dark), denoised **metric** depth heatmap with a meters colorbar, a composite montage, and a JSON scene summary (nearest/median/farthest, L/C/R occupancy, foreground object count) |
| `python kinect.py birdseye out/s` | Top-down occupancy floor-plan with metric axes |
| `python kinect.py ref out/s` / `python kinect.py motion out/s` | Reference + edge-robust depth change/motion detection |
| `python kinect.py tilt 0` | Level/aim the motor (−30..30°) |
| `python kinect.py led green` | Set the status LED (off/green/red/orange/blink-green/blink-orange-red) |
| `python kinect_audio.py record out/m 5` | 4-channel mic-array WAV + waveform + spectrogram + per-channel levels |
| `python kinect_doa.py listen out/d 5` | Sound **direction of arrival** (azimuth) + DOA-over-time heatmap + polar plot |
| `python kinect_fusion.py out/fix 5` | **3D fix on a sound source** — azimuth mapped into the depth frame, distance read out, annotated image + `xyz` |
| `python kinect_pose.py out/p` | **3D body skeleton** — MediaPipe pose lifted to metric 3D via the depth (annotated image + per-joint `xyz`). Needs `pip install -r requirements-pose.txt` |

`python usbprobe.py` dumps the Kinect USB descriptor tree (handy for debugging).

## Quick start (scripted)

On Windows x64 with [scoop](https://scoop.sh), git, and Python 3.10+ installed:

```powershell
git clone https://github.com/ovrsr/kinect-senses
cd kinect-senses
./setup.ps1
```

`setup.ps1` is **idempotent** (each step detects whether it's already done):
it installs the toolchain, fetches libusb, builds libfreenect, assembles
`dist/`, installs the Python deps, walks you through the USB driver binding, and
verifies the result. Helpers:

- `./doctor.ps1` — health check (toolchain, DLLs, deps, driver bindings). Add
  `-Live` to also grab a real frame + mic read.
- `./bind-drivers.ps1` — (re)bind just the USB drivers, guided and verified;
  `-Check` reports current bindings without changing anything.

The sections below explain what those scripts do, for a manual install or a
non-scoop machine.

## Requirements

- Windows 10/11 x64 with a first-gen Kinect (+ its USB/power adapter for the
  Kinect for Windows / Xbox sensor).
- **Python 3.10+** (developed on 3.14) and `pip install -r requirements.txt`.
- A build toolchain to compile libfreenect: **CMake, Ninja, and MinGW-w64 gcc**.
  The easiest route on Windows is [scoop](https://scoop.sh):
  ```
  scoop install cmake ninja mingw 7zip
  ```
- **[Zadig](https://zadig.akeo.ie/)** to bind the Kinect USB interfaces to
  libusb-compatible drivers.

## Manual build (what `setup.ps1` automates)

1. **Clone libfreenect and apply the one-line Windows/gcc fix** (gcc 14+ errors
   on the implicit `sleep()` in `usb_libusb10.c`):
   ```
   git clone --depth 1 https://github.com/OpenKinect/libfreenect.git
   git -C libfreenect apply patches/libfreenect-mingw-sleep.patch
   ```

2. **Get libusb** (MinGW prebuilt) — download `libusb-1.0.x.7z` from
   [libusb releases](https://github.com/libusb/libusb/releases) and extract it,
   e.g. to `deps/libusb`. You need `include/libusb.h`, the import lib
   `MinGW64/static/libusb-1.0.dll.a`, and `MinGW64/dll/libusb-1.0.dll`.

3. **Configure + build** (adjust the `<...>` paths to your libusb and mingw):
   ```
   cmake -G Ninja -S libfreenect -B build \
     -DCMAKE_BUILD_TYPE=Release \
     -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF -DBUILD_CPP=OFF \
     -DBUILD_C_SYNC=ON -DBUILD_REDIST_PACKAGE=ON \
     -DBUILD_PYTHON=OFF -DBUILD_PYTHON2=OFF -DBUILD_PYTHON3=OFF \
     -DLIBUSB_1_INCLUDE_DIRS="<deps>/libusb/include" \
     -DLIBUSB_1_LIBRARIES="<deps>/libusb/MinGW64/static/libusb-1.0.dll.a" \
     -DTHREADS_PTHREADS_INCLUDE_DIR="<mingw>/x86_64-w64-mingw32/include" \
     -DTHREADS_PTHREADS_WIN32_LIBRARY="<mingw>/x86_64-w64-mingw32/lib/libpthread.a"
   cmake --build build -j
   ```

4. **Assemble `dist/`** next to the Python scripts with the three runtime DLLs
   (everything else resolves to the inbox Windows UCRT):
   ```
   dist/libfreenect.dll
   dist/libfreenect_sync.dll
   dist/libusb-1.0.dll        # from deps/libusb/MinGW64/dll
   ```

## USB driver setup (Zadig)

Run Zadig **as admin**, enable *Options → List All Devices*, and replace drivers:

| Device (USB ID) | Driver | Needed for |
|---|---|---|
| Kinect camera `045E:02AE` | **libusbK** | depth + RGB + IR (isochronous streams) |
| Kinect motor `045E:02B0` | WinUSB (or libusbK) | tilt |
| Kinect audio **composite parent** `045E:02BB` | **libusbK** | the 4-mic array |

For the mic array you must put the driver on the **composite *parent***, not the
individual interfaces — enable *Options → uncheck "Ignore Hubs or Composite
Parents"* and select the `045E:02BB` "USB Composite Device". With `usbccgp` still
in charge, libusb can't reach the audio streaming interface (interface 3 / EP
`0x82`). See `docs`-in-code comments in `kinect_audio.py` for the gory details.

> Rebinding the camera/audio removes the Windows Kinect drivers for those
> functions (which don't work under HVCI / are gated anyway). Fully reversible:
> Device Manager → *Uninstall device* → unplug/replug.

## Python usage

```
pip install -r requirements.txt
python kinect.py look out/glance
python kinect_audio.py record out/mic 5
python kinect_doa.py listen out/doa 5
python kinect_fusion.py out/fix 5
```

## Agent access (MCP)

`mcp_server.py` exposes the Kinect to any
[Model Context Protocol](https://modelcontextprotocol.io) client (Claude
Desktop, Claude Code, …) so an agent can *see* through it and drive its status
LED / motor. Tool results return PNG images (which vision models consume) plus
JSON.

```powershell
pip install -r requirements-mcp.txt
```

Register with Claude Code:

```
claude mcp add kinect -- python C:/path/to/kinect-senses/mcp_server.py
```

…or in Claude Desktop's `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "kinect": { "command": "python", "args": ["C:/path/to/kinect-senses/mcp_server.py"] }
  }
}
```

Tools: `kinect_look`, `kinect_pose`, `kinect_hear`, `kinect_locate_sound`,
`kinect_set_led`, `kinect_tilt`, `kinect_health`. (`kinect_pose` needs
`requirements-pose.txt`; the pose model auto-downloads on first use.)

- The server runs where the hardware is (local **stdio** transport). Only one
  process can own the Kinect over USB, so every call is serialized behind a lock.
- **Privacy:** `kinect_hear` and `kinect_locate_sound` record the microphone.
  MCP clients prompt before each tool call — treat those as sensitive, and use
  the LED (`kinect_set_led "blink-green"`) as a visible "sensor active" cue.

## Notes & caveats

- **Mic privacy:** Windows may block app microphone access machine-wide. The
  direct-libusb mic path (`kinect_audio.py`) is unaffected, but the fallback
  `kinect.py audio <device>` path needs *Settings → Privacy → Microphone* on.
- **Kinect audio is plain USB Audio Class** (4ch / 32-bit / 16 kHz iso on EP
  `0x82`) — **no `audios.bin` firmware and no security handshake are required**
  on the Kinect for Windows; the "Security Control" interface does not gate it.
- **Direction of arrival** is azimuth-only (a single horizontal mic row) with a
  front/back ambiguity — it assumes the source is in front of the sensor. Sign
  convention: **+° = to your right, −° = to your left**, facing the sensor.
- **Fusion** maps DOA azimuth to a depth column with `AZ_SIGN = -1` (the camera
  faces you, so your right is image-left). Mic and lens are a few cm apart on
  the bar — negligible parallax beyond ~1 m.
- Depth colorization, occupancy, and 3D use the Kinect RGB intrinsics
  `fx=fy=525, cx=319.5, cy=239.5` (registered depth is aligned to the 640×480
  RGB frame).

## Layout

```
setup.ps1         # idempotent installer/builder/verifier
bind-drivers.ps1  # guided + verified USB driver binding (Zadig)
doctor.ps1        # health check (-Live for a real capture)
kinect.py         # RGB/IR/metric-depth, look, birdseye, motion, tilt (libfreenect_sync via ctypes)
kinect_audio.py   # 4-mic array capture over libusb (async iso, hand-rolled ctypes)
kinect_doa.py     # SRP-PHAT direction of arrival
kinect_fusion.py  # DOA x depth -> 3D sound-source fix
kinect_pose.py    # 3D body skeleton (MediaPipe pose lifted via depth)
mcp_server.py     # Model Context Protocol bridge (agent access)
usbprobe.py       # libusb USB descriptor dumper
patches/          # the libfreenect Windows/gcc fix
```

## Contributing

Issues and pull requests are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md)
and the [Code of Conduct](CODE_OF_CONDUCT.md). Report security issues privately
per [SECURITY.md](SECURITY.md). Notable changes are tracked in
[CHANGELOG.md](CHANGELOG.md).

## Credits & license

Built on [libfreenect](https://github.com/OpenKinect/libfreenect) (Apache-2.0 /
GPL-2.0) and [libusb](https://github.com/libusb/libusb) (LGPL-2.1). Those remain
under their own licenses; this repository's own code is licensed under
**Apache-2.0** (see `LICENSE`). Not affiliated with or endorsed by Microsoft or
the OpenKinect project.
