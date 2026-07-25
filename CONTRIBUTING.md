# Contributing to kinect-senses

Thanks for your interest! This is a small, hardware-specific project — a Windows
toolkit for first-generation Kinect sensors built on libfreenect + libusb.
Contributions of all kinds are welcome: bug reports, fixes, new capabilities,
docs, and testing on hardware/OS combinations the author can't reach.

## Ground rules

- Be respectful; see [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
- By contributing, you agree your contributions are licensed under the project's
  [Apache-2.0](LICENSE) license.
- Keep changes focused. One logical change per pull request.

## Getting set up

```powershell
git clone https://github.com/ovrsr/kinect-senses
cd kinect-senses
./setup.ps1        # toolchain, build, deps, drivers, verify (idempotent)
./doctor.ps1       # confirm a green health check
```

See the [README](README.md) for the manual build and the USB driver details.

## Testing changes

This project talks to real hardware, so automated CI can't cover most of it.
Before opening a PR, please:

- Run `./doctor.ps1` and make sure it still passes (`-Live` exercises a real
  capture).
- Manually exercise the command(s) your change touches, e.g.
  `python kinect.py look out/g`, `python kinect_audio.py record out/m 5`,
  `python kinect_doa.py listen out/d 5`, `python kinect_fusion.py out/fix 5`.
- Note in the PR **what hardware you tested on** (Kinect model — 1414 / 1473 /
  1517 — and Windows version). Behavior differs across Kinect revisions.

## Style

- Python: match the existing style — standard library + numpy/scipy/Pillow,
  clear names, comments where the *why* isn't obvious. No heavyweight deps or a
  compiled Python extension (ctypes over the DLLs keeps it Python-version-proof).
- PowerShell: keep scripts idempotent (detect-and-skip) and ASCII-only (Windows
  PowerShell 5.1 reads `.ps1` as ANSI without a BOM).
- Don't commit captures or build artifacts — `out/`, `dist/`, `deps/`, `build/`,
  and `*.wav` are gitignored for good reason (captures may contain personal
  images/audio).

## Reporting bugs / requesting features

Open an issue using the templates. For bugs, **paste your `./doctor.ps1`
output** — it captures almost everything needed to diagnose driver/toolchain
problems.

## Areas that could use help

- Testing/calibration on the Xbox 1414 and 1473 Kinects (mic geometry, DOA sign).
- A Linux/macOS port (the Python is largely portable; the driver/`dist` bits and
  `usbprobe`/audio paths are Windows-specific).
- Continuous tracking (follow a talker over time) and depth↔DOA person picking.
- Fully unattended driver binding via libwdi (`bind-drivers.ps1` is currently
  guided + verified).
