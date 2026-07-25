# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project aims
to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/ovrsr/kinect-senses/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/ovrsr/kinect-senses/releases/tag/v0.1.0
