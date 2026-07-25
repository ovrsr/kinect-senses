"""
kinect_audio.py — capture the Kinect for Windows mic array directly over libusb.

The K4W audio (VID 045E, PID 02BB) exposes the 4-mic array as USB Audio Class
isochronous IN on EP 0x82: 4 channels, 32-bit PCM, 16 kHz (256 bytes/packet).
This bypasses the (gated / non-working) Windows audio path.

Requires the 02BB audio interface to be bound to libusbK/WinUSB (via Zadig).

libusb has no synchronous iso API, so we drive the async transfer API by hand:
allocate N transfers of P packets each, submit them, and resubmit in the
completion callback to stream continuously while pumping libusb_handle_events.
"""

from __future__ import annotations
import ctypes as C
import os
import sys
import time
from pathlib import Path

import numpy as np

_DIST = Path(__file__).resolve().parent / "dist"
os.add_dll_directory(str(_DIST))
_lib = C.CDLL(str(_DIST / "libusb-1.0.dll"))

VID, PID = 0x045E, 0x02BB
EP_AUDIO_IN = 0x82
AUDIO_IFACE = 3            # USB Audio streaming interface
AUDIO_ALT = 1             # alt setting with the iso endpoint
PKT_SIZE = 256            # wMaxPacketSize of EP 0x82
CHANNELS = 4
SAMPLE_BYTES = 4          # 32-bit
RATE = 16000

# libusb transfer types / statuses
LIBUSB_TRANSFER_TYPE_ISOCHRONOUS = 1
_STATUS = {0: "COMPLETED", 1: "ERROR", 2: "TIMED_OUT", 3: "CANCELLED",
           4: "STALL", 5: "NO_DEVICE", 6: "OVERFLOW"}


class Timeval(C.Structure):
    _fields_ = [("tv_sec", C.c_long), ("tv_usec", C.c_long)]


class IsoPacketDescriptor(C.Structure):
    _fields_ = [("length", C.c_uint), ("actual_length", C.c_uint), ("status", C.c_int)]


class Transfer(C.Structure):
    # fixed head of struct libusb_transfer; iso_packet_desc[] follows at offset 60
    _fields_ = [
        ("dev_handle", C.c_void_p), ("flags", C.c_uint8), ("endpoint", C.c_uint8),
        ("type", C.c_uint8), ("timeout", C.c_uint), ("status", C.c_int),
        ("length", C.c_int), ("actual_length", C.c_int),
        ("callback", C.c_void_p), ("user_data", C.c_void_p),
        ("buffer", C.POINTER(C.c_uint8)), ("num_iso_packets", C.c_int)]


_ISO_OFF = 60  # offsetof(struct libusb_transfer, iso_packet_desc) on LP64/Win64
_CB = C.CFUNCTYPE(None, C.POINTER(Transfer))

# --- signatures -------------------------------------------------------------
_lib.libusb_init.argtypes = [C.POINTER(C.c_void_p)]
_lib.libusb_open_device_with_vid_pid.argtypes = [C.c_void_p, C.c_uint16, C.c_uint16]
_lib.libusb_open_device_with_vid_pid.restype = C.c_void_p
_lib.libusb_claim_interface.argtypes = [C.c_void_p, C.c_int]
_lib.libusb_release_interface.argtypes = [C.c_void_p, C.c_int]
_lib.libusb_set_interface_alt_setting.argtypes = [C.c_void_p, C.c_int, C.c_int]
_lib.libusb_alloc_transfer.argtypes = [C.c_int]
_lib.libusb_alloc_transfer.restype = C.c_void_p
_lib.libusb_submit_transfer.argtypes = [C.c_void_p]
_lib.libusb_cancel_transfer.argtypes = [C.c_void_p]
_lib.libusb_free_transfer.argtypes = [C.c_void_p]
_lib.libusb_handle_events_timeout_completed.argtypes = [C.c_void_p, C.POINTER(Timeval), C.POINTER(C.c_int)]
_lib.libusb_close.argtypes = [C.c_void_p]
_lib.libusb_exit.argtypes = [C.c_void_p]
_lib.libusb_strerror.argtypes = [C.c_int]
_lib.libusb_strerror.restype = C.c_char_p


def _err(code):
    return _lib.libusb_strerror(code).decode(errors="replace")


def _iso_desc(tp, i):
    addr = C.cast(tp, C.c_void_p).value + _ISO_OFF + i * C.sizeof(IsoPacketDescriptor)
    return IsoPacketDescriptor.from_address(addr)


def capture(seconds: float = 5.0, iface: int = AUDIO_IFACE, alt: int = AUDIO_ALT,
            n_transfers: int = 8, pkts_per_xfer: int = 32, verbose: bool = True):
    """Stream the mic array for `seconds`. Returns (samples, CHANNELS) int32 array."""
    ctx = C.c_void_p()
    if _lib.libusb_init(C.byref(ctx)) != 0:
        raise RuntimeError("libusb_init failed")

    handle = _lib.libusb_open_device_with_vid_pid(ctx, VID, PID)
    if not handle:
        _lib.libusb_exit(ctx)
        raise RuntimeError(f"could not open {VID:04X}:{PID:04X} — is 02BB on libusbK? "
                           "(run Zadig on 'Kinect USB Audio')")

    collected = []
    stats = {"packets": 0, "empty": 0, "bytes": 0, "status_counts": {}}

    def _on_done(tp):
        t = tp.contents
        st = t.status
        stats["status_counts"][_STATUS.get(st, st)] = \
            stats["status_counts"].get(_STATUS.get(st, st), 0) + 1
        for i in range(t.num_iso_packets):
            d = _iso_desc(tp, i)
            stats["packets"] += 1
            if d.status == 0 and d.actual_length > 0:
                off = i * PKT_SIZE
                stats["bytes"] += d.actual_length
                collected.append(bytes(t.buffer[off:off + d.actual_length]))
            else:
                stats["empty"] += 1
        if not _stop[0]:
            _lib.libusb_submit_transfer(tp)   # keep streaming

    cb = _CB(_on_done)
    _stop = [False]

    try:
        rc = _lib.libusb_claim_interface(handle, iface)
        if rc != 0:
            raise RuntimeError(f"claim_interface({iface}) failed: {_err(rc)}")
        rc = _lib.libusb_set_interface_alt_setting(handle, iface, alt)
        if rc != 0:
            raise RuntimeError(f"set alt {iface}/{alt} failed: {_err(rc)}")

        buffers = []
        transfers = []
        for _ in range(n_transfers):
            tp = _lib.libusb_alloc_transfer(pkts_per_xfer)
            if not tp:
                raise RuntimeError("libusb_alloc_transfer failed")
            buf = (C.c_uint8 * (pkts_per_xfer * PKT_SIZE))()
            t = Transfer.from_address(tp)
            t.dev_handle = handle
            t.endpoint = EP_AUDIO_IN
            t.type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS
            t.timeout = 1000
            t.buffer = C.cast(buf, C.POINTER(C.c_uint8))
            t.length = pkts_per_xfer * PKT_SIZE
            t.num_iso_packets = pkts_per_xfer
            t.callback = C.cast(cb, C.c_void_p)
            for i in range(pkts_per_xfer):
                _iso_desc(tp, i).length = PKT_SIZE
            buffers.append(buf)
            transfers.append(tp)

        for tp in transfers:
            rc = _lib.libusb_submit_transfer(tp)
            if rc != 0:
                raise RuntimeError(f"submit_transfer failed: {_err(rc)}")

        tv = Timeval(0, 100000)   # 100 ms
        done = C.c_int(0)
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            _lib.libusb_handle_events_timeout_completed(ctx, C.byref(tv), C.byref(done))

        # stop + drain
        _stop[0] = True
        for tp in transfers:
            _lib.libusb_cancel_transfer(tp)
        drain_end = time.monotonic() + 0.5
        while time.monotonic() < drain_end:
            _lib.libusb_handle_events_timeout_completed(ctx, C.byref(tv), C.byref(done))
        for tp in transfers:
            _lib.libusb_free_transfer(tp)

        if verbose:
            print(f"iso packets={stats['packets']} empty={stats['empty']} "
                  f"bytes={stats['bytes']} statuses={stats['status_counts']}")

        raw = b"".join(collected)
        n = len(raw) // (CHANNELS * SAMPLE_BYTES)
        if n == 0:
            return np.zeros((0, CHANNELS), np.int32), stats
        arr = np.frombuffer(raw[:n * CHANNELS * SAMPLE_BYTES],
                            dtype=np.int32).reshape(n, CHANNELS).copy()
        return arr, stats
    finally:
        _lib.libusb_release_interface(handle, iface)
        _lib.libusb_close(handle)
        _lib.libusb_exit(ctx)


def record(prefix: str, seconds: float = 5.0) -> dict:
    """Capture the mic array, save a 4-ch WAV, render waveform + spectrogram PNGs."""
    import soundfile as sf
    from scipy import signal
    from PIL import Image, ImageDraw
    import kinect  # reuse colormap + font helpers

    pp = Path(prefix)
    pp.parent.mkdir(parents=True, exist_ok=True)
    arr, cstats = capture(seconds)
    if arr.shape[0] == 0:
        raise RuntimeError("no audio captured")

    f32 = arr.astype(np.float64) / 2**31          # normalize 32-bit -> [-1,1]
    wav = f"{prefix}.wav"
    sf.write(wav, (f32).astype(np.float32), RATE, subtype="PCM_24")
    mono = f32.mean(axis=1)

    rms = np.sqrt(np.mean(f32 ** 2, axis=0))
    stats = {
        "device": "Kinect mic array (libusb iso EP 0x82)",
        "seconds": round(arr.shape[0] / RATE, 2), "samplerate": RATE,
        "channels": CHANNELS,
        "per_channel_dbfs": [round(20 * np.log10(r + 1e-12), 1) for r in rms],
        "wav": wav, "iso_packets": cstats["packets"], "iso_empty": cstats["empty"],
    }

    # waveform (mono mix)
    wf_w, wf_h = 900, 150
    wf = Image.new("RGB", (wf_w, wf_h), (16, 16, 16))
    dr = ImageDraw.Draw(wf)
    step = max(1, len(mono) // wf_w)
    env = np.abs(mono[:step * wf_w]).reshape(wf_w, step).max(axis=1)
    env = env / (env.max() + 1e-9)
    for x in range(wf_w):
        h = int(env[x] * (wf_h // 2 - 4))
        dr.line([(x, wf_h // 2 - h), (x, wf_h // 2 + h)], fill=(90, 200, 120))
    dr.text((4, 2), f"Kinect mic (mono mix)  per-ch dBFS {stats['per_channel_dbfs']}",
            font=kinect._font(13), fill=(220, 220, 220))
    wf.save(f"{prefix}_waveform.png")

    # spectrogram
    fr, tt, Sxx = signal.spectrogram(mono, fs=RATE, nperseg=512, noverlap=384)
    S = 10 * np.log10(Sxx + 1e-12)
    S = np.clip((S - S.min()) / (S.max() - S.min() + 1e-9), 0, 1)
    spec = kinect._cmap(1.0 - S[::-1, :])
    img = Image.fromarray(spec, "RGB").resize((900, 260), Image.BILINEAR)
    canvas = Image.new("RGB", (960, 300), (16, 16, 16))
    canvas.paste(img, (52, 8))
    dr = ImageDraw.Draw(canvas)
    fnt = kinect._font(12)
    dr.text((52, 284), f"Kinect mic array spectrogram  0..{stats['seconds']:.0f}s  "
            f"4ch/32-bit/16kHz via libusb", font=fnt, fill=(220, 220, 220))
    for frac in (0.0, 0.5, 1.0):
        fy = 8 + int((1 - frac) * 260)
        dr.text((4, fy - 7), f"{frac*(RATE/2)/1000:.0f}k", font=fnt, fill=(210, 210, 210))
    canvas.save(f"{prefix}_spectrogram.png")

    stats["waveform"] = f"{prefix}_waveform.png"
    stats["spectrogram"] = f"{prefix}_spectrogram.png"
    import json
    with open(f"{prefix}_audio.json", "w") as fh:
        json.dump(stats, fh, indent=2)
    return stats


if __name__ == "__main__":
    import json
    cmd = sys.argv[1] if len(sys.argv) > 1 else "record"
    if cmd == "record":
        prefix = sys.argv[2] if len(sys.argv) > 2 else "out\\kmic"
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
        print(json.dumps(record(prefix, secs), indent=2))
    else:
        secs = float(sys.argv[1])
        arr, stats = capture(secs)
        print(f"captured {arr.shape[0]} frames ({arr.shape[0]/RATE:.2f}s)")
