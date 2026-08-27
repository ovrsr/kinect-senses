"""
kinect_stt.py — Speech-to-text for the Kinect mic array via faster-whisper.

Takes the raw 4-channel int32 audio from kinect_audio.capture() and returns
a structured transcript. Lazy-loads the Whisper model on first call so the
MCP server starts fast.

The Kinect mic array outputs 4-ch / 32-bit / 16 kHz PCM. Whisper expects
mono 16 kHz float32, so we down-mix and normalize before transcribing.
"""

from __future__ import annotations

import io
import tempfile
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from faster_whisper import WhisperModel

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

MODEL_SIZE = "small"        # good accuracy / speed balance on CPU
DEVICE = "cpu"
COMPUTE_TYPE = "int8"       # fastest on CPU; use "float16" for CUDA
BEAM_SIZE = 5
VAD_FILTER = True           # skip silent chunks
LANGUAGE = None             # auto-detect; set "en" to force English

# ---------------------------------------------------------------------------
# Lazy model singleton
# ---------------------------------------------------------------------------

_model: WhisperModel | None = None


def _get_model() -> WhisperModel:
    """Load the Whisper model on first use (takes a few seconds)."""
    global _model
    if _model is None:
        from faster_whisper import WhisperModel
        _model = WhisperModel(
            MODEL_SIZE,
            device=DEVICE,
            compute_type=COMPUTE_TYPE,
        )
    return _model


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def transcribe_audio(audio: np.ndarray, samplerate: int = 16000) -> dict:
    """
    Transcribe raw Kinect mic-array audio.

    Parameters
    ----------
    audio : np.ndarray
        Shape (samples, channels), dtype int32 — straight from
        kinect_audio.capture().
    samplerate : int
        Sample rate (default 16000, matching the Kinect mic array).

    Returns
    -------
    dict with keys:
        transcript : str          — full text (all segments joined)
        segments   : list[dict]   — per-segment {start, end, text, avg_logprob}
        language   : str          — detected language code
        language_probability : float
        duration_s : float        — audio duration in seconds
        processing_s : float      — wall-clock STT time
    """
    if audio.ndim == 1:
        mono = audio.astype(np.float32) / 2**31
    elif audio.ndim == 2:
        # Down-mix to mono: average across channels
        mono = audio.astype(np.float32).mean(axis=1) / 2**31
    else:
        raise ValueError(f"expected 1-D or 2-D audio array, got {audio.ndim}-D")

    duration_s = len(mono) / samplerate

    # Skip transcription for very short clips (< 0.3s) — Whisper struggles
    if duration_s < 0.3:
        return {
            "transcript": "",
            "segments": [],
            "language": None,
            "language_probability": 0.0,
            "duration_s": round(duration_s, 2),
            "processing_s": 0.0,
        }

    model = _get_model()
    t0 = time.monotonic()

    segments_iter, info = model.transcribe(
        mono,
        beam_size=BEAM_SIZE,
        vad_filter=VAD_FILTER,
        language=LANGUAGE,
    )

    segments = []
    texts = []
    for seg in segments_iter:
        segments.append({
            "start": round(seg.start, 2),
            "end": round(seg.end, 2),
            "text": seg.text.strip(),
            "avg_logprob": round(seg.avg_logprob, 3),
        })
        texts.append(seg.text.strip())

    elapsed = time.monotonic() - t0
    transcript = " ".join(texts)

    return {
        "transcript": transcript,
        "segments": segments,
        "language": info.language,
        "language_probability": round(info.language_probability, 3),
        "duration_s": round(duration_s, 2),
        "processing_s": round(elapsed, 2),
    }


# ---------------------------------------------------------------------------
# CLI smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import json
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "live":
        # Live test: capture from the Kinect mic array
        import kinect_audio
        seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
        print(f"Capturing {seconds}s from Kinect mic array...")
        audio, stats = kinect_audio.capture(seconds, verbose=True)
        print(f"Captured {audio.shape[0]} frames, transcribing...")
        result = transcribe_audio(audio, kinect_audio.RATE)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        # Dry run: test with silence
        print("Dry run (silence):")
        silence = np.zeros((16000 * 3, 4), dtype=np.int32)
        result = transcribe_audio(silence, 16000)
        print(json.dumps(result, indent=2))
