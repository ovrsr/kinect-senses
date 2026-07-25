r"""streambench.py — measure Kinect USB stream health.

    python streambench.py            # rgb, depth, and both
    python streambench.py both 15    # one mode, 15 seconds

Reports achieved frames/sec and counts libfreenect's packet complaints, so you
can compare USB ports/controllers objectively. A healthy single stream is ~29
fps with zero resyncs; anything with a nonzero resync rate is losing isochronous
packets and will look stuttery in the GUI.

Note the Kinect v1 needs ~22 MB/s to run depth and video at once, which is most
of a USB 2.0 bus — dual-stream health depends heavily on which host controller
the device lands on.
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
MODES = ("rgb", "depth", "both")


def _run_one(mode: str, seconds: float) -> int:
    sys.path.insert(0, str(_ROOT))
    import kinect

    t0 = time.time()
    n = 0
    while time.time() - t0 < seconds:
        if mode == "rgb":
            kinect.get_rgb()
        elif mode == "depth":
            kinect.get_depth_mm(registered=True)
        else:
            kinect.get_rgb()
            kinect.get_depth_mm(registered=True)
        n += 1
    dt = time.time() - t0
    kinect.stop()
    print(f"__RESULT__ {n / dt:.1f}")
    return 0


def main(argv):
    # libfreenect writes its packet errors straight to the console from C, so
    # each mode runs in a child process whose output we can capture and count.
    if argv and argv[0] == "--child":
        return _run_one(argv[1], float(argv[2]))

    modes = [argv[0]] if argv and argv[0] in MODES else list(MODES)
    seconds = float(argv[1]) if len(argv) > 1 else 8.0

    print(f"{'mode':<8}{'fps':>7}{'resyncs':>10}{'bad pkts':>10}   verdict")
    for i, mode in enumerate(modes):
        if i:
            time.sleep(2.0)
        proc = subprocess.run(
            [sys.executable, __file__, "--child", mode, str(seconds)],
            capture_output=True, text=True, cwd=str(_ROOT))
        out = proc.stdout + proc.stderr
        fps = 0.0
        for line in out.splitlines():
            if line.startswith("__RESULT__"):
                fps = float(line.split()[1])
        resyncs = out.count("resyncing")
        bad = out.count("Invalid magic") + out.count("Expected")
        if resyncs == 0 and fps > 25:
            verdict = "clean"
        elif resyncs / max(seconds, 1) < 1:
            verdict = "minor loss"
        else:
            verdict = "LOSING PACKETS"
        print(f"{mode:<8}{fps:>7.1f}{resyncs:>10}{bad:>10}   {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
