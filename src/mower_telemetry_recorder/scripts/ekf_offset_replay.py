#!/usr/bin/env python3
"""Offline EKF position replay to verify the antenna-offset weighting fix.

Background: on a controlled string-line drive the RAW GPS antenna shows the
expected ~2*offset separation between opposite passes, but the FUSED position
(what gets mowed) still differs by ~20 cm — the EKF applies only a fraction of
the lateral antenna offset because the GPS position update is weighted far too
weakly (fixed measurement covariance 500), so dead-reckoning (which knows no
antenna offset) dominates.

This tool replays the recorded drive through a faithful 2-D position filter and
sweeps the GPS measurement covariance R (and process noise Q) to show which
weighting makes the offset fully effective — i.e. the opposite fused lines
collapse onto each other — without letting GPS noise blow up the track. It
changes nothing in the running system; it only informs the covariance choice
for the real fix in xbot_positioning.

Model (mirrors xbot_positioning):
  state  = center position (x, y) of the rotation centre
  predict: x += cos(theta)*v*dt, y += sin(theta)*v*dt   (dead-reckoning, no offset)
  measure: antenna = center + R(theta)*offset
           => innovation = raw_gps - (x + R(theta)*offset)
  update : x += K*innovation, with K = P/(P+R), P propagated with process noise Q

theta comes from the recording (fused_theta — verified good); v is derived from
the fused track; offset is the configured antenna offset.

Usage:
  ekf_offset_replay.py <session.jsonl|.omheat.zip> --offset <x_fwd> <y_left>
      --segments "OST=45:102,WEST=169:238,NORD=310:373,SUED=400:467"
"""

import json
import math
import sys
import zipfile
from typing import Optional


def _iter_jsonl(text: str):
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except ValueError:
            continue


def load_samples(path: str) -> list:
    if path.endswith(".zip") or zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            name = next((n for n in zf.namelist() if n.endswith("samples.jsonl")), None)
            if name is None:
                raise SystemExit(f"No samples.jsonl inside {path}")
            text = zf.read(name).decode("utf-8")
    else:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    return list(_iter_jsonl(text))


def theta_of(s: dict) -> Optional[float]:
    if "ekf_theta" in s:
        return float(s["ekf_theta"])
    if "fused_theta" in s:
        return float(s["fused_theta"])
    return None


def replay(samples: list, offset: tuple, q: float, r: float) -> list:
    """Run the position filter over the samples. Returns the fused centre track
    [(x, y), ...] aligned with the input samples (None where unusable)."""
    ox, oy = offset
    n = len(samples)
    out: list = [None] * n
    # State + scalar covariance per axis (x and y are symmetric and decoupled
    # for a position random walk, so one P suffices).
    px = py = None
    P = 1.0
    last_t = None
    for i, s in enumerate(samples):
        th = theta_of(s)
        if "raw_gps_x" not in s or th is None:
            continue
        t = float(s["ts"])
        rax, ray = float(s["raw_gps_x"]), float(s["raw_gps_y"])
        # measured centre = antenna - R(theta)*offset
        cmx = rax - (math.cos(th) * ox - math.sin(th) * oy)
        cmy = ray - (math.sin(th) * ox + math.cos(th) * oy)
        if px is None:
            px, py, last_t = cmx, cmy, t
            P = r
            out[i] = (px, py)
            continue
        dt = max(1e-3, t - last_t)
        last_t = t
        # Predict: dead-reckoning from derived velocity (the part that does NOT
        # know the antenna offset). Use the fused-track delta as the motion
        # proxy; this mirrors how wheel/odometry drives the predict step.
        fx, fy = s.get("x"), s.get("y")
        pfx, pfy = samples[i - 1].get("x"), samples[i - 1].get("y")
        if None not in (fx, fy, pfx, pfy):
            px += float(fx) - float(pfx)
            py += float(fy) - float(pfy)
        P += q * dt
        # Update toward offset-corrected measurement.
        K = P / (P + r)
        px += K * (cmx - px)
        py += K * (cmy - py)
        P *= (1.0 - K)
        out[i] = (px, py)
    return out


def perp_separation(track: list, seg_a: tuple, seg_b: tuple) -> Optional[float]:
    """Perpendicular distance between two opposite line segments of a track."""
    a = [p for p in track[seg_a[0]:seg_a[1] + 1] if p is not None]
    b = [p for p in track[seg_b[0]:seg_b[1] + 1] if p is not None]
    if len(a) < 2 or len(b) < 2:
        return None
    mx = sum(p[0] for p in a) / len(a)
    my = sum(p[1] for p in a) / len(a)
    sxx = sum((p[0] - mx) ** 2 for p in a)
    syy = sum((p[1] - my) ** 2 for p in a)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in a)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    nx, ny = -math.sin(ang), math.cos(ang)
    d1 = sum((p[0] - mx) * nx + (p[1] - my) * ny for p in a) / len(a)
    d2 = sum((p[0] - mx) * nx + (p[1] - my) * ny for p in b) / len(b)
    return abs(d2 - d1)


def track_roughness(track: list, seg: tuple) -> Optional[float]:
    """RMS deviation of the track from its own straight fit over a segment —
    a proxy for how much GPS noise leaks through (smaller = smoother)."""
    pts = [p for p in track[seg[0]:seg[1] + 1] if p is not None]
    if len(pts) < 3:
        return None
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    nx, ny = -math.sin(ang), math.cos(ang)
    res = [((p[0] - mx) * nx + (p[1] - my) * ny) for p in pts]
    return math.sqrt(sum(d * d for d in res) / len(res))


def parse_segments(spec: str) -> dict:
    segs = {}
    for part in spec.split(","):
        name, rng = part.split("=")
        a, b = rng.split(":")
        segs[name.strip()] = (int(a), int(b))
    return segs


def main(argv: list) -> None:
    args = argv[1:]
    offset = (0.0, 0.0)
    segments = {}
    if "--offset" in args:
        i = args.index("--offset")
        offset = (float(args[i + 1]), float(args[i + 2]))
        del args[i:i + 3]
    if "--segments" in args:
        i = args.index("--segments")
        segments = parse_segments(args[i + 1])
        del args[i:i + 2]
    if len(args) != 1:
        raise SystemExit(__doc__)
    samples = load_samples(args[0])
    print(f"samples: {len(samples)}  offset cfg: fwd={offset[0]:+.3f} left={offset[1]:+.3f}")
    if not segments:
        raise SystemExit("Pass --segments to measure opposite-line collapse.")

    pairs = [("OST", "WEST"), ("NORD", "SUED")]
    # Baseline: the recorded fused track itself (what the live filter produced).
    print("\n== Recorded fused track (live filter, R=500) ==")
    rec = [(float(s["x"]), float(s["y"])) if "x" in s and "y" in s else None for s in samples]
    for a, b in pairs:
        if a in segments and b in segments:
            sep = perp_separation(rec, segments[a], segments[b])
            print(f"  {a}<->{b}: line separation {sep * 100:5.1f} cm")

    # Sweep GPS measurement covariance R (m^2). Process noise q fixed moderate.
    print("\n== Replay sweep: separation (should -> ~0) and roughness (noise) ==")
    print(f"  {'R (m^2)':>10} | " + " | ".join(f"{a}<->{b} sep" for a, b in pairs) + " | mean roughness")
    q = 0.01
    for r in (500.0, 50.0, 5.0, 0.5, 0.05, 0.005, 0.0005):
        track = replay(samples, offset, q, r)
        seps = []
        for a, b in pairs:
            if a in segments and b in segments:
                seps.append(perp_separation(track, segments[a], segments[b]))
        rough = [track_roughness(track, segments[s]) for s in segments]
        rough = [x for x in rough if x is not None]
        sepstr = " | ".join(f"{s * 100:8.1f} cm" for s in seps)
        rstr = f"{(sum(rough) / len(rough)) * 100:.1f} cm" if rough else "n/a"
        print(f"  {r:>10.4g} | {sepstr} | {rstr}")

    print("\nReading: as R shrinks the offset becomes fully applied (separation -> 0)")
    print("while roughness stays small until R is so tiny that GPS noise leaks in.")
    print("Pick the R where separation is ~0 but roughness is still low — that is the")
    print("measurement covariance the live filter should use (e.g. position_accuracy^2).")


if __name__ == "__main__":
    main(sys.argv)
