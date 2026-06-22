#!/usr/bin/env python3
"""Offline localisation diagnosis for telemetry-recorder sessions.

Reads a session JSONL recorded with record_all_states=true (and
OM_POSITIONING_DEBUG=true) and answers one question: is the lane-collapse /
turn-wobble caused by a wrong antenna offset, or by a wrong heading (theta)?

Two INDEPENDENT signals are used (the naive "raw antenna minus fused centre"
vector is deliberately NOT used as a check: the EKF computes the centre as
antenna - R(theta)*offset, so that vector is R(theta)*offset by construction and
always equals the configured offset in the body frame, regardless of how wrong
theta is — it proves nothing):

  1. Heading consistency: EKF theta vs the GPS motion heading while driving
     forward. A constant offset between them is a heading bias; that bias times
     the antenna offset is exactly what flips the cross-track error sign between
     a forward and a return pass, collapsing the lanes.

  2. Spin circle-fit: while the robot rotates in place (high yaw rate, little
     translation) the raw antenna traces a circle whose centre is the TRUE
     rotation centre and whose radius is the TRUE antenna offset distance — both
     independent of theta. Comparing that radius to the configured offset
     magnitude, and the fitted centre to the EKF centre, exposes the error
     directly.

Usage:
  analyze_localization.py <session.jsonl> [--offset <x_fwd> <y_left>]
  analyze_localization.py <export.omheat.zip> [--offset 0.08 0.15]

Pass --offset with the configured antenna_offset_x/antenna_offset_y to compare
the measured offset magnitude against it.

No ROS or third-party deps — plain Python 3 stdlib so it runs anywhere
(developer laptop, the mower itself, CI).
"""

import json
import math
import sys
import zipfile
from typing import Iterator, Optional


def _iter_jsonl(text: str) -> Iterator[dict]:
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except ValueError:
            # Tolerate a truncated final line from a crashed recorder.
            continue


def load_samples(path: str) -> list[dict]:
    """Load samples from a raw .jsonl session or a .omheat.zip export."""
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


def _mean_std(values: list[float]) -> tuple[float, float]:
    n = len(values)
    if n == 0:
        return float("nan"), float("nan")
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return mean, math.sqrt(max(0.0, var))


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _circular_mean(angles: list[float]) -> float:
    if not angles:
        return float("nan")
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def _fit_circle(xs: list[float], ys: list[float]) -> Optional[tuple[float, float, float]]:
    """Kasa algebraic circle fit. Returns (cx, cy, radius) or None if singular."""
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    # Work centred for numerical stability.
    u = [x - mx for x in xs]
    v = [y - my for y in ys]
    suu = sum(a * a for a in u)
    svv = sum(b * b for b in v)
    suv = sum(a * b for a, b in zip(u, v))
    suuu = sum(a * a * a for a in u)
    svvv = sum(b * b * b for b in v)
    suvv = sum(a * b * b for a, b in zip(u, v))
    svuu = sum(b * a * a for a, b in zip(u, v))
    det = suu * svv - suv * suv
    if abs(det) < 1e-12:
        return None
    c1 = 0.5 * (suuu + suvv)
    c2 = 0.5 * (svvv + svuu)
    uc = (c1 * svv - c2 * suv) / det
    vc = (c2 * suu - c1 * suv) / det
    cx = uc + mx
    cy = vc + my
    radius = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    return cx, cy, radius


def analyze(samples: list[dict], cfg_offset: Optional[tuple[float, float]] = None) -> None:
    paired = [
        s for s in samples if all(k in s for k in ("raw_gps_x", "raw_gps_y", "ekf_theta"))
    ]
    print(f"samples total: {len(samples)}, with raw-gps+ekf: {len(paired)}")
    if len(paired) < 10:
        raise SystemExit(
            "Not enough debug samples. Record with OM_POSITIONING_DEBUG=true "
            "(enables kalman_state + record_all_states) and drive a bit."
        )

    cfg_mag = math.hypot(*cfg_offset) if cfg_offset else None

    # --- Signal 1: heading bias (forward driving) --------------------------
    heading_err: list[float] = []
    for s in paired:
        mh = s.get("gps_motion_heading")
        vx = s.get("ekf_vx")
        if mh is None:
            continue
        # Only when clearly translating, so the motion heading is meaningful.
        if vx is not None and abs(float(vx)) < 0.15:
            continue
        heading_err.append(_wrap_pi(float(s["ekf_theta"]) - float(mh)))

    print("\n== Signal 1: heading bias (EKF theta - GPS motion heading, driving) ==")
    bias = None
    if heading_err:
        bias = _circular_mean(heading_err)
        spread = _mean_std([math.degrees(_wrap_pi(e - bias)) for e in heading_err])[1]
        print(f"  moving samples: {len(heading_err)}")
        print(f"  mean bias: {math.degrees(bias):+.1f} deg   spread(std): {spread:.1f} deg")
        if cfg_mag is not None:
            xterr = cfg_mag * math.sin(abs(bias))
            print(f"  => cross-track error from this bias at offset {cfg_mag:.3f} m: "
                  f"{xterr * 100:.1f} cm per pass ({2 * xterr * 100:.1f} cm lane-to-lane).")
    else:
        print("  no forward-driving samples with a GPS motion heading found.")

    # --- Signal 2: spin circle-fit (rotation in place) ---------------------
    # Pick samples where the robot is turning fast but barely translating.
    spin = []
    for s in paired:
        vr = s.get("ekf_vr")
        vx = s.get("ekf_vx")
        if vr is None:
            continue
        if abs(float(vr)) >= 0.2 and (vx is None or abs(float(vx)) < 0.1):
            spin.append(s)

    print("\n== Signal 2: spin circle-fit (raw antenna while rotating in place) ==")
    if len(spin) >= 8:
        fit = _fit_circle([float(s["raw_gps_x"]) for s in spin],
                          [float(s["raw_gps_y"]) for s in spin])
        if fit is None:
            print("  circle fit failed (degenerate geometry).")
        else:
            cx, cy, radius = fit
            print(f"  spin samples: {len(spin)}")
            print(f"  fitted true rotation centre: ({cx:+.3f}, {cy:+.3f}) m")
            print(f"  fitted antenna offset magnitude (radius): {radius:.3f} m")
            if cfg_mag is not None:
                print(f"  configured offset magnitude: {cfg_mag:.3f} m   "
                      f"(diff {abs(radius - cfg_mag) * 100:.1f} cm)")
            exs = [float(s["ekf_x"]) for s in spin if "ekf_x" in s]
            eys = [float(s["ekf_y"]) for s in spin if "ekf_y" in s]
            if exs and eys:
                ex_m, ex_s = _mean_std(exs)
                ey_m, ey_s = _mean_std(eys)
                jump = math.hypot(ex_s, ey_s)
                print(f"  EKF centre during spin: mean=({ex_m:+.3f}, {ey_m:+.3f}) m, "
                      f"wander(std)={jump:.3f} m (should be ~0 if theta is right)")
    else:
        print(f"  not enough spin-in-place samples ({len(spin)}). For this test, rotate the")
        print("  robot in place ~360 deg during the recording.")

    # --- Verdict -----------------------------------------------------------
    print("\n== Verdict ==")
    if bias is not None and abs(math.degrees(bias)) > 5:
        print(f"  HEADING (theta) has a ~{math.degrees(bias):+.0f} deg bias vs the true")
        print("  direction of travel. Combined with the antenna offset this flips the")
        print("  cross-track error between forward/return passes -> lane collapse.")
        print("  Fix: anchor theta to the GPS motion heading. Do NOT just flip the offset")
        print("  sign (that re-rotates the error and breaks turns, as already observed).")
    elif bias is not None:
        print("  Heading bias is small. If lanes still collapse, capture a spin-in-place")
        print("  segment and/or a longer multi-direction drive and re-run.")
    else:
        print("  Could not measure a heading bias (no usable motion heading). Record a")
        print("  forward drive with OM_POSITIONING_DEBUG=true and a single antenna moving.")


def main(argv: list[str]) -> None:
    args = argv[1:]
    cfg_offset: Optional[tuple[float, float]] = None
    if "--offset" in args:
        i = args.index("--offset")
        try:
            cfg_offset = (float(args[i + 1]), float(args[i + 2]))
        except (IndexError, ValueError):
            raise SystemExit("--offset needs two numbers: --offset <x_forward> <y_left>")
        del args[i:i + 3]
    if len(args) != 1:
        raise SystemExit(__doc__)
    samples = load_samples(args[0])
    analyze(samples, cfg_offset)


if __name__ == "__main__":
    main(sys.argv)
