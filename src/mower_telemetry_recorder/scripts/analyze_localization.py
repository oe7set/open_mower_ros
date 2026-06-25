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
  analyze_localization.py <session.jsonl> [--offset <x_fwd> <y_left>] [--clean]
  analyze_localization.py <export.omheat.zip> [--offset 0.05 0.16] [--clean]

--clean keeps only near-straight driving segments (drops turns / approach
runs), which otherwise skew the per-direction statistics.
Theta is taken from ekf_theta when present, else from fused_theta (the EKF
pose heading carried in RobotState — available on every recording).

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


def analyze(samples: list[dict], cfg_offset: Optional[tuple[float, float]] = None, clean: bool = False) -> None:
    # Theta source: prefer the EKF debug topic, fall back to the fused pose
    # heading carried in RobotState (always present). Either pins the analysis.
    def theta_of(s: dict) -> Optional[float]:
        if "ekf_theta" in s:
            return float(s["ekf_theta"])
        if "fused_theta" in s:
            return float(s["fused_theta"])
        return None

    paired = [s for s in samples if "raw_gps_x" in s and "raw_gps_y" in s and theta_of(s) is not None]
    theta_src = "ekf_theta" if paired and "ekf_theta" in paired[0] else "fused_theta"
    print(f"samples total: {len(samples)}, usable (raw-gps + theta): {len(paired)}  [theta from {theta_src}]")
    if len(paired) < 10:
        raise SystemExit(
            "Not enough usable samples. Record with the localisation-debug recorder "
            "(OM_POSITIONING_DEBUG=true) and drive a bit."
        )

    # Fused position (x,y) is needed for derived speed/heading when ekf_vx/vr is
    # absent and for the per-direction body-frame check. Build an index list.
    P = paired

    def fused_xy(s: dict) -> Optional[tuple[float, float]]:
        if "x" in s and "y" in s:
            return float(s["x"]), float(s["y"])
        return None

    # Derived travel heading + speed from consecutive fused positions (centred
    # window), used both for the --clean straight filter and as a fallback when
    # ekf_vx/vr are not in the recording.
    def derived(i: int, w: int = 2) -> tuple[Optional[float], float]:
        a, b = fused_xy(P[max(0, i - w)]), fused_xy(P[min(len(P) - 1, i + w)])
        ta, tb = float(P[max(0, i - w)]["ts"]), float(P[min(len(P) - 1, i + w)]["ts"])
        if a is None or b is None or tb <= ta:
            return None, 0.0
        dx, dy = b[0] - a[0], b[1] - a[1]
        return math.atan2(dy, dx), math.hypot(dx, dy) / (tb - ta)

    travel = [derived(i) for i in range(len(P))]

    # --clean: keep only near-straight driving segments (sane speed, low heading
    # change) so turn/approach segments don't pollute the per-direction stats.
    keep = list(range(len(P)))
    if clean:
        kept = []
        for i in range(2, len(P) - 2):
            th, spd = travel[i]
            if th is None or spd < 0.15 or spd > 1.5:
                continue
            th0 = travel[max(0, i - 2)][0]
            th1 = travel[min(len(P) - 1, i + 2)][0]
            if th0 is None or th1 is None or abs(_wrap_pi(th1 - th0)) > math.radians(8):
                continue
            kept.append(i)
        keep = kept
        print(f"  --clean: {len(keep)}/{len(P)} near-straight samples retained")

    cfg_mag = math.hypot(*cfg_offset) if cfg_offset else None

    # --- Signal 1: heading bias (forward driving) --------------------------
    heading_err: list[float] = []
    for i in keep:
        s = P[i]
        mh = s.get("gps_motion_heading")
        if mh is None:
            continue
        vx = s.get("ekf_vx")
        spd = abs(float(vx)) if vx is not None else travel[i][1]
        if spd < 0.15:
            continue
        heading_err.append(_wrap_pi(theta_of(s) - float(mh)))

    print("\n== Signal 1: heading bias (theta - GPS motion heading, driving) ==")
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

    # --- Signal 1b: per-direction body-frame correction vector -------------
    # The correction (raw antenna - fused centre) rotated by travel heading must
    # be the SAME body-fixed offset in every driving direction if theta is good.
    # Variation across directions = theta error (works without spin or ekf).
    print("\n== Signal 1b: body-frame antenna correction per travel direction ==")
    cards = {0: "E", 90: "N", 180: "W", 270: "S"}
    groups: dict = {}
    for i in keep:
        th = travel[i][0]
        xy = fused_xy(P[i])
        if th is None or xy is None:
            continue
        deg = (math.degrees(th) + 360) % 360
        card = min(cards, key=lambda c: abs((deg - c + 180) % 360 - 180))
        dx = float(P[i]["raw_gps_x"]) - xy[0]
        dy = float(P[i]["raw_gps_y"]) - xy[1]
        co, si = math.cos(-th), math.sin(-th)
        groups.setdefault(card, []).append((co * dx - si * dy, si * dx + co * dy))
    body_means = {}
    for c in sorted(groups):
        rows = groups[c]
        fwd = _mean_std([r[0] for r in rows])[0]
        left = _mean_std([r[1] for r in rows])[0]
        body_means[c] = (fwd, left)
        print(f"  {cards[c]:>1} (n={len(rows):>3}): body fwd={fwd:+.3f}  left={left:+.3f} m")
    if cfg_offset is not None:
        print(f"  configured offset: fwd={cfg_offset[0]:+.3f}  left={cfg_offset[1]:+.3f} m "
              "(should match ALL directions if theta is correct)")

    # --- Signal 2: spin circle-fit (rotation in place) ---------------------
    # Samples turning fast but barely translating. Prefer ekf_vr; else detect a
    # high heading-rate with low speed from the derived track.
    spin = []
    for i in range(len(P)):
        s = P[i]
        vr = s.get("ekf_vr")
        vx = s.get("ekf_vx")
        if vr is not None:
            if abs(float(vr)) >= 0.2 and (vx is None or abs(float(vx)) < 0.1):
                spin.append(s)
        else:
            th0 = travel[max(0, i - 1)][0]
            th1 = travel[min(len(P) - 1, i + 1)][0]
            spd = travel[i][1]
            if th0 is not None and th1 is not None and abs(_wrap_pi(th1 - th0)) > math.radians(10) and spd < 0.1:
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
            cxs = [fused_xy(s)[0] for s in spin if fused_xy(s) is not None]
            cys = [fused_xy(s)[1] for s in spin if fused_xy(s) is not None]
            if cxs and cys:
                cx_m, cx_s = _mean_std(cxs)
                cy_m, cy_s = _mean_std(cys)
                jump = math.hypot(cx_s, cy_s)
                print(f"  fused centre during spin: mean=({cx_m:+.3f}, {cy_m:+.3f}) m, "
                      f"wander(std)={jump:.3f} m (should be ~0 if theta is right)")
    else:
        print(f"  not enough spin-in-place samples ({len(spin)}). For this test, rotate the")
        print("  robot in place ~360 deg during the recording.")

    # Cross-direction consistency of the body-frame correction. If theta is
    # correct it is identical in every direction; spread across directions is a
    # direct, motion-heading-independent symptom of a theta error.
    body_spread = None
    if len(body_means) >= 2:
        fwds = [v[0] for v in body_means.values()]
        lefts = [v[1] for v in body_means.values()]
        body_spread = math.hypot(_mean_std(fwds)[1], _mean_std(lefts)[1])

    # --- Verdict -----------------------------------------------------------
    print("\n== Verdict ==")
    theta_bad = (bias is not None and abs(math.degrees(bias)) > 5) or (body_spread is not None and body_spread > 0.03)
    if theta_bad:
        if bias is not None:
            print(f"  HEADING (theta) is off: ~{math.degrees(bias):+.0f} deg bias vs the true")
            print("  direction of travel.", end=" ")
        if body_spread is not None:
            print(f"Body-frame correction varies {body_spread * 100:.1f} cm across directions")
            print("  (should be ~0).", end=" ")
        print("\n  Combined with the antenna offset this flips the cross-track error between")
        print("  forward/return passes -> the lane/position offset you see.")
        print("  Fix: anchor theta to the GPS motion heading. Do NOT just flip the offset")
        print("  sign (that re-rotates the error and breaks turns, as already observed).")
    elif bias is not None or body_spread is not None:
        print("  Heading looks consistent. If an offset remains, check the offset VALUE")
        print("  against the spin circle-fit radius above.")
    else:
        print("  Could not measure heading. Record a forward drive (and ideally a")
        print("  spin-in-place) with the localisation-debug recorder and re-run.")


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
    clean = False
    if "--clean" in args:
        clean = True
        args.remove("--clean")
    if len(args) != 1:
        raise SystemExit(__doc__)
    samples = load_samples(args[0])
    analyze(samples, cfg_offset, clean=clean)


if __name__ == "__main__":
    main(sys.argv)
