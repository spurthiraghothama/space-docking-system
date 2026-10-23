#!/usr/bin/env python3
"""Verify that the physics loop really ran at the target rate, and how strictly.

Reads timestep.csv (written by sim) and prints a verdict.

    python3 check_timing.py                          # timestep.csv, target 1000 us
    python3 check_timing.py run1.csv --target-us 500
    python3 check_timing.py --limit-us 20            # strict: no tick may start > 20 us late

Two things are checked:
  * average rate  - did it really average 1 kHz (no drift)?
  * strict limit  - did EVERY tick start within --limit-us of its scheduled
                    time (latency_us column)? One bad tick = FAIL.

No third-party packages needed.
"""
import argparse
import csv
import statistics
import sys


def percentile(sorted_vals, q):
    return sorted_vals[int(q * (len(sorted_vals) - 1))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", default="timestep.csv")
    ap.add_argument("--target-us", type=float, default=1000.0, help="expected period (default 1000)")
    ap.add_argument("--limit-us", type=float, default=50.0,
                    help="strict limit: max allowed start delay of any tick (default 50)")
    args = ap.parse_args()

    steps, times, periods, lats = [], [], [], []
    try:
        with open(args.file, newline="") as f:
            for row in csv.DictReader(f):
                steps.append(int(row["step"]))
                times.append(float(row["timestamp_s"]))
                periods.append(float(row["period_us"]))
                if "latency_us" in row:
                    lats.append(float(row["latency_us"]))
    except FileNotFoundError:
        sys.exit(f"{args.file} not found. Run the sim first (it writes timestep.csv).")

    if len(times) < 3:
        sys.exit("Not enough rows to analyse.")

    p = periods[1:]                       # first tick has no previous tick
    p_sorted = sorted(p)
    n = len(p)
    target = args.target_us

    duration = times[-1] - times[0]
    avg_rate = (len(times) - 1) / duration if duration > 0 else 0.0
    late = sum(1 for v in p if v > 1.5 * target)
    over_target = sum(1 for v in p if v > target + args.limit_us)
    missing = (steps[-1] - steps[0] + 1) - len(steps)

    print(f"file             : {args.file}")
    print(f"ticks            : {len(times)}  over {duration:.3f} s")
    print(f"target period    : {target:.0f} us ({1e6 / target:.0f} Hz)")
    print(f"average period   : {statistics.fmean(p):.2f} us")
    print(f"overall rate     : {avg_rate:.2f} Hz")
    print(f"period min / max : {p_sorted[0]:.1f} / {p_sorted[-1]:.1f} us")
    print(f"std deviation    : {statistics.pstdev(p):.2f} us")
    print(f"p50 / p99 / p99.9: {percentile(p_sorted, 0.5):.1f} / "
          f"{percentile(p_sorted, 0.99):.1f} / {percentile(p_sorted, 0.999):.1f} us")
    print(f"periods > target+{args.limit_us:.0f} us: {over_target}")
    print(f"late (>1.5x)     : {late}")
    print(f"missing rows     : {missing}")

    rate_ok = abs(avg_rate - 1e6 / target) / (1e6 / target) < 0.001
    print()
    print("Average rate     :", "PASS (within 0.1% of target)" if rate_ok else "FAIL")

    if lats:
        ls = sorted(lats)
        bad = [(steps[i], v) for i, v in enumerate(lats) if v > args.limit_us]
        print(f"start delay      : max {ls[-1]:.1f} us, p99 {percentile(ls, 0.99):.1f} us, "
              f"p99.9 {percentile(ls, 0.999):.1f} us")
        print(f"ticks > {args.limit_us:.0f} us late: {len(bad)} of {len(lats)}")
        if bad:
            worst = sorted(bad, key=lambda t: -t[1])[:5]
            print("worst ticks      : " + ", ".join(f"step {s} (+{v:.0f} us)" for s, v in worst))
        print("Strict limit     :", f"PASS (no tick started more than {args.limit_us:.0f} us late)"
              if not bad else f"FAIL ({len(bad)} ticks exceeded {args.limit_us:.0f} us)")
    else:
        print("Strict limit     : n/a (no latency_us column; rebuild with the latest sim.cpp)")


if __name__ == "__main__":
    main()
