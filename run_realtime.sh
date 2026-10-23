#!/usr/bin/env bash
# Run the sim with the system settings that give the lowest jitter on Ubuntu.
#
#   sudo -E ./run_realtime.sh                       # GUI, physics on the last core
#   sudo -E SIM_CPU=3 ./run_realtime.sh             # choose the core
#   sudo -E ./run_realtime.sh --headless --seconds 60 --no-log
#
# What it does (and undoes when the sim exits):
#   1. disables kernel real-time throttling (otherwise a busy real-time thread
#      is paused ~50 ms every second)
#   2. sets the CPU frequency governor to "performance"
#   3. runs the sim pinned to one core, busy-waiting (--spin), real-time priority
#
# For the best result also isolate that core at boot - see README "Hard real-time".
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo:  sudo -E ./run_realtime.sh" >&2
    exit 1
fi

cd "$(dirname "$0")"
BIN=./build/sim

if [[ ! -x $BIN ]]; then
    echo "Build first:  cmake -B build && cmake --build build --target sim -j" >&2
    exit 1
fi

NCPU=$(nproc)
CPU=${SIM_CPU:-$((NCPU - 1))}

if (( NCPU < 2 )); then
    echo "WARNING: only 1 CPU core - cannot dedicate a core to the physics thread." >&2
fi

# ---- remember current settings so they can be restored ----
OLD_RT=$(cat /proc/sys/kernel/sched_rt_runtime_us)
declare -A OLD_GOV=()
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [[ -e $f ]] && OLD_GOV[$f]=$(cat "$f")
done

restore() {
    echo "$OLD_RT" > /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || true
    for f in "${!OLD_GOV[@]}"; do
        echo "${OLD_GOV[$f]}" > "$f" 2>/dev/null || true
    done
    echo "[run_realtime] system settings restored"
}
trap restore EXIT

# ---- apply low-jitter settings ----
echo -1 > /proc/sys/kernel/sched_rt_runtime_us || echo "WARNING: could not disable RT throttling" >&2
for f in "${!OLD_GOV[@]}"; do
    echo performance > "$f" 2>/dev/null || true
done

echo "[run_realtime] physics on CPU $CPU of $NCPU, spin mode, RT throttling off"

# ---- run ----
if [[ -n ${SUDO_UID:-} ]]; then
    # Run as the normal user (so the GUI can reach your display) but keep the
    # real-time and memory-lock limits raised.
    ulimit -r 99 2>/dev/null || echo "WARNING: could not raise the real-time priority limit" >&2
    ulimit -l unlimited 2>/dev/null || true
    setpriv --reuid "$SUDO_UID" --regid "$SUDO_GID" --init-groups \
        "$BIN" --cpu "$CPU" --spin "$@"
else
    "$BIN" --cpu "$CPU" --spin "$@"
fi
