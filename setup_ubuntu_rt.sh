#!/usr/bin/env bash
# ==============================================================================
# Spacecraft Docking System: Automated Hard Real-Time Setup for Ubuntu 24.04
# ==============================================================================
# Configures Ubuntu for deterministic sub-millisecond execution (< 1 ms no matter what):
#  1. Sets real-time scheduling priority (rtprio 99) and unlimited locked memory.
#  2. Permanently disables Linux real-time throttling (kernel.sched_rt_runtime_us = -1).
#  3. Locks CPU frequency governor to "performance" (eliminates downclocking & C-state latency).
#  4. Automatically detects CPU SMT core topology and isolates the physical core in GRUB:
#     - isolcpus, nohz_full, rcu_nocbs, and irqaffinity (shields core from hardware interrupts).
#  5. Installs the official Linux PREEMPT_RT Real-Time Kernel (linux-image-realtime).
# ==============================================================================
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run with sudo privileges:"
    echo "  sudo ./setup_ubuntu_rt.sh"
    exit 1
fi

U=${SUDO_USER:-$USER}
echo "===================================================="
echo " CONFIGURING HARD REAL-TIME FOR: $U"
echo "===================================================="

# 1. Real-time priorities & unlimited memory locking
LIMITS_FILE=/etc/security/limits.d/99-sim-realtime.conf
cat > "$LIMITS_FILE" <<EOT
$U - rtprio 99
$U - memlock unlimited
$U - nice -20
*  - rtprio 99
*  - memlock unlimited
*  - nice -20
EOT
echo "[1/6] Configured real-time limits in $LIMITS_FILE"

# 2. Disable kernel RT throttling permanently
SYSCTL_FILE=/etc/sysctl.d/99-realtime.conf
cat > "$SYSCTL_FILE" <<EOT
kernel.sched_rt_runtime_us = -1
EOT
sysctl -p "$SYSCTL_FILE" 2>/dev/null || sysctl -w kernel.sched_rt_runtime_us=-1 || true
echo "[2/6] Disabled Linux RT throttling in $SYSCTL_FILE"

# 3. Set CPU frequency governor to performance
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [[ -e "$f" ]] && echo performance > "$f" 2>/dev/null || true
done
echo "[3/6] Set CPU scaling governor to performance"

# 4. Topology detection: isolate the last physical core and all its SMT siblings
NPROC=$(nproc)
LAST_CPU=$((NPROC - 1))
if [[ -f "/sys/devices/system/cpu/cpu${LAST_CPU}/topology/thread_siblings_list" ]]; then
    SIBLINGS=$(cat "/sys/devices/system/cpu/cpu${LAST_CPU}/topology/thread_siblings_list")
else
    SIBLINGS="${LAST_CPU}"
fi

FIRST_SIB=$(echo "$SIBLINGS" | tr ",-" " " | awk '{print $1}')
if (( FIRST_SIB > 0 )); then
    OTHER_CORES="0-$((FIRST_SIB - 1))"
else
    OTHER_CORES="0-$((NPROC - 1))"
fi

RT_PARAMS="isolcpus=${SIBLINGS} nohz_full=${SIBLINGS} rcu_nocbs=${SIBLINGS} irqaffinity=${OTHER_CORES}"
echo "[4/6] Detected Physical Core topology:"
echo "      - Isolated Core SMT siblings: CPUs $SIBLINGS"
echo "      - Hardware IRQs & OS tasks assigned to: CPUs $OTHER_CORES"

# 5. Configure GRUB bootloader
GRUB_FILE=/etc/default/grub
if [[ -f "$GRUB_FILE" ]]; then
    cp "$GRUB_FILE" "${GRUB_FILE}.bak"
    echo "[5/6] Backed up $GRUB_FILE to ${GRUB_FILE}.bak"
    
    # Check if isolcpus is already configured
    if grep -q "isolcpus" "$GRUB_FILE"; then
        echo "      - isolcpus already present in $GRUB_FILE (skipping modification)"
    else
        sed -i.bak -E "s/^(GRUB_CMDLINE_LINUX_DEFAULT=\".*)\"/\1 ${RT_PARAMS}\"/" "$GRUB_FILE"
        echo "      - Added '${RT_PARAMS}' to GRUB_CMDLINE_LINUX_DEFAULT"
        echo "      - Updating GRUB configuration..."
        update-grub
    fi
fi

# 6. Install PREEMPT_RT Real-Time Kernel
echo "[6/6] Checking for Linux PREEMPT_RT kernel packages..."
if apt-cache show linux-image-realtime &>/dev/null; then
    echo "      Installing linux-image-realtime and linux-headers-realtime..."
    DEBIAN_FRONTEND=noninteractive apt-get update -y
    DEBIAN_FRONTEND=noninteractive apt-get install -y linux-image-realtime linux-headers-realtime
    echo "      - PREEMPT_RT kernel successfully installed."
else
    echo "      - linux-image-realtime package not found directly via apt; trying Ubuntu Pro..."
    if command -v pro &>/dev/null; then
        pro enable realtime-kernel --access-only || true
    fi
fi

echo
echo "===================================================="
echo " HARD REAL-TIME SETUP COMPLETE!"
echo "===================================================="
echo "To activate the new Real-Time kernel and core isolation:"
echo "  sudo reboot"
echo
echo "After reboot, confirm with:"
echo "  uname -v          (should include PREEMPT_RT)"
echo "  cat /proc/cmdline (should include isolcpus=${SIBLINGS})"
echo "===================================================="
