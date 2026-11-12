#!/usr/bin/env bash
# Drop inbound MAVLink to PX4 SITL from anywhere but this host.
#
# PX4 SITL binds its MAVLink sockets to 0.0.0.0: the GCS link on 18570, the
# offboard link on 14580, the payload link on 14280 and the gimbal link on 13030
# (ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink). PX4's own `-x` flag is the
# FTP server, not a bind restriction, and MAV_{i}_BROADCAST only governs where
# PX4 sends. So a peer on the network can address those ports and command the
# vehicle without ever binding anything locally - which the userspace guard in
# run_sim_wrapped.sh cannot stop, and which was observed: a flight under that
# guard logged `partner IP: 192.168.110.76`
# (log/runs/mavlink_guard_gate_20261112.txt).
#
# This script closes that path on the host: packets to those ports are accepted
# on loopback, where the simulator, the navigation stack and PX4's own streams
# live, and dropped on every other interface. Docker's own chains are not
# touched; the rule sits in its own chain so it can be read and removed whole.
#
#   scripts/host_mavlink_firewall.sh status    # show the chain and its counters
#   scripts/host_mavlink_firewall.sh apply     # install (idempotent)
#   scripts/host_mavlink_firewall.sh remove    # take it out again
#
# Needs root, and sudo resolves a relative script path against its own secure
# path, so name the interpreter:
#
#   sudo bash scripts/host_mavlink_firewall.sh apply
# The rule is not persisted across a reboot: run `apply` again after one, from
# whatever starts the simulation host's services. It blocks a MAVLink ground
# station or telemetry radio in another machine on purpose; list the interface
# or address to keep in ALLOW_INTERFACES to have a trust boundary instead.
set -euo pipefail

CHAIN="${MAVLINK_FIREWALL_CHAIN:-PX4_MAVLINK_LOCAL}"
PORTS="${MAVLINK_FIREWALL_PORTS:-18570 14580 14280 13030}"
# Interfaces that may still reach the ports, besides loopback. Empty by default:
# the contract is that nothing off this host commands the vehicle.
ALLOW_INTERFACES="${MAVLINK_FIREWALL_ALLOW_INTERFACES:-}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script needs root: sudo $0 $*" >&2
  exit 2
fi

status() {
  if ! iptables -n -L "${CHAIN}" >/dev/null 2>&1; then
    echo "MAVLINK_FIREWALL installed=false chain=${CHAIN}"
    return 1
  fi
  local rules
  rules="$(iptables -S "${CHAIN}" | tail -n +2)"
  echo "MAVLINK_FIREWALL installed=true chain=${CHAIN} ports=${PORTS} allow=${ALLOW_INTERFACES:-none}"
  printf '%s\n' "${rules}"
  iptables -vn -L "${CHAIN}" | sed -n '1,20p'
}

apply() {
  if iptables -n -L "${CHAIN}" >/dev/null 2>&1; then
    iptables -F "${CHAIN}"
  else
    iptables -N "${CHAIN}"
  fi
  for port in ${PORTS}; do
    # Loopback first: the simulator's own traffic must keep working.
    iptables -A "${CHAIN}" -i lo -p udp --dport "${port}" -j ACCEPT
    for interface in ${ALLOW_INTERFACES}; do
      iptables -A "${CHAIN}" -i "${interface}" -p udp --dport "${port}" -j ACCEPT
    done
    iptables -A "${CHAIN}" -p udp --dport "${port}" \
      -j LOG --log-prefix "PX4_MAVLINK_DROP " --log-level 4 2>/dev/null || true
    iptables -A "${CHAIN}" -p udp --dport "${port}" -j DROP
  done
  # One jump, so the chain is reached at all and can be removed in one step.
  if ! iptables -C INPUT -j "${CHAIN}" >/dev/null 2>&1; then
    iptables -I INPUT 1 -j "${CHAIN}"
  fi
  status
}

remove() {
  if iptables -C INPUT -j "${CHAIN}" >/dev/null 2>&1; then
    iptables -D INPUT -j "${CHAIN}"
  fi
  if iptables -n -L "${CHAIN}" >/dev/null 2>&1; then
    iptables -F "${CHAIN}"
    iptables -X "${CHAIN}"
  fi
  echo "MAVLINK_FIREWALL installed=false chain=${CHAIN} removed=true"
}

case "${1:-status}" in
  apply) apply ;;
  remove) remove ;;
  status) status ;;
  -h | --help)
    sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Usage: $0 [status|apply|remove]" >&2
    exit 2
    ;;
esac
