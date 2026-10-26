#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || "$1" != --managed-mirror ]]; then
  echo "Usage: bash deployment/systemd/install.sh --managed-mirror" >&2
  echo "Run as the simulation user on the dedicated remote source mirror only." >&2
  exit 2
fi
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
expected_root="$(cd "${HOME}/projects/px4-ros2-drone-nav" && pwd -P)"
if [[ "${repo_root}" != "${expected_root}" || "${EUID}" -eq 0 ]]; then
  echo "Install from ~/projects/px4-ros2-drone-nav as its non-root simulation user." >&2
  exit 2
fi
# Private viewer credentials and the graphical login are provisioned separately.
# Installing definitions neither resets credentials nor changes running services.
unit_dir="${HOME}/.config/systemd/user"
mkdir -p "${unit_dir}"
for unit in px4-nav-simulation.service px4-nav-desktop.service px4-nav-web.service; do
  source_unit="${repo_root}/deployment/systemd/${unit}"
  if ! cmp -s "${source_unit}" "${unit_dir}/${unit}"; then
    install -m 0644 "${source_unit}" "${unit_dir}/${unit}"
  fi
done
systemctl --user daemon-reload
if [[ ! -e "${repo_root}/.remote-sim-managed" ]]; then
  (umask 077; touch "${repo_root}/.remote-sim-managed")
fi
echo "Installed three user service definitions; no services were started, stopped or enabled."
echo "The dedicated remote source mirror is marked for tracked-source synchronization."
