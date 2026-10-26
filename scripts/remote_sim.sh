#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-help}"
if [[ "${action}" == "help" || "${action}" == "--help" ]]; then
  cat <<'USAGE'
Usage: remote_sim.sh <start|build|sync|stop|status|logs>

Set DRONE_REMOTE_HOST and DRONE_REMOTE_REPO for the provisioned simulation host.
DRONE_REMOTE_SSH optionally selects the SSH executable (including Windows ssh.exe
when this script runs in WSL). SSH key authentication must already work.

start   Stop the previous flight, mirror local source, and start the remote GUI.
        The normal simulation target performs an incremental workspace build.
build   Stop the previous flight, mirror source, and build without starting a flight.
sync    Mirror source only; refuses to modify a running simulation.
stop    Stop the remote simulation without affecting unrelated projects.
status  Show the remote simulation service status.
logs    Follow the remote build and simulation journal; Ctrl+C only stops viewing.

Local source is authoritative. Sync mirrors Git-indexed source and repository
metadata, deletes obsolete tracked source, and preserves generated/private data.
Add new source files to Git before syncing; untracked files are not deployed.
USAGE
  exit 0
fi
case "${action}" in
  start|build|sync|stop|status|logs) ;;
  *) echo "Unknown action: ${action}" >&2; exit 2 ;;
esac

: "${DRONE_REMOTE_HOST:?Set DRONE_REMOTE_HOST to the simulation SSH destination}"
: "${DRONE_REMOTE_REPO:?Set DRONE_REMOTE_REPO to its absolute project directory}"
ssh_bin="${DRONE_REMOTE_SSH:-ssh}"
ssh_args=(-n -T -o BatchMode=yes -o ConnectTimeout=10)
service="px4-nav-simulation.service"
printf -v remote_repo_q '%q' "${DRONE_REMOTE_REPO}"

remote() {
  "${ssh_bin}" "${ssh_args[@]}" "${DRONE_REMOTE_HOST}" "$1"
}

stop_simulation() {
  remote "systemctl --user stop ${service}"
}

sync_source() (
  if remote "systemctl --user is-active --quiet ${service}"; then
    echo "Stop the remote simulation before syncing, or use start/build." >&2
    exit 2
  fi
  # Provisioning creates this marker only in the dedicated source mirror.
  remote "test -f ${remote_repo_q}/.remote-sim-managed"
  local sync_dir path escaped
  sync_dir="$(mktemp -d)"
  trap 'rm -rf -- "${sync_dir}"' EXIT
  git -C "${repo_root}" ls-files --cached -z >"${sync_dir}/paths"
  # Include old tracked paths so obsolete source is deleted, without granting
  # deletion authority over remote untracked evidence or private configuration.
  remote "git -C ${remote_repo_q} ls-files --cached -z" >>"${sync_dir}/paths"
  while IFS= read -r -d '' path; do
    while [[ -n "${path}" ]]; do
      escaped="${path//\\/\\\\}"
      escaped="${escaped//\*/\\*}"
      escaped="${escaped//\?/\\?}"
      escaped="${escaped//\[/\\[}"
      printf '/%s\0' "${escaped}" >>"${sync_dir}/includes"
      [[ "${path}" == */* ]] || break
      path="${path%/*}"
      # Parent directories must be traversable, but their other files stay out.
      escaped="${path//\\/\\\\}"
      escaped="${escaped//\*/\\*}"
      escaped="${escaped//\?/\\?}"
      escaped="${escaped//\[/\\[}"
      printf '/%s/\0' "${escaped}" >>"${sync_dir}/includes"
    done
  done <"${sync_dir}/paths"
  touch "${sync_dir}/includes"
  # Only indexed source is eligible: generated/private untracked paths are
  # protected without guessing from names such as build*, log* or external.
  # Copy only portable Git metadata, not host config, hooks, reflogs or URLs.
  # Never add --delete-excluded: excluded remote data belongs to that host.
  # Windows OpenSSH through WSL interop needs blocking transport pipes.
  rsync -a --blocking-io --delete-delay --info=stats2 \
    --exclude=/.remote-sim-managed \
    --exclude=/.colcon/ \
    --exclude=/.git/objects/info/alternates \
    --exclude=/.git/objects/info/http-alternates \
    --include=/.git/ --include=/.git/HEAD --include=/.git/index \
    --include=/.git/packed-refs --include=/.git/shallow \
    --include='/.git/refs/***' --include='/.git/objects/***' \
    --from0 --include-from="${sync_dir}/includes" \
    --exclude='*' \
    -e "${ssh_bin} -o BatchMode=yes -o ConnectTimeout=10" \
    "${repo_root}/" "${DRONE_REMOTE_HOST}:${DRONE_REMOTE_REPO}/"
)

case "${action}" in
  start)
    stop_simulation
    sync_source
    remote "systemctl --user start ${service}"
    echo "Remote simulation starting; use logs to follow build and readiness."
    ;;
  build)
    stop_simulation
    sync_source
    remote "cd ${remote_repo_q} && bash scripts/container_run.sh env CMAKE_BUILD_PARALLEL_LEVEL=8 MAKEFLAGS=-j8 make build"
    ;;
  sync) sync_source ;;
  stop) stop_simulation ;;
  status) remote "systemctl --user status --no-pager --full ${service}" ;;
  logs) remote "journalctl --user -u ${service} --no-pager -n 80 --follow" ;;
esac
