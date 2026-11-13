#!/usr/bin/env bash
# How the flight starts PX4 SITL and how it learns that PX4 left.
#
# PX4 is a background job of the launcher and, once the flight is under way,
# nothing watched it: three flights lost it in the air with no message from PX4,
# every topic in their ulogs stopping on one stamp while Gazebo and the stack
# kept running, and the vehicle flew on under frozen actuator outputs until it
# hit a building (log/runs/px4_termination_20261112.txt, and the correction at
# the top of log/runs/20261113T091953Z-1573/root_cause.txt). The functions here
# start that job, name the autopilot process inside it, and record how it left.
#
# Sourced by scripts/run_drone_nav_sim.sh; reads and sets its caller's variables.

# `kill -0` does not answer whether a process is alive: a child that has exited
# but not been reaped still answers it, and nothing waited on PX4 before. The
# zombie state is what distinguishes the two.
px4_process_alive() {
  local pid="$1"
  local state

  [[ -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null)" || return 1
  [[ "${state}" != "Z" ]]
}

# Starts one PX4 per vehicle and leaves px4_pid (the first) and
# multi_vehicle_px4_pids set, the way the launcher used to do inline.
# Runs one autopilot with the parameter stream on its stdin and records its own
# exit status where it happens.
#
# The stream is a pipe in the launcher's older form, and it never ends by
# itself: px4_parameter_stream ends in `while true; do sleep 3600; done`, which
# holds px4's stdin open so the autopilot's console can be typed into. A pipeline
# therefore returns only when *both* stages end, and a killed autopilot left the
# pipeline waiting on the stream: the status was unobtainable, and the first
# version of this recorded `exit_status=unknown` on a SIGKILL it had caused
# itself. A FIFO on the same stdin lets the autopilot be a child of this shell,
# so its status is here the moment it ends: 137 is a SIGKILL (an OOM killer or a
# supervisor), 139 a SIGSEGV.
#
# launch_px4_with_parameter_stream <cruise> <maximum> <status file> <launcher> [args]
launch_px4_with_parameter_stream() {
  local cruise_speed="$1"
  local maximum_speed="$2"
  local status_file="$3"
  local launcher="$4"
  shift 4
  local fifo="${runtime_artifact_dir}/px4_stdin_${BASHPID}.fifo"
  local stream_pid
  local status=0

  rm -f "${fifo}"
  mkfifo -m 600 "${fifo}"
  px4_parameter_stream "${cruise_speed}" "${maximum_speed}" > "${fifo}" 2>&1 &
  stream_pid=$!
  ( "${launcher}" "$@" < "${fifo}" ) || status=$?
  kill "${stream_pid}" 2>/dev/null || true
  wait "${stream_pid}" 2>/dev/null || true
  rm -f "${fifo}"
  printf 'status=%s\n' "${status}" > "${status_file}"
}

start_px4_sitl_processes() {
  local instance

  echo "PX4 SITL log: ${px4_log_file}"
  if bool_is_true "${multi_vehicle_mission}"; then
    echo "Multi-vehicle scenario: ${multi_vehicle_scenario_path}"
    for instance in "${!multi_vehicle_ids[@]}"; do
      echo "MULTI_VEHICLE_COORDINATE_CONTRACT vehicle_id='${multi_vehicle_ids[instance]}' map_start='${multi_vehicle_map_start_poses[instance]}' gazebo_spawn='${multi_vehicle_gazebo_spawn_poses[instance]}' gazebo_model='${multi_vehicle_gazebo_model_names[instance]}'"
      echo "PX4 SITL log for ${multi_vehicle_ids[instance]}: ${multi_vehicle_px4_logs[instance]}"
    done
    multi_vehicle_px4_cruise_speeds=()
    multi_vehicle_px4_maximum_speeds=()
    for instance in "${!multi_vehicle_ids[@]}"; do
      multi_vehicle_px4_cruise_speeds+=("${px4_active_cruise_speed_mps}")
      multi_vehicle_px4_maximum_speeds+=("${px4_active_max_horizontal_speed_mps}")
    done
    multi_vehicle_px4_pids=()
    for instance in "${!multi_vehicle_ids[@]}"; do
      reset_px4_instance_state "${instance}"
      (
        declare -x PX4_GZ_WORLD="${world_name}"
        declare -x PX4_GZ_STANDALONE=1
        declare -x PX4_GZ_MODEL_POSE="${multi_vehicle_gazebo_spawn_poses[instance]}"
        declare -x PX4_SIM_MODEL="${multi_vehicle_px4_model_targets[instance]}"
        declare -x PX4_UXRCE_DDS_NS="${multi_vehicle_px4_namespaces[instance]}"
        declare -x PX4_SYS_AUTOSTART=4001
        declare -x HEADLESS="${headless}"
        launch_px4_with_parameter_stream \
          "${multi_vehicle_px4_cruise_speeds[instance]}" \
          "${multi_vehicle_px4_maximum_speeds[instance]}" \
          "${runtime_artifact_dir}/px4_job_exit_${instance}.txt" \
          run_px4_instance "${instance}"
      ) > "${multi_vehicle_px4_logs[instance]}" 2>&1 &
      multi_vehicle_px4_pids+=("$!")
    done
    px4_pid="${multi_vehicle_px4_pids[0]}"
  else
    echo "PX4 Gazebo spawn pose: ${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}"
    reset_px4_instance_state 0
    (
      declare -x PX4_GZ_WORLD="${world_name}"
      declare -x PX4_GZ_STANDALONE=1
      declare -x PX4_GZ_MODEL_POSE="${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}"
      declare -x PX4_SIM_MODEL="${px4_model_target}"
      declare -x PX4_SYS_AUTOSTART=4001
      declare -x HEADLESS="${headless}"
      launch_px4_with_parameter_stream \
        "${px4_active_cruise_speed_mps}" \
        "${px4_active_max_horizontal_speed_mps}" \
        "${runtime_artifact_dir}/px4_job_exit.txt" \
        run_px4_sitl
    ) > "${px4_log_file}" 2>&1 &
    px4_pid=$!
  fi
}

check_px4_sitl_started() {
  local instance
  local candidate

  sleep "${startup_sleep_s}"

  if ! kill -0 "${px4_pid}" 2>/dev/null; then
    echo "PX4 SITL exited before ROS launch. Last PX4 log lines:" >&2
    tail -n 80 "${px4_log_file}" >&2
    exit 1
  fi
  px4_autopilot_pid=""
  for candidate in ${px4_pid} $(collect_descendant_pids "${px4_pid}" 2>/dev/null || true); do
    [[ -r "/proc/${candidate}/cmdline" ]] || continue
    if tr '\0' ' ' < "/proc/${candidate}/cmdline" |
      grep -q 'px4_sitl_default/bin/px4'; then
      px4_autopilot_pid="${candidate}"
      break
    fi
  done
  echo "${px4_autopilot_pid:-${px4_pid}}" > "${runtime_artifact_dir}/px4.pid"
  if bool_is_true "${multi_vehicle_mission}"; then
    for instance in "${!multi_vehicle_ids[@]}"; do
      if ! kill -0 "${multi_vehicle_px4_pids[instance]}" 2>/dev/null; then
        echo "${multi_vehicle_px4_namespaces[instance]} PX4 SITL exited before ROS launch. Last log lines:" >&2
        tail -n 80 "${multi_vehicle_px4_logs[instance]}" >&2
        exit 1
      fi
    done
  fi
}

# Records how the autopilot left: phase=in_flight or phase=teardown, decided on
# the autopilot's own process, plus its exit status when the job carrying it has
# been reaped. SIGKILL means an OOM killer or a supervisor, SIGSEGV a crash; the
# PX4 log separates a killed autopilot from one that closed its logfile itself.
record_px4_exit_status() {
  local role="$1"
  local job_pid="$2"
  local autopilot_pid="${3:-}"
  local watched
  local status=unknown
  local line

  [[ -n "${job_pid}" ]] || return 0
  # What the flight needs to know is whether the autopilot left, not whether the
  # job that carried it did: a killed PX4 can leave the parameter stream beside
  # it running, and that is still a lost autopilot.
  watched="${autopilot_pid:-${job_pid}}"
  local px4_job_exit_file="${runtime_artifact_dir}/px4_job_exit.txt"
  if px4_process_alive "${watched}"; then
    printf 'role=%s pid=%s autopilot_pid=%s phase=teardown\n' \
      "${role}" "${job_pid}" "${watched}" >> "${px4_exit_status_file}"
    return 0
  fi
  if [[ -r "${px4_job_exit_file}" ]]; then
    status="$(sed -n 's/^status=\([0-9][0-9]*\)$/\1/p' "${px4_job_exit_file}" | head -1)"
    [[ -n "${status}" ]] || status=unknown
  elif ! px4_process_alive "${job_pid}"; then
    # Reaped without the file: the status is the shell's, and it is only
    # obtainable once the job has ended.
    status=0
    wait "${job_pid}" 2>/dev/null || status=$?
  fi
  line="$(printf 'role=%s pid=%s autopilot_pid=%s phase=in_flight exit_status=%s observed_at=%s' \
    "${role}" "${job_pid}" "${watched}" "${status}" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)")"
  if [[ "${status}" != "unknown" && "${status}" -gt 128 ]]; then
    line+=" signal=$((status - 128))"
  fi
  printf '%s\n' "${line}" >> "${px4_exit_status_file}"
  printf 'PX4_EXITED_IN_FLIGHT %s\n' "${line}" | tee -a "${px4_log_file}" >&2
}

# Records every vehicle's autopilot, the first through px4_pid.
record_px4_exit_statuses() {
  local instance

  record_px4_exit_status px4 "${px4_pid:-}" "${px4_autopilot_pid:-}"
  if bool_is_true "${multi_vehicle_mission}"; then
    for instance in "${!multi_vehicle_px4_pids[@]}"; do
      [[ "${instance}" -eq 0 ]] && continue
      record_px4_exit_status "${multi_vehicle_ids[instance]}" \
        "${multi_vehicle_px4_pids[instance]}"
    done
  fi
}
