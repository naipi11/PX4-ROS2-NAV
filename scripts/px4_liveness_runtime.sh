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
        px4_parameter_stream "${multi_vehicle_px4_cruise_speeds[instance]}" \
          "${multi_vehicle_px4_maximum_speeds[instance]}" |
          PX4_GZ_WORLD="${world_name}" \
            PX4_GZ_STANDALONE=1 \
            PX4_GZ_MODEL_POSE="${multi_vehicle_gazebo_spawn_poses[instance]}" \
            PX4_SIM_MODEL="${multi_vehicle_px4_model_targets[instance]}" \
            PX4_UXRCE_DDS_NS="${multi_vehicle_px4_namespaces[instance]}" \
            PX4_SYS_AUTOSTART=4001 \
            HEADLESS="${headless}" \
            run_px4_instance "${instance}"
      ) > "${multi_vehicle_px4_logs[instance]}" 2>&1 &
      multi_vehicle_px4_pids+=("$!")
    done
    px4_pid="${multi_vehicle_px4_pids[0]}"
  else
    echo "PX4 Gazebo spawn pose: ${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}"
    reset_px4_instance_state 0
    (
      px4_parameter_stream "${px4_active_cruise_speed_mps}" \
        "${px4_active_max_horizontal_speed_mps}" |
        PX4_GZ_WORLD="${world_name}" \
        PX4_GZ_STANDALONE=1 \
        PX4_GZ_MODEL_POSE="${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}" \
          PX4_SIM_MODEL="${px4_model_target}" \
          PX4_SYS_AUTOSTART=4001 \
          HEADLESS="${headless}" \
          run_px4_sitl
    ) > "${px4_log_file}" 2>&1 &
    px4_pid=$!
  fi
}

# Waits out the startup delay, refuses a PX4 that has already gone, and names
# the autopilot process inside the job. The autopilot is neither the job itself
# nor its first child: `( params | env ... px4 )` puts the parameter stream
# beside it, so the first child is that stream and killing it leaves PX4 flying.
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
  if px4_process_alive "${watched}"; then
    printf 'role=%s pid=%s autopilot_pid=%s phase=teardown\n' \
      "${role}" "${job_pid}" "${watched}" >> "${px4_exit_status_file}"
    return 0
  fi
  if ! px4_process_alive "${job_pid}"; then
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
