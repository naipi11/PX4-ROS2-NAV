# Offboard's setpoint-publication witness, not a PX4 acknowledgement or measured
# actuator/vehicle response. Acceleration is in header.frame_id (normally map).
# The offboard producer, horizon producer, sequence, and header stamp identify
# the exact emitted feedback. Zero horizon identity with non-authoritative hold
# is a liveness heartbeat for the offboard process, not horizon revocation.
# Yaw rate is the lead-sampled command sent to PX4. Yaw acceleration is sampled
# at the trajectory time without that lead and seeds MPPI control continuity;
# it is neither a PX4 setpoint field nor a measured angular acceleration.
std_msgs/Header header
uint8 EXECUTION_MODE_PLANNED=0
uint8 EXECUTION_MODE_POSITION_HOLD=1
uint64 producer_instance_id
uint64 horizon_producer_instance_id
uint64 horizon_sequence
uint8 execution_mode
bool control_authoritative
geometry_msgs/Vector3 acceleration
float32 yaw_rate_radps
float32 yaw_acceleration_radps2
