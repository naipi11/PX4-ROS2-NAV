#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_publication_currentness_3d.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/offboard_session_admission.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class ExecutionHorizonCommitKind3D : std::uint8_t {
  kTransition,
  kUnchangedPlan,
  kPendingTransition,
};

enum class ExecutionHorizonCommitStatus3D : std::uint8_t {
  kCommitted,
  kInvalidRequest,
  kAuthorityNotCurrent,
  kVehicleStatusNotAuthoritative,
  kRawWorldNotCurrent,
  kHorizonTimeNotCurrent,
  kRevocationEpochChanged,
  kObjectiveChanged,
  kNavigationNotAuthoritative,
  kOffboardSessionNotCurrent,
  kExecutionInputNotCurrent,
  kExecutionInputNotFresh,
  kEvidenceNotCurrent,
  kSensorEvidenceNotCurrent,
  kControlEvidenceNotCurrent,
  kOwnerInvalid,
  kOwnerPlanIdentityInvalid,
  kLeaseRejected,
};

enum class ExecutionHorizonRevocationRequest3D : std::uint8_t {
  kNone,
  kUnavailableWorld,
  kNoExecutableHorizon,
};

// Adapter-owned runtime facts captured under its input/evidence locks. The
// supervisor evaluates them in the same ordered transaction as plan evidence
// and the final lease compare-and-swap.
struct ExecutionHorizonRuntimeCurrentness3D {
  bool vehicle_status_authoritative{false};
  bool raw_world_identity_conflicted{false};
  double current_raw_age_ms{-1.0};
  double maximum_raw_age_ms{0.0};
  bool revocation_epoch_current{false};
  bool objective_current{false};
  bool navigation_authoritative{false};
  OffboardSessionPublicationCurrentnessStatus offboard_session_currentness{
      OffboardSessionPublicationCurrentnessStatus::kInvalidInput};
};

// Exact navigation/control witness captured by the runtime adapter. No mutable
// node state or ROS message crosses the execution-service boundary.
struct ExecutionHorizonNavigationWitness3D {
  MotionState3D state{};
  MotionControl3D measured_equivalent_control{};
  std::uint64_t pose_revision{0U};
  std::uint64_t source_timestamp_us{0U};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t measured_control_source_sequence{0U};
  std::int64_t measured_control_source_stamp_ns{0};
  std::int64_t measured_control_receive_stamp_ns{0};
  bool measured_acceleration_authoritative{false};
};

// Owned candidate for one controller-visible lease. A progress-only snapshot
// may certify a transition but can never be published independently of the
// exact resident predecessor and composed transition stored here.
struct ExecutionHorizonLeaseCandidate3D {
  ExecutionHorizonCommitKind3D kind{ExecutionHorizonCommitKind3D::kUnchangedPlan};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionPlan3D> expected_plan;
  std::shared_ptr<const ExecutionPlan3D> certification_plan;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> progress_preparation;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  std::shared_ptr<const PendingCertifiedRoute3D> expected_pending;
  ExecutionOwnerIdentity3D owner{};
  std::uint64_t expected_horizon_producer_instance_id{0U};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  bool stationary_capture_rearm_intent{false};
};

struct ExecutionHorizonCommitRequest3D {
  ExecutionHorizonLeaseCandidate3D candidate{};
  ExecutionHorizonRuntimeCurrentness3D runtime{};
  ExecutionHorizonNavigationWitness3D navigation{};
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world;
  std::shared_ptr<const VersionedLatestSensorEvidence3D> current_sensor_evidence;
  std::int64_t publication_now_ns{0};
  double maximum_control_feedback_age_ms{0.0};
  bool latest_sensor_identity_conflicted{false};
};

struct ExecutionHorizonCommitResult3D {
  ExecutionHorizonCommitStatus3D status{
      ExecutionHorizonCommitStatus3D::kInvalidRequest};
  ExecutionHorizonRevocationRequest3D revocation_request{
      ExecutionHorizonRevocationRequest3D::kNone};
  ExecutionPublicationCurrentnessStatus3D publication_currentness{
      ExecutionPublicationCurrentnessStatus3D::kSnapshotMissing};
  ExecutionRoutePublicationStatus3D lease_status{
      ExecutionRoutePublicationStatus3D::kInvalidCandidate};
  bool latest_evidence_revalidated{false};
  // Why the latest-evidence revalidation refused, or nullptr when it did not run
  // or did not fail; the boolean above is derived from it. Half the stops of
  // 2026-11-13 (865 of 1691) followed a refusal here, and the stage name alone
  // could not say whether the published path met new evidence - a stop the world
  // asked for - or the sweep could not be made, which is a tick the next one may
  // publish. `path_meets_occupied_evidence` and `path_refused_mechanically` are
  // the sweep's own two answers; the rest name the clause that stopped it, or the
  // view it could not build.
  const char* evidence_revalidation_failure{nullptr};
  // Which view the refusal was about: "command", "braking_fallback", "stop" or
  // "hold", or nullptr when no view was reached. Plain text because these headers
  // are independent of the controller (test_navigation_dependency_contract.py).
  const char* evidence_revalidation_view{nullptr};
  // Wall time of the latest-evidence revalidation sweep, zero when it did not
  // run, for the tick phase diagnostics.
  double revalidation_ms{0.0};
  bool replaced_applied_control{false};

  [[nodiscard]] bool committed() const noexcept;
};

[[nodiscard]] bool appliedControlAuthoritativeForExecution3D(
    const AppliedControlEvidence3D& control, const ExecutionOwnerIdentity3D& owner,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

// Applied-control evidence usable as the previous control of a new execution
// input: it must come from the offboard process that executes this owner's
// lineage, be fresh, and refer to the owner's own horizon or to its immediate
// predecessor. The predecessor is accepted because the offboard keeps applying
// it until it switches to the newly published horizon; a jerk-limited ramp
// that restarts from the measured acceleration on every tick never leaves
// rest. For a stationary hold owner the offboard's hold feedback of that same
// hold is current: it carries the zero control the hold applies. Unlike the
// witness above it does not require the control stamp inside the owner's
// lease, and it never certifies a supersession.
[[nodiscard]] bool appliedControlCurrentForExecutionInput3D(
    const AppliedControlEvidence3D& control, const ExecutionOwnerIdentity3D& owner,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

// Names the first clause that makes that evidence unusable, or nullptr when it is
// usable; the bool above is derived from it. A stop needs this evidence, and on
// 2026-11-13 seven stops in a row were refused here with nothing recorded to say
// which clause failed.
[[nodiscard]] const char* appliedControlCurrentForExecutionInputFailure3D(
    const AppliedControlEvidence3D& control, const ExecutionOwnerIdentity3D& owner,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

// The same for the previous-control evidence a new execution input carries: the
// offboard feedback it names, or the measured acceleration, must still describe
// the lease being executed. A stop needs this evidence, and on 2026-11-13 seven
// stops in a row were refused because of it with nothing recorded to say which
// clause failed.
[[nodiscard]] const char* previousControlEvidenceFailure3D(
    const VersionedExecutionInput3D& input,
    const ExecutionHorizonNavigationWitness3D& navigation,
    const AppliedControlEvidence3D& resident_control,
    const ExecutionOwnerIdentity3D& resident_owner, std::int64_t now_ns,
    double maximum_control_feedback_age_ms, bool stationary_capture_rearm) noexcept;

// Why the latest-evidence revalidation would refuse the execution this plan
// published, or nullptr when it can be made against that evidence. The commit's
// `latest_evidence_revalidated` is this being nullptr; a caller that has to tell a
// stop the world asked for from a tick that could not be revalidated reads it here.
[[nodiscard]] const char* evidenceRevalidationFailure3D(
    const ExecutionPlan3D& plan,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestSensorEvidence3D>&
        latest_sensor) noexcept;

// Names the first predicate that makes the applied-control evidence
// non-authoritative for this owner, or returns nullptr when it is
// authoritative. Diagnostics use it so a deferred supersession can say why.
[[nodiscard]] const char*
appliedControlAuthorityFailure3D(const AppliedControlEvidence3D& control,
                                 const ExecutionOwnerIdentity3D& owner,
                                 std::int64_t now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] const char*
executionHorizonCommitStatus3DName(ExecutionHorizonCommitStatus3D status) noexcept;

} // namespace drone_city_nav
