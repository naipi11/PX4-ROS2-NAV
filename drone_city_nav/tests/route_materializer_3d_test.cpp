#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "production_mppi_route_world.hpp"
#include "route_materializer_3d.hpp"
#include "route_trajectory_compiler_3d.hpp"

namespace drone_city_nav {
namespace {

struct MaterializerFixture3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
};

[[nodiscard]] RoutePlannerConfig3D plannerConfig() {
  RoutePlannerConfig3D config;
  config.planner.minimum_horizontal_step_m = 1.0;
  config.planner.minimum_vertical_step_m = 1.0;
  config.planner.maximum_adaptive_lattice_level = 0U;
  config.planner.time_model.maximum_horizontal_speed_mps = 5.0;
  config.planner.time_model.maximum_vertical_speed_mps = 2.0;
  config.planner.goal_tolerance_m = 0.01;
  config.planner.feasibility_first_enabled = false;
  config.planner.maximum_compute_time_ms = 1000.0;
  config.planner.maximum_no_route_compute_time_ms = 1000.0;
  config.planner.maximum_expansions_per_update = 100000U;
  config.planner.physical_footprint.radius_m = 0.0;
  config.planner.physical_footprint.lower_extent_m = 0.0;
  config.planner.physical_footprint.upper_extent_m = 0.0;
  config.planner.physical_footprint.perimeter_samples = 0U;
  config.planner.physical_footprint.radial_rings = 0U;
  config.planner.physical_footprint.axial_samples = 0U;
  config.planner.physical_footprint.sweep_step_m = 0.2;
  config.planner.flight_envelope.minimum_target_z_m = 0.0;
  config.planner.flight_envelope.maximum_target_z_m = 20.0;
  config.route_sampling_step_m = 0.5;
  config.cruise_speed_mps = 5.0;
  return config;
}

[[nodiscard]] RouteMaterializerConfig3D materializerConfig() {
  RouteMaterializerConfig3D config;
  config.route_geometry.enabled = false;
  config.physical_footprint.radius_m = 0.0;
  config.physical_footprint.lower_extent_m = 0.0;
  config.physical_footprint.upper_extent_m = 0.0;
  config.physical_footprint.perimeter_samples = 0U;
  config.physical_footprint.radial_rings = 0U;
  config.physical_footprint.axial_samples = 0U;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  config.critical_distance_m = 1.5;
  config.preferred_distance_m = 4.0;
  config.worker_pool = nullptr;
  config.cooperative_traffic_enabled = false;
  return config;
}

[[nodiscard]] MaterializerFixture3D
fixture(const StaticRouteSearchRequestIdentity request =
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kInitial,
                .base_route_generation = 0U,
            },
        const RouteReleaseReason3D release_reason = RouteReleaseReason3D::kNone,
        const bool invalid_derived_distances = false,
        const bool partition_goal = false) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 8};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 71U);
  if (partition_goal) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int z = 0; z < bounds.depth_cells; ++z) {
        occupancy->setOccupied({8, y, z});
      }
    }
  }
  const std::uint64_t revision = occupancy->fingerprint();
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = occupied_fingerprint;
  world->raw_occupied_fingerprint = occupied_fingerprint;
  world->grid = EsdfGrid3D{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
      .outside_is_unknown = true,
  };
  const std::size_t voxel_count = static_cast<std::size_t>(bounds.width_cells) *
                                  static_cast<std::size_t>(bounds.height_cells) *
                                  static_cast<std::size_t>(bounds.depth_cells);
  const float distance_value =
      invalid_derived_distances ? std::numeric_limits<float>::quiet_NaN() : 20.0F;
  world->distances_m =
      std::make_shared<const std::vector<float>>(voxel_count, distance_value);
  world->static_occupancy = occupancy;
  world->local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map = {.producer_instance_id = 0U,
                  .base_snapshot_revision = revision,
                  .revision = revision},
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  std::shared_ptr<const WorldSnapshot3D> immutable_world = std::move(world);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(immutable_world, planner_world,
                                     StaticRouteObjective{
                                         .goal = {12.5, 12.5, 4.5},
                                         .mission_epoch = 7U,
                                         .available = true,
                                     },
                                     request, std::nullopt, release_reason);
  return MaterializerFixture3D{
      .world = std::move(immutable_world),
      .planner_world = planner_world,
      .transaction = transaction,
  };
}

[[nodiscard]] RoutePlannerVehicleState3D vehicleState() {
  return RoutePlannerVehicleState3D{
      .position = Point3{1.5, 1.5, 1.5},
      .velocity = Vec3{0.0, 0.0, 0.0},
      .valid = true,
  };
}

[[nodiscard]] RouteSearchCandidate3D
plannerCandidate(const std::shared_ptr<const PlannerSearchTransaction3D>& transaction) {
  if (transaction == nullptr) {
    throw std::invalid_argument{"planner transaction is unavailable"};
  }
  RoutePlanner3D planner{plannerConfig()};
  RoutePlannerUpdate3D update = planner.update(*transaction, vehicleState());
  if (!update.improved_incumbent.has_value()) {
    throw std::runtime_error{"planner did not produce a materializer fixture"};
  }
  return std::move(update.improved_incumbent).value_or(RouteSearchCandidate3D{});
}

[[nodiscard]] RouteMaterializationRequest3D
requestFor(const MaterializerFixture3D& input, RouteSearchCandidate3D candidate,
           const std::uint64_t candidate_generation = 8U) {
  return RouteMaterializationRequest3D{
      .transaction = input.transaction,
      .world_telemetry = {.build_ms = 12.5},
      .current_position = vehicleState().position,
      .candidate = std::move(candidate),
      .candidate_generation = candidate_generation,
      .active_route = nullptr,
      .activation_raw_world = nullptr,
  };
}

[[nodiscard]] PassageTraversalEdge
traversalFor(const RouteSearchCandidate3D& candidate) {
  const std::vector<RouteSample3D>& centerline = candidate.route;
  if (centerline.size() < 2U) {
    throw std::invalid_argument{"candidate route cannot define a traversal"};
  }
  return PassageTraversalEdge{
      .id = PassageTraversalId{"candidate-traversal"},
      .region_id = FreeSpaceRegionId{"candidate-region"},
      .entry_portal_id = PassagePortalId{"candidate-entry"},
      .exit_portal_id = PassagePortalId{"candidate-exit"},
      .centerline = centerline,
      .entry = centerline.front().position,
      .exit = centerline.back().position,
      .min_z_m = 0.0,
      .max_z_m = 20.0,
      .width_m = 6.0,
      .height_m = 20.0,
      .minimum_clearance_m = 3.0,
      .speed_limit_mps = 3.0,
      .segment_spans = {},
  };
}

[[nodiscard]] MaterializerFixture3D
withTopology(const MaterializerFixture3D& input,
             const PassageTraversalEdge& traversal) {
  if (input.world == nullptr || input.transaction == nullptr) {
    throw std::invalid_argument{"materializer fixture is unavailable"};
  }
  WorldSnapshot3D world = *input.world;
  world.topology_passage_traversals =
      std::make_shared<const std::vector<PassageTraversalEdge>>(1U, traversal);
  auto immutable_world = std::make_shared<const WorldSnapshot3D>(std::move(world));
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*immutable_world);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(
          immutable_world, planner_world, input.transaction->objective,
          input.transaction->request, input.transaction->continuity_base,
          input.transaction->release_reason);
  return MaterializerFixture3D{
      .world = std::move(immutable_world),
      .planner_world = planner_world,
      .transaction = transaction,
  };
}

TEST(RouteMaterializer3DTest, RejectsInvalidConfigurationAndRequest) {
  RouteMaterializerConfig3D invalid = materializerConfig();
  invalid.preferred_distance_m = invalid.critical_distance_m - 1.0;
  EXPECT_THROW(static_cast<void>(RouteMaterializer3D{invalid}), std::invalid_argument);

  RouteMaterializer3D materializer{materializerConfig()};
  const ProductionRouteMaterialization3D result = materializer.materialize({});
  EXPECT_EQ(result.validation.status, StaticRouteCandidateStatus::kInvalidInput);
  EXPECT_FALSE(result.validation.accepted);
  EXPECT_EQ(result.route.route, nullptr);
}

TEST(RouteMaterializer3DTest,
     MaterializesOneImmutablePlannerCandidateWithoutNodeState) {
  const MaterializerFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  RouteMaterializer3D materializer{materializerConfig()};

  RouteSearchCandidate3D candidate = plannerCandidate(input.transaction);
  ASSERT_TRUE(candidate.spatial_route.valid());
  ASSERT_TRUE(spatialRouteCandidateReachesGoal3D(candidate.spatial_route.source));
  EXPECT_TRUE(candidate.evidence.planner_executable);
  EXPECT_TRUE(candidate.evidence.reaches_mission_target);
  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, std::move(candidate)));

  EXPECT_TRUE(result.validation.accepted);
  EXPECT_EQ(result.route.world, input.world);
  EXPECT_EQ(result.route.objective.mission_epoch, 7U);
  EXPECT_EQ(result.route.candidate_generation, 8U);
  ASSERT_NE(result.route.route, nullptr);
  EXPECT_GE(result.route.route->size(), 2U);
  EXPECT_NE(result.route.fingerprint, 0U);
  EXPECT_TRUE(result.route.initial_projection.valid);
  EXPECT_TRUE(result.telemetry.planner.invoked);
  EXPECT_DOUBLE_EQ(result.telemetry.world_build.build_ms, 12.5);
  EXPECT_FALSE(result.geometry_optimization_fallback.has_value());
  EXPECT_TRUE(result.telemetry.planner.executable);
  EXPECT_TRUE(result.route.planner_executable);
  EXPECT_TRUE(result.route.reaches_mission_goal);
  EXPECT_TRUE(result.route.segment_evidence.reaches_mission_target);
  EXPECT_EQ(routeEndpointSemantics3D(result.route.reaches_mission_goal, true),
            RouteEndpointSemantics3D::kMissionStop);
  EXPECT_EQ(routeEndpointSemantics3D(result.route.reaches_mission_goal, false),
            RouteEndpointSemantics3D::kContinuation);
  const StaticRouteExtensionDecision extension = evaluateStaticRouteExtension(
      materializerConfig().route_extension,
      StaticRouteExtensionObservation{
          .route_generation = result.route.candidate_generation,
          .route_remaining_m = 0.0,
          .route_reaches_mission_goal = result.route.reaches_mission_goal,
      });
  ASSERT_TRUE(extension.valid);
  EXPECT_FALSE(extension.request_extension);

  MissionGoalCaptureLatch capture{MissionGoalCaptureConfig{}};
  mppi::State state{};
  state.x = static_cast<float>(vehicleState().position.x);
  state.y = static_cast<float>(vehicleState().position.y);
  state.z = static_cast<float>(vehicleState().position.z);
  EXPECT_FALSE(capture
                   .update(MissionGoalCaptureObservation{
                       .mission_goal = result.route.objective.goal,
                       .state = state,
                       .terminal_route_available = result.route.reaches_mission_goal,
                   })
                   .latched);
  state.x = static_cast<float>(result.route.objective.goal.x);
  state.y = static_cast<float>(result.route.objective.goal.y);
  state.z = static_cast<float>(result.route.objective.goal.z);
  EXPECT_TRUE(capture
                  .update(MissionGoalCaptureObservation{
                      .mission_goal = result.route.objective.goal,
                      .state = state,
                      .terminal_route_available = result.route.reaches_mission_goal,
                  })
                  .newly_latched);
}

TEST(RouteMaterializer3DTest,
     ClosestApproachStaysExecutableWithoutClaimingMissionGoalReach) {
  // The wall closes the lattice completely. Drive the real persistent planner
  // until it offers the raw-valid closest approach, without changing its source.
  const MaterializerFixture3D input = fixture(
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kInitial,
      },
      RouteReleaseReason3D::kNone, false, true);
  ASSERT_NE(input.transaction, nullptr);
  RoutePlannerConfig3D config = plannerConfig();
  config.planner.feasibility_first_enabled = true;
  config.planner.maximum_feasibility_compute_time_ms = 500.0;
  config.planner.maximum_feasibility_expansions_per_update = 100000U;
  config.planner.escape_search_maximum_probes_per_update = 1U << 16U;
  RoutePlanner3D planner{config};
  RoutePlannerUpdate3D update = planner.update(*input.transaction, vehicleState());
  ASSERT_EQ(update.planner_input_status, PlannerInputStatus3D::kAccepted);
  for (int attempt = 0; attempt < 120 && !update.improved_incumbent; ++attempt) {
    update = planner.update(*input.transaction, vehicleState(), update.planner_session);
  }
  ASSERT_TRUE(update.improved_incumbent.has_value())
      << "frontier_exhausted="
      << update.planner_telemetry.feasibility_frontier_exhausted
      << " escape_exhausted=" << update.planner_telemetry.escape_search_exhausted;
  RouteSearchCandidate3D candidate = std::move(*update.improved_incumbent);
  ASSERT_TRUE(candidate.spatial_route.valid());
  ASSERT_EQ(candidate.spatial_route.source,
            SpatialRouteCandidateSource3D::kFeasibilityClosestApproach);
  ASSERT_FALSE(spatialRouteCandidateReachesGoal3D(candidate.spatial_route.source));
  ASSERT_LT(candidate.spatial_route.points.back().x, 8.0);
  EXPECT_TRUE(candidate.evidence.planner_executable);
  EXPECT_TRUE(candidate.evidence.physical_executable);
  EXPECT_FALSE(candidate.evidence.reaches_mission_target);

  const RouteMaterializerConfig3D materializer_config = materializerConfig();
  RouteMaterializer3D materializer{materializer_config};
  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, candidate));
  ASSERT_TRUE(result.validation.accepted);
  ASSERT_NE(result.route.route, nullptr);
  EXPECT_TRUE(result.telemetry.planner.executable);
  EXPECT_TRUE(result.route.planner_executable);
  EXPECT_FALSE(result.route.reaches_mission_goal);
  EXPECT_FALSE(result.route.segment_evidence.reaches_mission_target);
  const RouteEndpointSemantics3D semantics =
      routeEndpointSemantics3D(result.route.reaches_mission_goal, true);
  EXPECT_EQ(semantics, RouteEndpointSemantics3D::kContinuation);
  EXPECT_FALSE(routeEndpointHasTerminalStop3D(semantics));
  EXPECT_TRUE(routeEndpointUsesLocalBoundary3D(semantics));
  const StaticRouteExtensionDecision extension = evaluateStaticRouteExtension(
      materializer_config.route_extension,
      StaticRouteExtensionObservation{
          .route_generation = result.route.candidate_generation,
          .route_remaining_m = 0.0,
          .route_reaches_mission_goal = result.route.reaches_mission_goal,
      });
  ASSERT_TRUE(extension.valid);
  EXPECT_TRUE(extension.request_extension);

  // Even a stationary estimate at the goal cannot turn a partial-route claim
  // into a terminal-route witness for the capture latch.
  MissionGoalCaptureLatch capture{MissionGoalCaptureConfig{}};
  mppi::State state{};
  state.x = static_cast<float>(result.route.objective.goal.x);
  state.y = static_cast<float>(result.route.objective.goal.y);
  state.z = static_cast<float>(result.route.objective.goal.z);
  EXPECT_FALSE(capture
                   .update(MissionGoalCaptureObservation{
                       .mission_goal = result.route.objective.goal,
                       .state = state,
                       .terminal_route_available = result.route.reaches_mission_goal,
                   })
                   .latched);

  // The same partial endpoint is not an improvement over resident geometry.
  // This comparison needs geometry only, not an activation/certification claim.
  RouteTrajectoryCompilerConfig3D compiler_config;
  compiler_config.trajectory.physical_footprint =
      materializer_config.physical_footprint;
  compiler_config.trajectory.physical_footprint.perimeter_samples = 8U;
  compiler_config.trajectory.physical_footprint.radial_rings = 1U;
  compiler_config.trajectory.physical_footprint.axial_samples = 1U;
  RouteTrajectoryCompiler3D compiler{compiler_config};
  const RouteTrajectoryCompilationResult3D compiled =
      compiler.compile(RouteTrajectoryCompilationRequest3D{
          .materialized = result.route,
          .exact_initial_state =
              VehicleState3D{
                  .identity = {.revision = 5U,
                               .source_timestamp_us = 7U,
                               .receive_stamp_ns = 11},
                  .position = vehicleState().position,
              },
          .endpoint_semantics = semantics,
          .observed_raw_world = nullptr,
      });
  ASSERT_TRUE(compiled.compiled())
      << compiledTrajectoryFailureReason3DName(compiled.validation.reason);
  auto active = std::make_shared<CertifiedRouteSuffix3D>();
  active->geometry = compiled.trajectory;
  RouteMaterializationRequest3D repeated = requestFor(input, std::move(candidate), 9U);
  repeated.active_route = std::move(active);
  const ProductionRouteMaterialization3D unchanged =
      materializer.materialize(std::move(repeated));
  EXPECT_EQ(unchanged.validation.status,
            StaticRouteCandidateStatus::kNoEndpointImprovement);
  EXPECT_FALSE(unchanged.validation.accepted);
}

TEST(RouteMaterializer3DTest,
     InvalidDerivedDistanceEvidenceRemainsNeutralForRawSafeRoute) {
  const MaterializerFixture3D input = fixture(
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kInitial,
          .base_route_generation = 0U,
      },
      RouteReleaseReason3D::kNone, true);
  ASSERT_NE(input.transaction, nullptr);
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, plannerCandidate(input.transaction)));

  ASSERT_TRUE(result.validation.accepted);
  ASSERT_NE(result.route.route, nullptr);
  for (const RouteSample3D& sample : *result.route.route) {
    EXPECT_EQ(sample.required_risk_tier, RouteRiskTier3D::kPreferred);
  }
}

TEST(RouteMaterializer3DTest,
     MaterializedRouteRetainsAndAssociatesStaticTopologyTraversal) {
  const MaterializerFixture3D base = fixture();
  ASSERT_NE(base.transaction, nullptr);
  RouteSearchCandidate3D candidate = plannerCandidate(base.transaction);
  const MaterializerFixture3D input = withTopology(base, traversalFor(candidate));
  ASSERT_NE(input.transaction, nullptr);
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, std::move(candidate)));

  ASSERT_TRUE(result.validation.accepted);
  EXPECT_EQ(result.route.world, input.world);
  ASSERT_NE(result.route.selected_passage_traversal_ids, nullptr);
  ASSERT_EQ(result.route.selected_passage_traversal_ids->size(), 1U);
  EXPECT_EQ(result.route.selected_passage_traversal_ids->front(),
            PassageTraversalId{"candidate-traversal"});
  ASSERT_NE(result.route.constrained_spans, nullptr);
  ASSERT_EQ(result.route.constrained_spans->size(), 1U);
  EXPECT_EQ(result.route.constrained_spans->front().passage_traversal_id,
            PassageTraversalId{"candidate-traversal"});
}

TEST(RouteMaterializer3DTest, OverlapCandidateRequiresTheExactCertifiedActiveRoute) {
  const MaterializerFixture3D input = fixture(
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kReplan,
          .base_route_generation = 4U,
      },
      RouteReleaseReason3D::kBlocked);
  ASSERT_NE(input.transaction, nullptr);
  RouteSearchCandidate3D candidate = plannerCandidate(input.transaction);
  candidate.search_base_route_instance_id = RouteInstanceId3D{.value = 91U};
  candidate.search_base_stitch_station_m = 3.0;
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, std::move(candidate)));

  EXPECT_EQ(result.validation.status, StaticRouteCandidateStatus::kInvalidPassageSpan);
  EXPECT_FALSE(result.validation.accepted);
  EXPECT_EQ(result.replacement_policy,
            StaticRouteReplacementPolicy::kAllowSafetyReplan);
}

} // namespace
} // namespace drone_city_nav
