#include "drone_city_nav/executed_horizon_clearance_3d.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

// A 1 m lattice, three cells wide and deep, twenty cells long: the first ten
// cells along x are observed free, far from anything; the rest are unknown.
constexpr int kLength{20};
constexpr int kKnownLength{10};

[[nodiscard]] EsdfGrid3D grid() {
  return EsdfGrid3D{.width = kLength,
                    .height = 3,
                    .resolution_m = 1.0F,
                    .origin_x_m = 0.0F,
                    .origin_y_m = 0.0F,
                    .depth = 3,
                    .origin_z_m = 0.0F,
                    .outside_is_unknown = true};
}

[[nodiscard]] std::vector<float> esdf() {
  std::vector<float> field(static_cast<std::size_t>(kLength) * 3U * 3U, 5.0F);
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      for (int x = kKnownLength; x < kLength; ++x) {
        field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(kLength) +
              static_cast<std::size_t>(x)] = kUnknownEsdfDistanceM;
      }
    }
  }
  return field;
}

[[nodiscard]] WorldSnapshot3D clearanceWorld(const EsdfGrid3D& lattice,
                                             std::vector<float> distances) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(GridBounds3D{
      .origin_x = lattice.origin_x_m,
      .origin_y = lattice.origin_y_m,
      .origin_z = lattice.origin_z_m,
      .resolution_m = lattice.resolution_m,
      .width_cells = lattice.width,
      .height_cells = lattice.height,
      .depth_cells = lattice.depth,
  });
  for (int z = 0; z < lattice.depth; ++z) {
    for (int y = 0; y < lattice.height; ++y) {
      for (int x = 0; x < lattice.width; ++x) {
        const auto index =
            (static_cast<std::size_t>(z) * static_cast<std::size_t>(lattice.height) +
             static_cast<std::size_t>(y)) *
                static_cast<std::size_t>(lattice.width) +
            static_cast<std::size_t>(x);
        if (distances[index] >= 0.0F) {
          occupancy->setState({x, y, z}, distances[index] == 0.0F
                                             ? ObservedVoxelState::kOccupied
                                             : ObservedVoxelState::kFree);
        }
      }
    }
  }
  WorldSnapshot3D world;
  world.grid = lattice;
  world.distances_m = std::make_shared<const std::vector<float>>(std::move(distances));
  world.observed_occupancy = std::move(occupancy);
  return world;
}

[[nodiscard]] SweptFootprintConfig pointFootprint() {
  SweptFootprintConfig footprint;
  footprint.radius_m = 0.0;
  footprint.lower_extent_m = 0.0;
  footprint.upper_extent_m = 0.0;
  footprint.perimeter_samples = 0U;
  footprint.radial_rings = 0U;
  footprint.axial_samples = 0U;
  footprint.sweep_step_m = 0.5;
  return footprint;
}

[[nodiscard]] FiniteMotionHorizon3D horizonAlongX(const float first_x,
                                                  const std::size_t states) {
  FiniteMotionHorizon3D horizon;
  for (std::size_t index = 0U; index < states; ++index) {
    MotionState3D state;
    state.x = first_x + static_cast<float>(index);
    state.y = 1.5F;
    state.z = 1.5F;
    horizon.states.push_back(state);
  }
  return horizon;
}

TEST(ExecutedHorizonClearance3DTest, TheFirstUnobservedSampleEndsTheObservedRange) {
  // From x = 0.5 the motion runs one metre per state. The segment leaving
  // x = 9.5 sweeps into the unknown cells from x = 10: the observed range
  // along the motion is the nine metres travelled before that segment.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 0U, clearanceWorld(grid(), esdf()), pointFootprint(),
      0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_TRUE(clearance.unobserved());
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 9.0, 1.0e-6);
  // Unknown space is not a clearance constraint.
  EXPECT_FALSE(clearance.constrained());
  EXPECT_TRUE(std::isfinite(clearance.minimum_clearance_m));
}

TEST(ExecutedHorizonClearance3DTest, AMotionThroughObservedSpaceHasNoFrontier) {
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 8U), 0U, clearanceWorld(grid(), esdf()), pointFootprint(),
      0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_FALSE(clearance.unobserved());
  EXPECT_TRUE(std::isinf(clearance.distanceToUnobservedM()));
}

TEST(ExecutedHorizonClearance3DTest, TheRangeIsMeasuredFromTheRemainingMotion) {
  // Execution has advanced four states: the observed range is what remains.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 4U, clearanceWorld(grid(), esdf()), pointFootprint(),
      0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 5.0, 1.0e-6);
}

[[nodiscard]] std::vector<RouteSample3D> routeAlongX(const double first_x,
                                                     const std::size_t samples) {
  std::vector<RouteSample3D> route;
  for (std::size_t index = 0U; index < samples; ++index) {
    route.push_back(RouteSample3D{
        .position = {first_x + static_cast<double>(index), 1.5, 1.5},
        .tangent = {1.0, 0.0, 0.0},
        .station_m = static_cast<double>(index),
    });
  }
  return route;
}

TEST(ExecutedHorizonClearance3DTest, TheRouteAheadReportsTheFrontierBeyondTheHorizon) {
  // The route runs from x = 0.5 one metre per sample; the vehicle projects at
  // station 2. The segment leaving x = 9.5 (station 9) sweeps into the unknown
  // cells from x = 10: the observed range along the route is seven metres.
  const std::optional<double> range_m = measureRouteObservedRange3D(
      routeAlongX(0.5, 20U), 2.0, 30.0, grid(), esdf(), pointFootprint());
  ASSERT_TRUE(range_m.has_value());
  EXPECT_NEAR(*range_m, 7.0, 1.0e-6);
}

TEST(ExecutedHorizonClearance3DTest, TheRouteIsProbedNoFartherThanTheLookahead) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 5.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
  EXPECT_TRUE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 8.0, grid(),
                                          esdf(), pointFootprint())
                  .has_value());
}

TEST(ExecutedHorizonClearance3DTest, ARouteThroughObservedSpaceHasNoFrontier) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 8U), 0.0, 30.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
}

[[nodiscard]] std::vector<float> esdfWithTightColumn(const int column,
                                                     const float clearance_m) {
  std::vector<float> field = esdf();
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                static_cast<std::size_t>(kLength) +
            static_cast<std::size_t>(column)] = clearance_m;
    }
  }
  return field;
}

TEST(ExecutedHorizonClearance3DTest, TheRouteAheadReportsWhereItRunsClose) {
  // The route runs from x = 0.5 one metre per sample and the vehicle projects
  // at station 2. Everything keeps 5 m but the column at x = 6, which keeps 3:
  // the segment that leaves station 5 sweeps through it, three metres ahead of
  // the projection, and the conservative query answers for less than the cell
  // distance because the sweep samples are not on its centre.
  const ExecutedHorizonClearance3D clearance = measureRouteClearance3D(
      routeAlongX(0.5, 20U), 2.0, 30.0,
      clearanceWorld(grid(), esdfWithTightColumn(6, 3.0F)), pointFootprint(), 2.0);
  ASSERT_TRUE(clearance.available);
  ASSERT_TRUE(clearance.constrained());
  EXPECT_NEAR(clearance.distanceToConstraintM(), 3.0, 1.0e-6);
  EXPECT_GT(clearance.constrainedClearanceM(), 0.0);
  EXPECT_LT(clearance.constrainedClearanceM(), 2.0);
}

TEST(ExecutedHorizonClearance3DTest, TheBodyMarginIncludesVerticalErrorDuringTheSweep) {
  const EsdfGrid3D lattice{.width = 8,
                           .height = 3,
                           .resolution_m = 0.25F,
                           .origin_x_m = 0.0F,
                           .origin_y_m = 0.0F,
                           .depth = 12,
                           .origin_z_m = 0.0F,
                           .outside_is_unknown = true};
  SweptFootprintConfig footprint = pointFootprint();
  footprint.body_radius_m = 0.0;
  footprint.body_lower_extent_m = 0.125;
  footprint.body_upper_extent_m = 0.25;
  std::vector<RouteSample3D> route;
  route.reserve(3U);
  for (int index = 0; index < 3; ++index) {
    route.push_back(RouteSample3D{.position = {0.5 + 0.5 * index, 0.375, 1.125},
                                  .tangent = {1.0, 0.0, 0.0},
                                  .station_m = 0.5 * index});
  }
  for (const int occupied_z : {0, 10}) {
    std::vector<float> field(std::size_t{8U} * 3U * 12U);
    for (std::size_t index = 0U; index < field.size(); ++index) {
      const int z = static_cast<int>(index / (std::size_t{8U} * 3U));
      field[index] = 0.25F * static_cast<float>(std::abs(z - occupied_z));
    }
    const ExecutedHorizonClearance3D clearance = measureRouteClearance3D(
        route, 0.0, 1.0, clearanceWorld(lattice, std::move(field)), footprint, 2.0);
    ASSERT_TRUE(clearance.constrained());
    // Floor top 0.25, or ceiling bottom 2.5. Include the physical axial extent
    // and the full half-metre error toward that surface, not a radial inflation.
    const double expected_margin_m = occupied_z == 0 ? 0.25 : 0.625;
    for (const ConstrainedHorizonSample3D& sample : clearance.constrained_samples) {
      EXPECT_NEAR(sample.body_clearance_m, expected_margin_m, 1.0e-9);
    }
  }
}

TEST(ExecutedHorizonClearance3DTest, ParallelWallMotionDoesNotBecomeAnApproach) {
  const EsdfGrid3D lattice{.width = 8,
                           .height = 8,
                           .resolution_m = 0.25F,
                           .origin_x_m = 0.0F,
                           .origin_y_m = 0.0F,
                           .depth = 8,
                           .origin_z_m = 0.0F,
                           .outside_is_unknown = true};
  // Occupied voxels fill x = 0. The field stores the distance to their centres.
  std::vector<float> field(std::size_t{8U} * 8U * 8U);
  for (std::size_t index = 0U; index < field.size(); ++index) {
    field[index] = 0.25F * static_cast<float>(index % 8U);
  }
  const WorldSnapshot3D world = clearanceWorld(lattice, std::move(field));
  SweptFootprintConfig footprint = pointFootprint();
  footprint.body_radius_m = 0.0;
  footprint.body_lower_extent_m = 0.0;
  footprint.body_upper_extent_m = 0.0;
  FiniteMotionHorizon3D horizon;
  horizon.states.reserve(4U);
  for (const float y : {0.365F, 0.385F, 0.405F, 0.425F}) {
    MotionState3D state;
    state.x = 0.375F;
    state.y = y;
    state.z = 1.125F;
    horizon.states.push_back(state);
  }
  MppiSpeedPolicyConfig config;
  config.clearance_response_time_s = 0.075;
  config.clearance_minimum_progress_speed_mps = 3.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  config.reference_tracking_lag_s = 0.3;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.executed_horizon_clearance =
      measureExecutedHorizonClearance3D(horizon, 0U, world, footprint, 0.4875);
  ASSERT_TRUE(input.executed_horizon_clearance->constrained());

  // Every body pose stays 0.125 m from the wall, including the vertical-error
  // probes. Moving across a voxel must not turn that into an approach to contact.
  EXPECT_GE(evaluateMppiSpeedPolicy(config, input).clearance_limit_mps, 1.0);

  // A real approach to the wall still loses the departure floor.
  horizon.states[1U].x = 0.305F;
  horizon.states[2U].x = 0.275F;
  horizon.states[3U].x = 0.255F;
  input.executed_horizon_clearance =
      measureExecutedHorizonClearance3D(horizon, 0U, world, footprint, 0.4875);
  EXPECT_LT(evaluateMppiSpeedPolicy(config, input).clearance_limit_mps, 1.0);

  // The physical cylinder, not the wider validation envelope, owns this margin.
  footprint.radius_m = 0.2;
  footprint.body_radius_m = 0.1;
  for (MotionState3D& state : horizon.states) {
    state.x = 0.375F;
  }
  const ExecutedHorizonClearance3D body =
      measureExecutedHorizonClearance3D(horizon, 0U, world, footprint, 0.4875);
  ASSERT_TRUE(body.constrained());
  for (const ConstrainedHorizonSample3D& sample : body.constrained_samples) {
    EXPECT_NEAR(sample.body_clearance_m, 0.025, 1.0e-9);
  }
}

TEST(ExecutedHorizonClearance3DTest, TheBodyMarginIncludesContactBetweenStates) {
  const EsdfGrid3D lattice{.width = 8,
                           .height = 8,
                           .resolution_m = 0.25F,
                           .origin_x_m = 0.0F,
                           .origin_y_m = 0.0F,
                           .depth = 8,
                           .origin_z_m = 0.0F,
                           .outside_is_unknown = true};
  std::vector<float> field(std::size_t{8U} * 8U * 8U, 1.0F);
  field[(4U * 8U + 4U) * 8U + 4U] = 0.0F;
  SweptFootprintConfig footprint = pointFootprint();
  footprint.body_radius_m = 0.0;
  footprint.body_lower_extent_m = 0.0;
  footprint.body_upper_extent_m = 0.0;
  FiniteMotionHorizon3D horizon;
  horizon.states = {
      MotionState3D{.x = 0.375F, .y = 1.125F, .z = 1.125F},
      MotionState3D{.x = 1.875F, .y = 1.125F, .z = 1.125F},
  };
  for (const bool use_static : {false, true}) {
    WorldSnapshot3D world = clearanceWorld(lattice, field);
    if (use_static) {
      auto occupancy =
          std::make_shared<OccupancyGrid3D>(world.observed_occupancy->bounds());
      occupancy->setOccupied({4, 4, 4});
      world.observed_occupancy.reset();
      world.static_occupancy = std::move(occupancy);
    }
    const ExecutedHorizonClearance3D clearance =
        measureExecutedHorizonClearance3D(horizon, 0U, world, footprint, 2.0);
    ASSERT_TRUE(clearance.constrained());
    // Both endpoint bodies clear the voxel. The continuous motion crosses it.
    EXPECT_DOUBLE_EQ(clearance.constrained_samples.front().body_clearance_m, 0.0);
  }
}

TEST(ExecutedHorizonClearance3DTest, MemoryAnswersHowFarAMotionWasObserved) {
  // A 20 m cube of 0.25 m voxels, observed free for x below 6 m: the shaft of
  // r500 seen from inside, its far wall never looked at.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{
      .resolution_m = 0.25, .width_cells = 80, .height_cells = 80, .depth_cells = 80}};
  for (int x = 0; x < 24; ++x) {
    for (int y = 0; y < 80; ++y) {
      for (int z = 0; z < 80; ++z) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  const Point3 vehicle{2.0, 10.0, 10.0};

  // Toward the unobserved space: 4 m of memory, to within a voxel.
  EXPECT_NEAR(
      measureObservedRangeAlong3D(occupancy, vehicle, Vec3{1.0, 0.0, 0.0}, 0.55, 30.0),
      4.0, 0.25);
  // Along it the memory reaches as far as it is probed.
  EXPECT_NEAR(
      measureObservedRangeAlong3D(occupancy, vehicle, Vec3{0.0, 1.0, 0.0}, 0.55, 6.0),
      6.0, 0.25);
  // The body is as wide as it is: beside the unobserved space, a motion along
  // it is not observed across its width.
  EXPECT_NEAR(measureObservedRangeAlong3D(occupancy, Point3{5.7, 10.0, 10.0},
                                          Vec3{0.0, 1.0, 0.0}, 0.55, 6.0),
              0.55, 1.0e-9);
  EXPECT_DOUBLE_EQ(measureObservedRangeAlong3D(occupancy, vehicle, Vec3{}, 0.55, 30.0),
                   0.0);

  // A known wall closes the lines that meet it: beside a wall along x = 4 m
  // the body's width brushes it, and what lies unobserved behind the wall is
  // not what a motion along it enters.
  for (int y = 0; y < 80; ++y) {
    for (int z = 0; z < 80; ++z) {
      occupancy.setState(GridIndex3D{16, y, z}, ObservedVoxelState::kOccupied);
      for (int x = 17; x < 24; ++x) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kUnknown);
      }
    }
  }
  EXPECT_NEAR(measureObservedRangeAlong3D(occupancy, Point3{3.4, 10.0, 10.0},
                                          Vec3{0.2, 1.0, 0.0}, 0.55, 6.0),
              6.0, 0.25);
}

TEST(ExecutedHorizonClearance3DTest, AHorizonMayNotCarrySpeedAlongAMotionNothingSees) {
  // The 20 m cube again, observed free for x below 6 m, and the stereo set:
  // 6.4 m inside 60 degrees of the heading, 2.8 m inside 22.5 of the vertical.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{
      .resolution_m = 0.25, .width_cells = 80, .height_cells = 80, .depth_cells = 80}};
  for (int x = 0; x < 24; ++x) {
    for (int y = 0; y < 80; ++y) {
      for (int z = 0; z < 80; ++z) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  SensorBrakingContract3D contract;
  contract.guaranteed_detection_range_m = 6.4;
  contract.physical_margin_m = 2.0;
  contract.forward_vertical_half_angle_rad = 0.9145;
  contract.forward_horizontal_half_angle_rad = 1.0472;
  contract.vertical_detection_range_m = 2.8;
  contract.vertical_cone_half_angle_rad = 0.3927;
  contract.vertical_physical_margin_m = 1.0;
  const StoppingCapability capability;
  const auto first_unseen = [&](const std::vector<MotionState3D>& horizon) {
    return firstUnseenMotionState3D(horizon, occupancy, contract, capability, 10.0,
                                    0.55, 0.25, 0.27);
  };

  // r518: the horizon turns toward the unobserved space at 2 m/s while its
  // planned heading still faces away. The first such state is named.
  const std::vector<MotionState3D> turning{
      MotionState3D{.x = 3.0F, .y = 10.0F, .z = 10.0F, .vy = 2.0F, .yaw = 1.5708F},
      MotionState3D{
          .x = 3.2F, .y = 10.4F, .z = 10.0F, .vx = 1.4F, .vy = 1.4F, .yaw = 1.5708F},
      MotionState3D{.x = 3.6F, .y = 10.6F, .z = 10.0F, .vx = 2.0F, .yaw = 2.8F},
      MotionState3D{.x = 4.0F, .y = 10.6F, .z = 10.0F, .vx = 2.0F, .yaw = 2.8F},
  };
  EXPECT_EQ(first_unseen(turning), 2U);

  // Faced, the same motion is the forward sensor's to answer for.
  std::vector<MotionState3D> faced = turning;
  faced[2].yaw = 0.2F;
  faced[3].yaw = 0.2F;
  EXPECT_EQ(first_unseen(faced), faced.size());

  // Unfaced through space seen before, at a speed its depth admits, and at
  // rest anywhere.
  const std::vector<MotionState3D> backing{
      MotionState3D{.x = 5.0F, .y = 10.0F, .z = 10.0F, .vx = -1.0F, .yaw = 0.0F},
      MotionState3D{.x = 5.5F, .y = 10.0F, .z = 10.0F, .vx = 0.2F, .yaw = 3.0F},
  };
  EXPECT_EQ(first_unseen(backing), backing.size());

  // A hover correction drifts a few centimetres beside the body and is not
  // refused for it; the same drift kept up is, once it has travelled the
  // envelope's clearance.
  std::vector<MotionState3D> drifting;
  for (int step = 0; step < 30; ++step) {
    drifting.push_back(MotionState3D{.x = 5.6F + 0.015F * static_cast<float>(step),
                                     .y = 10.0F,
                                     .z = 10.0F,
                                     .vx = 0.3F,
                                     .yaw = 3.0F});
  }
  EXPECT_EQ(
      first_unseen(std::vector<MotionState3D>(drifting.begin(), drifting.begin() + 15)),
      15U);
  EXPECT_EQ(first_unseen(drifting), 19U);

  // A horizon that is slowing down is answering for a speed the vehicle
  // already has, however it got it.
  const std::vector<MotionState3D> braking{
      MotionState3D{.x = 5.0F, .y = 10.0F, .z = 10.0F, .vx = 2.5F, .yaw = 3.0F},
      MotionState3D{.x = 5.12F, .y = 10.0F, .z = 10.0F, .vx = 2.3F, .yaw = 3.0F},
      MotionState3D{.x = 5.23F, .y = 10.0F, .z = 10.0F, .vx = 2.1F, .yaw = 3.0F},
      MotionState3D{.x = 5.33F, .y = 10.0F, .z = 10.0F, .vx = 1.9F, .yaw = 3.0F},
  };
  EXPECT_EQ(first_unseen(braking), braking.size());

  // A lidar that sees all around refuses nothing.
  contract.forward_horizontal_half_angle_rad = 3.141592653589793;
  EXPECT_EQ(first_unseen(turning), turning.size());
}

TEST(ExecutedHorizonClearance3DTest, TheRouteClearanceStopsAtTheLookahead) {
  const ExecutedHorizonClearance3D near = measureRouteClearance3D(
      routeAlongX(0.5, 20U), 2.0, 2.0,
      clearanceWorld(grid(), esdfWithTightColumn(6, 3.0F)), pointFootprint(), 2.0);
  ASSERT_TRUE(near.available);
  EXPECT_FALSE(near.constrained());
}

TEST(ExecutedHorizonClearance3DTest, TheRouteClearanceIgnoresUnobservedSpace) {
  // Unknown space stays traversable and carries no clearance, so a route that
  // leaves the observed cells reports no constraint and no frontier here.
  const ExecutedHorizonClearance3D clearance =
      measureRouteClearance3D(routeAlongX(0.5, 20U), 2.0, 30.0,
                              clearanceWorld(grid(), esdf()), pointFootprint(), 2.0);
  ASSERT_TRUE(clearance.available);
  EXPECT_FALSE(clearance.constrained());
  EXPECT_FALSE(clearance.unobserved());
}

TEST(ExecutedHorizonClearance3DTest,
     TheVerticalAllowanceIsTheProfilesOwnAndIsNotRoundedUp) {
  // K27's allowance belongs to the estimator that produces the height. On a
  // 0.25 m grid the camera's measured 0.5 m and the lidar's measured 0.34 m
  // are different figures, not one band: rounding the allowance up to whole
  // evidence cells made them equal, so a profile that holds its height better
  // could not say so and a ceiling inside the camera's band held the reference
  // at the contact-departure speed for the whole time the vehicle stood under
  // it, which is the shape of r3400's 19.5 s at 0.064 m/s.
  //
  // A ceiling over a level motion: the body top is 0.35 m over the estimate,
  // the slab's underside 1.00 m over the body's own lower extent, so the true
  // gap from the body's top edge is 0.40 m. The allowance decides how much of
  // it the body is charged with.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{
      .resolution_m = 0.25, .width_cells = 16, .height_cells = 16, .depth_cells = 16}};
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
      for (int z = 0; z < 16; ++z) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  // The slab: one voxel layer whose box spans z in [1.75, 2.0) m.
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
      occupancy.setState(GridIndex3D{x, y, 7}, ObservedVoxelState::kOccupied);
    }
  }
  const GridBounds3D bounds = occupancy.bounds();
  WorldSnapshot3D world;
  world.grid = EsdfGrid3D{.width = bounds.width_cells,
                          .height = bounds.height_cells,
                          .resolution_m = static_cast<float>(bounds.resolution_m),
                          .origin_x_m = static_cast<float>(bounds.origin_x),
                          .origin_y_m = static_cast<float>(bounds.origin_y),
                          .depth = bounds.depth_cells,
                          .origin_z_m = static_cast<float>(bounds.origin_z),
                          .outside_is_unknown = false};
  world.observed_occupancy = std::make_shared<ObservedOccupancyGrid3D>(occupancy);
  std::vector<float> field(static_cast<std::size_t>(bounds.width_cells) *
                               static_cast<std::size_t>(bounds.height_cells) *
                               static_cast<std::size_t>(bounds.depth_cells),
                           2.0F);
  world.distances_m = std::make_shared<const std::vector<float>>(std::move(field));

  SweptFootprintConfig footprint = pointFootprint();
  footprint.body_radius_m = 0.0;
  footprint.body_lower_extent_m = 0.35;
  footprint.body_upper_extent_m = 0.35;
  FiniteMotionHorizon3D horizon;
  horizon.states = {
      MotionState3D{.x = 2.0F, .y = 2.0F, .z = 1.0F},
      MotionState3D{.x = 2.0F, .y = 3.0F, .z = 1.0F},
  };

  const auto measured = [&](const double allowance) {
    const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
        horizon, 0U, world, footprint, 2.0, allowance);
    EXPECT_TRUE(clearance.constrained()) << "allowance " << allowance;
    return clearance.constrained_samples.front().body_clearance_m;
  };

  // 0.35 (body) + 0.50 (camera allowance) overruns the 0.40 m gap: the slab
  // reads as body contact, which is the reference sitting at the departure
  // floor.
  EXPECT_DOUBLE_EQ(measured(0.5), 0.0);
  // 0.35 + 0.34 leaves 0.06 m, a figure the 0.25 m grid cannot express: with
  // the allowance rounded up to a whole cell this answer would still be 0.0.
  EXPECT_NEAR(measured(0.34), 0.06, 1.0e-9);
  // The allowance is the only difference between the three answers.
  EXPECT_NEAR(measured(0.16), 0.24, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
