#include "drone_city_nav/executed_horizon_clearance_3d.hpp"

#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"
#include "drone_city_nav/tracking_error_tube_config_3d.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 position(const MotionState3D& state) noexcept {
  return Point3{static_cast<double>(state.x), static_cast<double>(state.y),
                static_cast<double>(state.z)};
}

[[nodiscard]] double segmentLength(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(second.x - first.x, second.y - first.y),
                    second.z - first.z);
}

// Compare geometric margins, not ESDF lower bounds. Enclose each sweep
// interval by its centre-line box plus the physical cylinder and configured
// vertical allowance; the result is a conservative speed input.
[[nodiscard]] double bodyClearanceM(const WorldSnapshot3D& world, const Point3& first,
                                    const Point3& second,
                                    const SweptFootprintConfig& footprint,
                                    const double cap_m,
                                    const double estimate_vertical_error_m) {
  // Measure against continuous voxel boxes using the configured allowance.
  // The 0.34 m trial value came from one recorded offset, not a safety bound.
  const double error_m = std::isfinite(estimate_vertical_error_m)
                             ? std::max(0.0, estimate_vertical_error_m)
                             : 0.0;
  const double radius_m = footprint.body_radius_m;
  const double lower_extent_m = footprint.body_lower_extent_m + error_m;
  const double upper_extent_m = footprint.body_upper_extent_m + error_m;
  const auto interval_count = std::max(
      std::size_t{1U},
      static_cast<std::size_t>(std::ceil(segmentLength(first, second) /
                                         std::max(footprint.sweep_step_m, 1.0e-6))));
  const auto measure = [&](const auto& occupancy) {
    double margin_m = cap_m;
    Point3 previous = first;
    for (std::size_t index = 1U; index <= interval_count && margin_m > 0.0; ++index) {
      const double ratio =
          static_cast<double>(index) / static_cast<double>(interval_count);
      const Point3 next{std::lerp(first.x, second.x, ratio),
                        std::lerp(first.y, second.y, ratio),
                        std::lerp(first.z, second.z, ratio)};
      const Point3 minimum{std::min(previous.x, next.x), std::min(previous.y, next.y),
                           std::min(previous.z, next.z)};
      const Point3 maximum{std::max(previous.x, next.x), std::max(previous.y, next.y),
                           std::max(previous.z, next.z)};
      const Point3 center{0.5 * (minimum.x + maximum.x), 0.5 * (minimum.y + maximum.y),
                          0.5 * (minimum.z + maximum.z)};
      const double reach_m =
          margin_m + std::max({0.5 * (maximum.x - minimum.x) + radius_m,
                               0.5 * (maximum.y - minimum.y) + radius_m,
                               0.5 * (maximum.z - minimum.z) +
                                   std::max(lower_extent_m, upper_extent_m)});
      forEachRawOccupiedVoxelNear3D(
          occupancy, center, reach_m, nullptr,
          [&](const Point3& box_minimum, const Point3& box_maximum) {
            const double dx =
                std::max({box_minimum.x - maximum.x, minimum.x - box_maximum.x, 0.0});
            const double dy =
                std::max({box_minimum.y - maximum.y, minimum.y - box_maximum.y, 0.0});
            const double horizontal_gap_m = std::hypot(dx, dy) - radius_m;
            const double vertical_gap_m =
                std::max(box_minimum.z - maximum.z - upper_extent_m,
                         minimum.z - lower_extent_m - box_maximum.z);
            margin_m =
                std::min(margin_m, std::max({horizontal_gap_m, vertical_gap_m, 0.0}));
          });
      previous = next;
    }
    return margin_m;
  };
  if (world.observed_occupancy != nullptr) {
    return measure(*world.observed_occupancy);
  }
  if (world.static_occupancy != nullptr) {
    return measure(*world.static_occupancy);
  }
  return std::numeric_limits<double>::infinity();
}

} // namespace

ExecutedHorizonClearance3D measureExecutedHorizonClearance3D(
    const FiniteMotionHorizon3D& horizon, const std::size_t first_remaining_state_index,
    const WorldSnapshot3D& world, const SweptFootprintConfig& footprint,
    const double constraint_clearance_m, const double estimate_vertical_error_m) {
  ExecutedHorizonClearance3D result;
  if (horizon.states.size() < 2U ||
      first_remaining_state_index + 1U >= horizon.states.size() ||
      world.distances_m == nullptr || world.distances_m->empty() ||
      !(constraint_clearance_m > 0.0)) {
    return result;
  }
  result.available = true;
  double travelled_m{0.0};
  for (std::size_t index = first_remaining_state_index;
       index + 1U < horizon.states.size(); ++index) {
    const Point3 first = position(horizon.states[index]);
    const Point3 second = position(horizon.states[index + 1U]);
    const DerivedFootprintClearance3D clearance = querySweptFootprintClearance3D(
        world.grid, *world.distances_m, first, second, footprint);
    if (clearance.evidence.unknown_exposure && !result.unobserved_distance_m) {
      result.unobserved_distance_m = travelled_m;
    }
    if (clearance.evidence.known_clearance_observed) {
      const double clearance_m = clearance.evidence.minimum_known_clearance_m;
      result.minimum_clearance_m = std::min(result.minimum_clearance_m, clearance_m);
      if (clearance_m < constraint_clearance_m) {
        result.constrained_samples.push_back(ConstrainedHorizonSample3D{
            .distance_m = travelled_m,
            .clearance_m = clearance_m,
            .body_clearance_m =
                bodyClearanceM(world, first, second, footprint, constraint_clearance_m,
                               estimate_vertical_error_m)});
      }
    }
    travelled_m += segmentLength(first, second);
  }
  return result;
}

ExecutedHorizonClearance3D measureRouteClearance3D(
    const std::span<const RouteSample3D> route, const double from_station_m,
    const double lookahead_m, const WorldSnapshot3D& world,
    const SweptFootprintConfig& footprint, const double constraint_clearance_m,
    const double estimate_vertical_error_m) {
  ExecutedHorizonClearance3D result;
  if (route.size() < 2U || !std::isfinite(from_station_m) ||
      !std::isfinite(lookahead_m) || !(lookahead_m > 0.0) ||
      world.distances_m == nullptr || world.distances_m->empty() ||
      !(constraint_clearance_m > 0.0)) {
    return result;
  }
  result.available = true;
  const double end_station_m = from_station_m + lookahead_m;
  Point3 previous = sampleRoute3DAtStation(route, from_station_m).position;
  double previous_station_m = from_station_m;
  for (const RouteSample3D& sample : route) {
    if (sample.station_m <= from_station_m) {
      continue;
    }
    const DerivedFootprintClearance3D clearance = querySweptFootprintClearance3D(
        world.grid, *world.distances_m, previous, sample.position, footprint);
    if (clearance.evidence.known_clearance_observed) {
      const double clearance_m = clearance.evidence.minimum_known_clearance_m;
      result.minimum_clearance_m = std::min(result.minimum_clearance_m, clearance_m);
      if (clearance_m < constraint_clearance_m) {
        result.constrained_samples.push_back(ConstrainedHorizonSample3D{
            .distance_m = std::max(0.0, previous_station_m - from_station_m),
            .clearance_m = clearance_m,
            .body_clearance_m =
                bodyClearanceM(world, previous, sample.position, footprint,
                               constraint_clearance_m, estimate_vertical_error_m)});
      }
    }
    if (sample.station_m >= end_station_m) {
      break;
    }
    previous = sample.position;
    previous_station_m = sample.station_m;
  }
  return result;
}

std::optional<double>
measureRouteObservedRange3D(const std::span<const RouteSample3D> route,
                            const double from_station_m, const double lookahead_m,
                            const EsdfGrid3D& grid, const std::span<const float> esdf_m,
                            const SweptFootprintConfig& footprint) {
  if (route.size() < 2U || !std::isfinite(from_station_m) ||
      !std::isfinite(lookahead_m) || !(lookahead_m > 0.0) || esdf_m.empty()) {
    return std::nullopt;
  }
  const double end_station_m = from_station_m + lookahead_m;
  Point3 previous = sampleRoute3DAtStation(route, from_station_m).position;
  double previous_station_m = from_station_m;
  for (const RouteSample3D& sample : route) {
    if (sample.station_m <= from_station_m) {
      continue;
    }
    const DerivedFootprintClearance3D clearance = querySweptFootprintClearance3D(
        grid, esdf_m, previous, sample.position, footprint);
    if (clearance.evidence.unknown_exposure) {
      return std::max(0.0, previous_station_m - from_station_m);
    }
    if (sample.station_m >= end_station_m) {
      break;
    }
    previous = sample.position;
    previous_station_m = sample.station_m;
  }
  return std::nullopt;
}

std::size_t firstUnseenMotionState3D(const std::span<const MotionState3D> horizon,
                                     const ObservedOccupancyGrid3D& occupancy,
                                     const SensorBrakingContract3D& contract,
                                     const StoppingCapability& stopping_capability,
                                     const double absolute_speed_limit_mps,
                                     const double body_radius_m,
                                     const double rest_speed_mps,
                                     const double travel_allowance_m) {
  double unseen_travel_m{0.0};
  double previous_speed_mps{std::numeric_limits<double>::infinity()};
  for (std::size_t index = 0U; index < horizon.size(); ++index) {
    const MotionState3D& state = horizon[index];
    const Vec3 velocity{static_cast<double>(state.vx), static_cast<double>(state.vy),
                        static_cast<double>(state.vz)};
    const double speed_mps = std::hypot(std::hypot(velocity.x, velocity.y), velocity.z);
    const bool slowing = speed_mps < previous_speed_mps;
    previous_speed_mps = speed_mps;
    if (index == 0U || slowing || speed_mps <= rest_speed_mps ||
        !sensorBrakingMotionUnfaced3D(contract, velocity,
                                      static_cast<double>(state.yaw))) {
      continue;
    }
    const double observed_range_m = measureObservedRangeAlong3D(
        occupancy,
        Point3{static_cast<double>(state.x), static_cast<double>(state.y),
               static_cast<double>(state.z)},
        velocity, body_radius_m, contract.guaranteed_detection_range_m);
    if (speed_mps <= sensorBrakingMemorySpeedMps(contract, stopping_capability,
                                                 absolute_speed_limit_mps, velocity,
                                                 observed_range_m)) {
      continue;
    }
    const MotionState3D& before = horizon[index - 1U];
    unseen_travel_m += std::hypot(std::hypot(static_cast<double>(state.x - before.x),
                                             static_cast<double>(state.y - before.y)),
                                  static_cast<double>(state.z - before.z));
    if (unseen_travel_m > travel_allowance_m) {
      return index;
    }
  }
  return horizon.size();
}

double measureObservedRangeAlong3D(const ObservedOccupancyGrid3D& occupancy,
                                   const Point3& origin, const Vec3& direction,
                                   const double body_radius_m,
                                   const double maximum_range_m) {
  const double length = std::hypot(std::hypot(direction.x, direction.y), direction.z);
  const double step_m = 0.5 * occupancy.bounds().resolution_m;
  if (!(length > 1.0e-9) || !(step_m > 0.0) || !(body_radius_m >= 0.0) ||
      !std::isfinite(maximum_range_m)) {
    return 0.0;
  }
  const Vec3 along{direction.x / length, direction.y / length, direction.z / length};
  // Two axes across the motion: the body's radius is probed on both.
  const Vec3 reference =
      std::abs(along.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
  Vec3 across{along.y * reference.z - along.z * reference.y,
              along.z * reference.x - along.x * reference.z,
              along.x * reference.y - along.y * reference.x};
  const double across_length = std::hypot(std::hypot(across.x, across.y), across.z);
  across = Vec3{across.x / across_length, across.y / across_length,
                across.z / across_length};
  const Vec3 other{along.y * across.z - along.z * across.y,
                   along.z * across.x - along.x * across.z,
                   along.x * across.y - along.y * across.x};
  const std::array<Vec3, 5U> offsets{
      Vec3{},
      Vec3{across.x * body_radius_m, across.y * body_radius_m,
           across.z * body_radius_m},
      Vec3{-across.x * body_radius_m, -across.y * body_radius_m,
           -across.z * body_radius_m},
      Vec3{other.x * body_radius_m, other.y * body_radius_m, other.z * body_radius_m},
      Vec3{-other.x * body_radius_m, -other.y * body_radius_m,
           -other.z * body_radius_m},
  };
  // Each line across the body is followed until memory has not observed a
  // voxel, which ends what is observed along the motion, or until it meets a
  // voxel memory holds occupied, which closes that line: what lies behind a
  // known surface is unobserved and cannot be entered, and the clearance laws
  // own the surface. Read through it, a body's width beside a wall reached the
  // unobserved inside of the wall at once, and a vehicle in a corridor was
  // refused every motion it did not face.
  double observed_m = maximum_range_m;
  for (const Vec3& offset : offsets) {
    for (double range_m = body_radius_m + step_m; range_m <= maximum_range_m;
         range_m += step_m) {
      const std::optional<GridIndex3D> cell =
          occupancy.worldToCell(Point3{origin.x + along.x * range_m + offset.x,
                                       origin.y + along.y * range_m + offset.y,
                                       origin.z + along.z * range_m + offset.z});
      if (!cell.has_value() || !occupancy.isKnown(*cell)) {
        observed_m = std::min(observed_m, range_m - step_m);
        break;
      }
      if (occupancy.isOccupied(*cell)) {
        break;
      }
    }
  }
  return std::max(observed_m, body_radius_m);
}

} // namespace drone_city_nav
