#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_test_support.hpp"

namespace drone_city_nav {
namespace {
// A corridor crossed by a thin wall whose doorway is offset from the lattice
// rows: on a 2 m lattice over a 0.25 m map with a 0.5 m body, the rows lie at
// y = 1, 3 and 5, the wall stands at x in [8, 8.25) between the nodes at x = 7
// and x = 9, and the door spans y in [2.75, 4.25). The straight row segment at
// y = 3 clips the lower jamb, the diagonals clip a jamb too, and a lateral
// step to y = 3.5 at the wall clears it.
struct RefinedDoorwayFixture {
  static constexpr double kResolution = 0.25;
  static constexpr int kWidth = 64;  // 16 m
  static constexpr int kHeight = 24; // 6 m
  static constexpr int kDepth = 16;  // 4 m
  static constexpr int kWallX = 32;  // x in [8, 8.25)
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.0, 3.0};
  Point3 goal{13.0, 3.0, 3.0};
  // The door edge on the upper lattice level (z = 3).
  detail::PersistentPlannerNode3D door_west{3, 1, 1};
  detail::PersistentPlannerNode3D door_east{4, 1, 1};

  RefinedDoorwayFixture() {
    for (int y = 0; y < kHeight; ++y) {
      const double y_m = static_cast<double>(y) * kResolution;
      if (y_m >= 2.75 && y_m < 4.25) {
        continue;
      }
      for (int z = 0; z < kDepth; ++z) {
        if (!occupancy->setState({kWallX, y, z}, ObservedVoxelState::kOccupied)) {
          throw std::logic_error{"fixture cell outside the grid"};
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.edge_refinement_offsets = 3U;
    config.maximum_edge_refinement_probes = 256U;
  }

  // Cells above the upper level's waypoint at (8, 3.5, 3): x in [7.75, 8.25),
  // y in [3.75, 4.25), z in [2.75, 3.25). They touch the body at the waypoint
  // and on both legs, and their centres lie at least 0.875 m from the straight
  // row segment at y = 3 against a body margin of 0.5 m plus a half voxel
  // diagonal. The lower level's door at z = 1 is untouched.
  [[nodiscard]] static std::vector<GridIndex3D> cellsAboveTheUpperWaypoint() {
    std::vector<GridIndex3D> cells;
    for (const int x : {31, 32}) {
      for (const int y : {15, 16}) {
        for (const int z : {11, 12}) {
          cells.push_back({x, y, z});
        }
      }
    }
    return cells;
  }

  [[nodiscard]] static std::shared_ptr<ObservedOccupancyGrid3D>
  withCells(const ObservedOccupancyGrid3D& base,
            const std::vector<GridIndex3D>& cells) {
    auto changed = std::make_shared<ObservedOccupancyGrid3D>(base);
    for (const GridIndex3D cell : cells) {
      if (!changed->setState(cell, ObservedVoxelState::kOccupied)) {
        throw std::logic_error{"changed cell outside the grid"};
      }
    }
    return changed;
  }

  [[nodiscard]] static std::vector<OccupancyChunkIndex3D>
  chunksOf(const std::vector<GridIndex3D>& cells) {
    std::vector<OccupancyChunkIndex3D> chunks;
    for (const GridIndex3D cell : cells) {
      chunks.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
    }
    return chunks;
  }

  [[nodiscard]] static PlannerUpdate3D
  planUntilPublishable(PersistentDStarLitePlanner3D& planner,
                       const PersistentPlannerRequest3D& req) {
    PlannerUpdate3D update = planner.plan(req);
    for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
      update = planner.plan(req);
    }
    return update;
  }
};

} // namespace
} // namespace drone_city_nav
