#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct LidarSelfFilterConfig {
  // The x500 rotor tips reach about 0.386 m from base_link. Keep a small
  // sensor-noise margin while reducing the blind zone outside the airframe.
  // Other airframes must configure this radius for their own geometry.
  double horizontal_radius_m{0.4};
  double upward_extent_m{0.5};
  double downward_extent_m{0.5};
};

enum class LidarSelfFilterDisposition : std::uint8_t {
  kKeep,
  kDiscardFromAllObstacleInputs,
  kDiscardFromPersistentMemory,
};

[[nodiscard]] bool
lidarSelfFilterConfigIsValid(const LidarSelfFilterConfig& config) noexcept;

[[nodiscard]] bool isLidarSelfReturn(const Point3& point_body_frd,
                                     const LidarSelfFilterConfig& config) noexcept;

[[nodiscard]] LidarSelfFilterDisposition
classifyLidarSelfHit(const Point3& endpoint_body_frd,
                     const std::optional<Point3>& occupancy_voxel_center_body_frd,
                     const LidarSelfFilterConfig& config) noexcept;

} // namespace drone_city_nav
