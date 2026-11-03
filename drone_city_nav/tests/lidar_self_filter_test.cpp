#include "drone_city_nav/lidar_self_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

TEST(LidarSelfFilter, RejectsOwnBodyAndRotorReturns) {
  const LidarSelfFilterConfig config{};

  EXPECT_TRUE(isLidarSelfReturn(Point3{0.0, 0.0, 0.0}, config));
  // A rotor tip on the x500's 0.386 m swept circle.
  EXPECT_TRUE(isLidarSelfReturn(Point3{0.27, 0.27, -0.06}, config));
}

TEST(LidarSelfFilter, HonorsConfiguredCylinderBoundaries) {
  const LidarSelfFilterConfig config{
      .horizontal_radius_m = 0.7, .upward_extent_m = 0.4, .downward_extent_m = 0.6};

  EXPECT_TRUE(isLidarSelfReturn(Point3{0.7, 0.0, -0.4}, config));
  EXPECT_TRUE(isLidarSelfReturn(Point3{0.0, 0.0, 0.6}, config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{0.701, 0.0, 0.0}, config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{0.0, 0.0, -0.401}, config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{0.0, 0.0, 0.601}, config));
}

TEST(LidarSelfFilter, PreservesMeasuredWallReturnOutsideAirframe) {
  // r3200: this measured lidar endpoint is 0.898 m from the body axis and
  // coincides with a world wall, not the x500 airframe. It must reach both
  // the current obstacle scan and, when its voxel is outside the body, memory.
  const Point3 wall_return{-0.773394, 0.456683, 0.497496};
  const Point3 wall_voxel{-0.75, 0.50, 0.375};
  EXPECT_EQ(classifyLidarSelfHit(wall_return, wall_voxel, LidarSelfFilterConfig{}),
            LidarSelfFilterDisposition::kKeep);
}

TEST(LidarSelfFilter, SeparatesDirectSelfReturnsFromVoxelQuantization) {
  const LidarSelfFilterConfig config{.horizontal_radius_m = 0.5};

  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.49, 0.0, 0.0}, Point3{0.375, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kDiscardFromAllObstacleInputs);
  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.51, 0.0, 0.0}, Point3{0.375, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kDiscardFromPersistentMemory);
  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.51, 0.0, 0.0}, Point3{0.625, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kKeep);
  EXPECT_EQ(classifyLidarSelfHit(Point3{0.51, 0.0, 0.0}, std::nullopt, config),
            LidarSelfFilterDisposition::kKeep);
}

TEST(LidarSelfFilter, InvalidInputCannotHideAHit) {
  LidarSelfFilterConfig config{};
  config.horizontal_radius_m = -1.0;

  EXPECT_FALSE(lidarSelfFilterConfigIsValid(config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{}, config));
  EXPECT_FALSE(
      isLidarSelfReturn(Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                        LidarSelfFilterConfig{}));
}

} // namespace
} // namespace drone_city_nav
