#include "drone_city_nav/stereo_depth_returns.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "stereo_depth_border_internal.hpp"

namespace drone_city_nav {
namespace {

// The working pair of roadmap item 14: 1280 px over 120 degrees, 0.20 m apart.
constexpr StereoPairGeometry kGeometry{.focal_px = 369.504,
                                       .principal_x_px = 640.0,
                                       .principal_y_px = 480.0,
                                       .baseline_m = 0.20};

TEST(StereoDepthReturns, ConfidentDepthFollowsTheDisparityErrorLaw) {
  StereoDepthReturnsConfig config;
  config.disparity_error_px = 0.45;
  config.allowed_depth_error_m = 0.25;
  const double depth_m = stereoConfidentDepthM(kGeometry, config);
  EXPECT_NEAR(depth_m, 6.41, 0.01);
  // At that depth the error law gives exactly the allowed error.
  EXPECT_NEAR(depth_m * depth_m * config.disparity_error_px /
                  (kGeometry.baseline_m * kGeometry.focal_px),
              config.allowed_depth_error_m, 1.0e-9);
}

TEST(StereoDepthReturns, APixelWithoutConfidentDepthYieldsNoReturn) {
  StereoDepthReturnsConfig config;
  config.pixel_stride = 1U;
  const std::size_t width = 4U;
  const std::size_t height = 1U;
  // 4 m, no match, 12 m (beyond the confident depth: a free ray), 0.1 m
  // (inside the minimum range).
  const auto sixteenths = [](const double depth_m) {
    return static_cast<std::int16_t>(
        std::lround(16.0 * kGeometry.focal_px * kGeometry.baseline_m / depth_m));
  };
  const std::vector<std::int16_t> disparity{sixteenths(4.0), -16, sixteenths(12.0),
                                            sixteenths(0.1)};

  const std::vector<StereoDepthReturn> returns =
      stereoDepthReturns(disparity, width, height, kGeometry, config);

  ASSERT_EQ(returns.size(), 2U);
  // Pixel (0, 0) lies left of and above the principal point: forward, to the
  // left and up in the camera's forward-left-up frame.
  EXPECT_TRUE(returns[0].hit);
  EXPECT_NEAR(returns[0].point.x, 4.0, 0.01);
  EXPECT_NEAR(returns[0].point.y, 640.0 * 4.0 / kGeometry.focal_px, 0.02);
  EXPECT_NEAR(returns[0].point.z, 480.0 * 4.0 / kGeometry.focal_px, 0.02);
  // The match at 12 m measures no surface, but its ray is free as far as the
  // confident depth: the disparity plus its error still puts the surface
  // beyond it.
  EXPECT_FALSE(returns[1].hit);
  EXPECT_NEAR(returns[1].point.x, stereoConfidentDepthM(kGeometry, config), 1.0e-9);
}

TEST(StereoDepthReturns, OffCenterCalibrationSetsRayDirection) {
  const StereoPairGeometry geometry{.focal_px = 100.0,
                                    .principal_x_px = 20.0,
                                    .principal_y_px = 10.0,
                                    .baseline_m = 0.2};
  StereoDepthReturnsConfig config;
  config.pixel_stride = 1U;
  const std::vector<std::int16_t> disparity{160};
  const auto returns = stereoDepthReturns(disparity, 1U, 1U, geometry, config);
  ASSERT_EQ(returns.size(), 1U);
  EXPECT_TRUE(returns[0].hit);
  EXPECT_NEAR(returns[0].point.x, 2.0, 1e-12);
  EXPECT_NEAR(returns[0].point.y, 0.4, 1e-12);
  EXPECT_NEAR(returns[0].point.z, 0.2, 1e-12);
}

TEST(StereoDepthReturns, TheStrideThinsTheReturns) {
  StereoDepthReturnsConfig config;
  config.pixel_stride = 4U;
  const std::vector<std::int16_t> disparity(
      16U * 8U, static_cast<std::int16_t>(16.0 * kGeometry.focal_px *
                                          kGeometry.baseline_m / 3.0));
  EXPECT_EQ(stereoDepthReturns(disparity, 16U, 8U, kGeometry, config).size(), 4U * 2U);
  EXPECT_TRUE(stereoDepthReturns(disparity, 16U, 7U, kGeometry, config).empty());
}

TEST(StereoDepthReturns, SupportedBorderMatchCannotCarveThroughANearSurface) {
  // The retained real-scene sample at column 116 had an aliased 2.1875 px
  // far match while a supported near match places the surface near 2.04 m.
  const StereoPairGeometry geometry{.focal_px = 1164.779,
                                    .principal_x_px = 570.726,
                                    .principal_y_px = 463.526,
                                    .baseline_m = 0.193006};
  StereoDepthReturnsConfig config;
  config.pixel_stride = 1U;
  const double fb = geometry.focal_px * geometry.baseline_m;
  const double confident = stereoConfidentDepthM(geometry, config);
  const std::int16_t original{35};
  const std::int16_t corrected = detail::conservativeBorderDisparity16(
      original, 880, 116, 2, fb, confident, config.disparity_error_px);
  const auto before =
      stereoDepthReturns(std::vector<std::int16_t>{original}, 1U, 1U, geometry, config);
  const auto after = stereoDepthReturns(std::vector<std::int16_t>{corrected}, 1U, 1U,
                                        geometry, config);
  ASSERT_EQ(before.size(), 1U);
  ASSERT_EQ(after.size(), 1U);
  EXPECT_FALSE(before.front().hit);
  EXPECT_GT(before.front().point.x, 11.0);
  EXPECT_TRUE(after.front().hit);
  EXPECT_NEAR(after.front().point.x, 2.04, 0.01);
}

TEST(StereoDepthReturns, UnsupportedBorderSearchProducesNoNewObservation) {
  constexpr double fb{100.0};
  constexpr double confident{7.45};
  constexpr double error{0.45};
  EXPECT_LE(
      detail::conservativeBorderDisparity16(160, -16, 116, 2, fb, confident, error), 0);
  // The matching block would reach into the synthetic padding, not the image.
  EXPECT_LE(
      detail::conservativeBorderDisparity16(160, 960, 116, 2, fb, confident, error), 0);
  // A padded match must never turn an originally missing return into evidence.
  EXPECT_LE(
      detail::conservativeBorderDisparity16(-16, 880, 116, 2, fb, confident, error), 0);
}

TEST(StereoDepthReturns, BorderCorrectionNeverExtendsTheOriginalFreeRay) {
  const StereoPairGeometry geometry{.focal_px = 100.0,
                                    .principal_x_px = 0.0,
                                    .principal_y_px = 0.0,
                                    .baseline_m = 1.0};
  StereoDepthReturnsConfig config;
  config.pixel_stride = 1U;
  config.minimum_range_m = 0.0;
  const double fb = geometry.focal_px * geometry.baseline_m;
  const double confident = stereoConfidentDepthM(geometry, config);
  // Crossing from a free-only ray to a hit can otherwise extend the ray even
  // when the selected disparity is larger, due to the old error allowance.
  EXPECT_LE(detail::conservativeBorderDisparity16(212, 108, 64, 2, fb, confident,
                                                  config.disparity_error_px),
            0);
  for (int old16 = 8; old16 <= 512; old16 += 7) {
    const auto old = static_cast<std::int16_t>(old16);
    const auto before =
        stereoDepthReturns(std::vector<std::int16_t>{old}, 1U, 1U, geometry, config);
    ASSERT_EQ(before.size(), 1U);
    for (int half16 = 8; half16 <= 512; half16 += 11) {
      const auto corrected = detail::conservativeBorderDisparity16(
          old, static_cast<std::int16_t>(half16), 128, 2, fb, confident,
          config.disparity_error_px);
      const auto after = stereoDepthReturns(std::vector<std::int16_t>{corrected}, 1U,
                                            1U, geometry, config);
      if (!after.empty()) {
        EXPECT_LE(after.front().point.x, before.front().point.x + 1.0e-12);
      }
    }
  }
}

} // namespace
} // namespace drone_city_nav
