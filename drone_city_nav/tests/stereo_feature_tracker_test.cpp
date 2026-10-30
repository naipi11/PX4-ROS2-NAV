#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "stereo_feature_tracker.hpp"

namespace drone_city_nav {
namespace {

TEST(RectifiedStereoMount, DefaultAndRotatedMountTransformBothCamerasAndDepth) {
  RectifiedStereoMount mount;
  ASSERT_TRUE(mount.valid());
  EXPECT_TRUE(mount.rightBodyM().isApprox(Eigen::Vector3d{0.32, 0.10, -0.02}));
  EXPECT_TRUE(mount.cameraFluToBody({2.0, 0.3, 0.4})
                  .isApprox(Eigen::Vector3d{2.32, -0.40, -0.42}));
  mount.optical_to_body = Eigen::Quaterniond::Identity();
  mount.left_body_m = {1.0, 2.0, 3.0};
  mount.baseline_m = 0.4;
  EXPECT_TRUE(mount.rightBodyM().isApprox(Eigen::Vector3d{1.4, 2.0, 3.0}));
  EXPECT_TRUE(
      mount.cameraFluToBody({2.0, 0.3, 0.4}).isApprox(Eigen::Vector3d{0.7, 1.6, 5.0}));
}

TEST(RectifiedStereoMount, RejectsInvalidCalibration) {
  RectifiedStereoMount mount;
  mount.baseline_m = 0.0;
  EXPECT_FALSE(mount.valid());
  mount.baseline_m = 0.2;
  mount.optical_to_body = Eigen::Quaterniond{2.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(mount.valid());
  mount.optical_to_body = Eigen::Quaterniond::Identity();
  mount.left_body_m.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mount.valid());
}

TEST(StereoFeatureTracker, RejectsInvalidIntrinsics) {
  StereoFeatureTrackerConfig config;
  config.focal_px = 0.0;
  EXPECT_THROW(StereoFeatureTracker tracker{config}, std::invalid_argument);
  config.focal_px = 100.0;
  config.principal_x_px = std::numeric_limits<double>::infinity();
  EXPECT_THROW(StereoFeatureTracker tracker{config}, std::invalid_argument);
}

TEST(StereoFeatureTracker, ObservationsUseCalibratedOffCenterIntrinsics) {
  StereoFeatureTrackerConfig first;
  first.image_width = 320;
  first.image_height = 240;
  first.focal_px = 200.0;
  first.principal_x_px = 160.0;
  first.principal_y_px = 120.0;
  StereoFeatureTrackerConfig second = first;
  second.focal_px = 250.0;
  second.principal_x_px = 137.0;
  second.principal_y_px = 91.0;
  cv::Mat left(240, 320, CV_8UC1);
  cv::RNG rng{12345};
  rng.fill(left, cv::RNG::UNIFORM, 0, 256);
  cv::Mat right = cv::Mat::zeros(left.size(), left.type());
  left(cv::Rect{6, 0, 314, 240}).copyTo(right(cv::Rect{0, 0, 314, 240}));
  StereoFeatureTracker a{first};
  StereoFeatureTracker b{second};
  const auto observations_a = a.track(left, right, std::nullopt);
  const auto observations_b = b.track(left, right, std::nullopt);
  ASSERT_FALSE(observations_a.empty());
  ASSERT_EQ(observations_a.size(), observations_b.size());
  for (std::size_t i = 0; i < observations_a.size(); ++i) {
    EXPECT_EQ(observations_a[i].id, observations_b[i].id);
    for (const auto side : {0, 1}) {
      const auto& p = side == 0 ? observations_a[i].left : observations_a[i].right;
      const auto& q = side == 0 ? observations_b[i].left : observations_b[i].right;
      EXPECT_NEAR(
          q.x(),
          (p.x() * first.focal_px + first.principal_x_px - second.principal_x_px) /
              second.focal_px,
          1e-6);
      EXPECT_NEAR(
          q.y(),
          (p.y() * first.focal_px + first.principal_y_px - second.principal_y_px) /
              second.focal_px,
          1e-6);
    }
  }
}

} // namespace
} // namespace drone_city_nav
