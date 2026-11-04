#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

namespace drone_city_nav::detail {

// On the left edge the full-range half-resolution matcher cannot search the
// original image without padding. Accept its padded match only when its whole
// matching block lies inside the real right image. Never create a return where
// the original matcher had none, and never move an existing ray's free-space
// endpoint farther away.
[[nodiscard]] inline std::int16_t conservativeBorderDisparity16(
    const std::int16_t original_disparity_16,
    const std::int16_t padded_half_disparity_16, const int image_column,
    const int half_block_radius, const double focal_baseline_m_px,
    const double confident_depth_m, const double disparity_error_px) noexcept {
  if (original_disparity_16 <= 0) {
    return original_disparity_16;
  }
  const int right_support_half = image_column / 2 - half_block_radius;
  if (padded_half_disparity_16 <= 0 || right_support_half < 0 ||
      static_cast<std::int64_t>(padded_half_disparity_16) >
          static_cast<std::int64_t>(right_support_half) * 16) {
    return std::int16_t{-16};
  }
  const int full_disparity_16 =
      std::min(static_cast<int>(std::numeric_limits<std::int16_t>::max()),
               2 * static_cast<int>(padded_half_disparity_16));
  const auto selected = static_cast<std::int16_t>(
      std::max(static_cast<int>(original_disparity_16), full_disparity_16));
  if (selected == original_disparity_16) {
    return selected;
  }
  const double original_px = static_cast<double>(original_disparity_16) / 16.0;
  const double selected_px = static_cast<double>(selected) / 16.0;
  const double original_depth = focal_baseline_m_px / original_px;
  const double selected_depth = focal_baseline_m_px / selected_px;
  if (original_depth > confident_depth_m && selected_depth <= confident_depth_m) {
    const double original_free_endpoint = std::min(
        confident_depth_m, focal_baseline_m_px / (original_px + disparity_error_px));
    if (selected_depth > original_free_endpoint) {
      return std::int16_t{-16};
    }
  }
  return selected;
}

} // namespace drone_city_nav::detail
