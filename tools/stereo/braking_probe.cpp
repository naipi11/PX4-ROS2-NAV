// Offline illustration of the production stopping law. Never a real-airframe guarantee.
#include "drone_city_nav/sensor_braking_contract_3d.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <string_view>

namespace {
[[nodiscard]] bool number(const char* text, double& value) {
  const std::string_view input{text};
  const auto [end, error] =
      std::from_chars(input.data(), input.data() + input.size(), value);
  return error == std::errc{} && end == input.data() + input.size() &&
         std::isfinite(value);
}
} // namespace

int main(int argc, char** argv) {
  double range_m{}, evidence_age_s{};
  if (argc != 3 || !number(argv[1], range_m) || !number(argv[2], evidence_age_s) ||
      range_m <= 0.0 || evidence_age_s < 0.0) {
    std::fprintf(stderr, "usage: braking_probe RANGE_M EVIDENCE_AGE_S\n");
    return 2;
  }
  // These are the urban simulation's level-flight parameters, not measurements
  // of the robot or of any physical UAV.
  const drone_city_nav::StoppingCapability capability{
      .maximum_commanded_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_vertical_deceleration_mps2 = 1.4,
      .reaction_latency_s = 0.1};
  const drone_city_nav::SensorBrakingContract3D contract{
      .guaranteed_detection_range_m = range_m,
      .maximum_evidence_age_s = evidence_age_s,
      .physical_margin_m = 2.0,
      .maximum_horizontal_acceleration_mps2 = 4.0,
      .maximum_vertical_acceleration_mps2 = 1.4,
      .maximum_control_jerk_mps3 = 12.0};
  const bool valid =
      drone_city_nav::sensorBrakingContract3DIsValid(contract, capability);
  std::printf("{\"scope\":\"simulation stopping assumptions only; not a real-airframe "
              "guarantee\","
              "\"range_m\":%.9g,\"evidence_age_s\":%.9g,\"physical_margin_m\":2.0,"
              "\"reaction_latency_s\":0.1,\"horizontal_deceleration_mps2\":4.0,"
              "\"maximum_acceleration_mps2\":4.0,\"maximum_jerk_mps3\":12.0,"
              "\"contract_valid\":%s,\"maximum_forward_speed_mps\":",
              range_m, evidence_age_s, valid ? "true" : "false");
  if (valid) {
    const double maximum = drone_city_nav::sensorBrakingMaximumSpeedMps(
        contract, capability, 10.0, drone_city_nav::Vec3{1.0, 0.0, 0.0});
    std::printf("%.9g", maximum);
  } else {
    std::printf("null");
  }
  std::printf("}\n");
  return 0;
}
