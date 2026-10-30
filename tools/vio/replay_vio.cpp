// replay_vio DIR [key=value ...]: replay the production tracker and filter.
// calibration=PATH reads a converted EuRoC record's calibrated rectified pair,
// IMU noise and camera-to-IMU mount. rest_start_s and start_s bound the IMU
// samples assumed stationary. The initial position and heading come from the
// reference once; later reference poses are used only for evaluation.
// baro=1 is a synthetic truth-derived barometer and is forbidden with calibration.
#include "drone_city_nav/visual_inertial_odometry.hpp"
#include "stereo_feature_tracker.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
using namespace drone_city_nav;
struct Imu { double t; Eigen::Vector3d g, a; };
struct Truth { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };
struct Frame { double t; int w, h; long long left_offset, right_offset; };
static std::vector<std::vector<std::string>> csv(const std::string& path) {
  std::vector<std::vector<std::string>> rows; std::ifstream f(path); std::string line; std::getline(f, line);
  while (std::getline(f, line)) { std::vector<std::string> c; std::stringstream s(line); std::string x; while (std::getline(s, x, ',')) c.push_back(x); if (!c.empty()) rows.push_back(c); }
  return rows;
}
int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: replay_vio DIR [key=value ...]\n"); return 2; }
  const std::string dir = argv[1];
  std::map<std::string, std::string> opt{{"imu", "px4"}, {"every", "1"}, {"start_s", "0"}, {"rest_start_s", "0"}, {"prior", "1"}, {"scale", "1"}, {"features", "200"}, {"clones", "12"}, {"noise_px", "1.0"}, {"tag", "run"}, {"end_s", "1e9"}, {"dark_from_s", "-1"}, {"dark_to_s", "-1"}, {"baro", "0"}, {"ramp_s", "0"}, {"calibration", ""}};
  constexpr std::string_view allowed[] = {"imu", "offset_ms", "every", "start_s", "rest_start_s", "prior", "scale", "features", "clones", "noise_px", "tag", "end_s", "dark_from_s", "dark_to_s", "baro", "ramp_s", "calibration", "gyro_noise", "acc_noise", "gate", "drag", "per_update", "bg_walk", "bg_sigma", "ba_walk", "baseline", "maxdepth", "mindisp", "focal_px", "principal_x_px", "principal_y_px", "minframes"};
  for (int i = 2; i < argc; ++i) {
    std::string kv = argv[i]; const auto eq = kv.find('=');
    if (eq == std::string::npos || std::find(std::begin(allowed), std::end(allowed), kv.substr(0, eq)) == std::end(allowed)) { std::fprintf(stderr, "unknown or invalid option: %s\n", argv[i]); return 2; }
    opt[kv.substr(0, eq)] = kv.substr(eq + 1);
  }
  double offset_s = opt.contains("offset_ms") ? std::atof(opt["offset_ms"].c_str()) * 1e-3 : 0.0;
  const int every = std::atoi(opt["every"].c_str()); const int scale = std::atoi(opt["scale"].c_str());
  if (every <= 0 || scale <= 0) { std::fprintf(stderr, "every and scale must be positive\n"); return 2; }
  // ENU/FLU truth -> NED/FRD
  Eigen::Matrix3d enu_to_ned; enu_to_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d frd_to_flu = Eigen::Vector3d{1, -1, -1}.asDiagonal();
  std::vector<Imu> imu;
  for (auto& r : csv(dir + (opt["imu"] == "gz" ? "/imu_gz.csv" : "/imu_px4.csv"))) {
    Imu s{std::atof(r[0].c_str()), {std::atof(r[1].c_str()), std::atof(r[2].c_str()), std::atof(r[3].c_str())}, {std::atof(r[4].c_str()), std::atof(r[5].c_str()), std::atof(r[6].c_str())}};
    if (opt["imu"] == "gz") { s.g = frd_to_flu * s.g; s.a = frd_to_flu * s.a; }
    imu.push_back(s);
  }
  std::vector<Truth> truth;
  for (auto& r : csv(dir + "/truth.csv")) {
    const Eigen::Quaterniond q_enu_flu{std::atof(r[4].c_str()), std::atof(r[5].c_str()), std::atof(r[6].c_str()), std::atof(r[7].c_str())};
    truth.push_back({std::atof(r[0].c_str()), enu_to_ned * Eigen::Vector3d{std::atof(r[1].c_str()), std::atof(r[2].c_str()), std::atof(r[3].c_str())},
                     Eigen::Quaterniond{enu_to_ned * q_enu_flu.toRotationMatrix() * frd_to_flu}});
  }
  std::map<std::string, Frame> frames;  // by stamp text: both sides share a stamp
  for (auto& r : csv(dir + "/frames.csv")) { Frame& f = frames[r[1]]; f.t = std::atof(r[1].c_str()); f.w = std::atoi(r[3].c_str()); f.h = std::atoi(r[4].c_str()); (r[0] == "left" ? f.left_offset : f.right_offset) = std::atoll(r[5].c_str()) + 1; }
  std::vector<Frame> ordered; for (auto& [k, f] : frames) if (f.left_offset > 0 && f.right_offset > 0) ordered.push_back(f);
  std::sort(ordered.begin(), ordered.end(), [](const Frame& a, const Frame& b) { return a.t < b.t; });
  if (imu.empty() || truth.size() < 2 || ordered.empty()) { std::fprintf(stderr, "record lacks IMU, reference or paired frames\n"); return 2; }
  const auto truthAt = [&](double t) {
    const auto next = std::lower_bound(truth.begin(), truth.end(), t, [](const Truth& value, double stamp) { return value.t < stamp; });
    if (next == truth.begin()) return truth.front();
    if (next == truth.end()) return truth.back();
    const Truth& a = *(next - 1); const Truth& b = *next;
    const double u = (t - a.t) / (b.t - a.t);
    return Truth{t, a.p + u * (b.p - a.p), a.q.slerp(u, b.q)};
  };

  RectifiedStereoMount mount;
  VisualInertialOdometryConfig config;
  config.maximum_clones = static_cast<std::size_t>(std::atoi(opt["clones"].c_str()));
  StereoFeatureTrackerConfig tracker_config;
  tracker_config.image_width = static_cast<std::size_t>(ordered.front().w / scale); tracker_config.image_height = static_cast<std::size_t>(ordered.front().h / scale);
  // Simulation records may have been captured at half resolution.
  tracker_config.focal_px *= static_cast<double>(ordered.front().w) / 1280.0 / scale;
  tracker_config.principal_x_px = 0.5 * tracker_config.image_width;
  tracker_config.principal_y_px = 0.5 * tracker_config.image_height;
  tracker_config.maximum_features = static_cast<std::size_t>(std::atoi(opt["features"].c_str()));
  if (scale > 1) { tracker_config.minimum_distance_px /= scale; tracker_config.maximum_disparity_px /= scale; tracker_config.minimum_disparity_px /= scale; }
  if (!opt["calibration"].empty()) {
    cv::FileStorage file{opt["calibration"], cv::FileStorage::READ};
    if (!file.isOpened()) { std::fprintf(stderr, "cannot open calibration\n"); return 2; }
    std::string schema; file["schema"] >> schema;
    if (schema != "drone_city_nav_euroc_calibration_v1" || static_cast<int>(file["width"]) != ordered.front().w || static_cast<int>(file["height"]) != ordered.front().h || opt["imu"] != "px4" || opt["baro"] != "0" || opt.contains("baseline")) {
      std::fprintf(stderr, "invalid calibration, image dimensions, IMU frame, baseline override, or truth-derived barometer\n"); return 2;
    }
    std::vector<double> q, p; file["left_camera_to_body_frd_wxyz"] >> q; file["left_camera_position_body_frd_m"] >> p;
    if (q.size() != 4U || p.size() != 3U) { std::fprintf(stderr, "invalid camera mount\n"); return 2; }
    mount.optical_to_body = Eigen::Quaterniond{q[0], q[1], q[2], q[3]}; mount.left_body_m = {p[0], p[1], p[2]}; mount.baseline_m = static_cast<double>(file["baseline_m"]);
    if (!mount.valid()) { std::fprintf(stderr, "invalid calibrated camera mount\n"); return 2; }
    tracker_config.focal_px = static_cast<double>(file["focal_px"]) / scale;
    tracker_config.principal_x_px = static_cast<double>(file["principal_x_px"]) / scale;
    tracker_config.principal_y_px = static_cast<double>(file["principal_y_px"]) / scale;
    config.gyro_noise_radps_sqrt_hz = static_cast<double>(file["gyroscope_noise_density"]);
    config.accelerometer_noise_mps2_sqrt_hz = static_cast<double>(file["accelerometer_noise_density"]);
    config.gyro_bias_walk_radps2_sqrt_hz = static_cast<double>(file["gyroscope_random_walk"]);
    config.accelerometer_bias_walk_mps3_sqrt_hz = static_cast<double>(file["accelerometer_random_walk"]);
    if (!std::isfinite(config.gyro_noise_radps_sqrt_hz) || config.gyro_noise_radps_sqrt_hz <= 0.0 ||
        !std::isfinite(config.accelerometer_noise_mps2_sqrt_hz) || config.accelerometer_noise_mps2_sqrt_hz <= 0.0 ||
        !std::isfinite(config.gyro_bias_walk_radps2_sqrt_hz) || config.gyro_bias_walk_radps2_sqrt_hz < 0.0 ||
        !std::isfinite(config.accelerometer_bias_walk_mps3_sqrt_hz) || config.accelerometer_bias_walk_mps3_sqrt_hz < 0.0) {
      std::fprintf(stderr, "invalid calibrated IMU noise\n"); return 2;
    }
    config.rotor_drag_1ps = 0.0; // The simulated x500's drag is not this airframe's calibration.
    if (!opt.contains("offset_ms")) offset_s = static_cast<double>(file["frame_stamp_offset_s"]);
  }
  const Eigen::Matrix3d camera_to_body = mount.optical_to_body.toRotationMatrix();
  config.left_camera = {mount.optical_to_body, mount.left_body_m};
  config.right_camera = {mount.optical_to_body, mount.rightBodyM()};
  for (auto& [k, v] : opt) {
    if (k == "gyro_noise") config.gyro_noise_radps_sqrt_hz = std::atof(v.c_str());
    if (k == "acc_noise") config.accelerometer_noise_mps2_sqrt_hz = std::atof(v.c_str());
    if (k == "gate") config.gate_normal_quantile = std::atof(v.c_str());
    if (k == "drag") config.rotor_drag_1ps = std::atof(v.c_str());
    if (k == "per_update") config.maximum_features_per_update = static_cast<std::size_t>(std::atoi(v.c_str()));
    if (k == "bg_walk") config.gyro_bias_walk_radps2_sqrt_hz = std::atof(v.c_str());
    if (k == "bg_sigma") config.minimum_gyro_bias_sigma_radps = std::atof(v.c_str());
    if (k == "ba_walk") config.accelerometer_bias_walk_mps3_sqrt_hz = std::atof(v.c_str());
    if (k == "baseline") { config.left_camera.position_body_m.y() = -0.5 * std::atof(v.c_str()); config.right_camera.position_body_m.y() = 0.5 * std::atof(v.c_str()); }
    if (k == "maxdepth") config.maximum_depth_m = std::atof(v.c_str());
    if (k == "mindisp") tracker_config.minimum_disparity_px = std::atof(v.c_str());
    if (k == "focal_px") tracker_config.focal_px = std::atof(v.c_str());
    if (k == "principal_x_px") tracker_config.principal_x_px = std::atof(v.c_str());
    if (k == "principal_y_px") tracker_config.principal_y_px = std::atof(v.c_str());
    if (k == "minframes") config.minimum_track_frames = static_cast<std::size_t>(std::atoi(v.c_str()));
  }
  config.observation_noise = std::atof(opt["noise_px"].c_str()) / tracker_config.focal_px;
  VisualInertialOdometry odometry{config};
  StereoFeatureTracker tracker{tracker_config};
  std::FILE* left = std::fopen((dir + "/left.bin").c_str(), "rb"); std::FILE* right = std::fopen((dir + "/right.bin").c_str(), "rb");
  std::FILE* out = std::fopen((dir + "/replay_" + opt["tag"] + ".csv").c_str(), "w");
  if (left == nullptr || right == nullptr || out == nullptr) { std::fprintf(stderr, "cannot open replay files\n"); return 2; }
  std::fprintf(out, "stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,untriangulated,rms,ms_track,ms_filter,bgx,bgy,bgz,bax,bay,baz,healthy,dead_reckoning,unaided_s,imu_lag_ns,heading_variance_rad2\n");
  std::size_t next_imu = 0, prior_imu = 0, written = 0; double previous_stamp = -1.0; int index = 0; double baro_h = -truth.front().p.z();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  cv::Mat l_full, r_full, l, r;
  for (const Frame& f : ordered) {
    if (f.t < std::atof(opt["start_s"].c_str()) || f.t > std::atof(opt["end_s"].c_str()) || (index++ % every) != 0) continue;
    const double stamp = f.t + offset_s;
    if (stamp < truth.front().t || stamp > truth.back().t) continue;
    l_full.create(f.h, f.w, CV_8UC1); r_full.create(f.h, f.w, CV_8UC1);
    std::fseek(left, f.left_offset - 1, SEEK_SET); std::fseek(right, f.right_offset - 1, SEEK_SET);
    if (std::fread(l_full.data, 1, static_cast<std::size_t>(f.w) * f.h, left) != static_cast<std::size_t>(f.w) * f.h || std::fread(r_full.data, 1, static_cast<std::size_t>(f.w) * f.h, right) != static_cast<std::size_t>(f.w) * f.h) { std::fprintf(stderr, "incomplete stereo frame\n"); return 2; }
    if (scale > 1) { cv::resize(l_full, l, {}, 1.0 / scale, 1.0 / scale, cv::INTER_AREA); cv::resize(r_full, r, {}, 1.0 / scale, 1.0 / scale, cv::INTER_AREA); } else { l = l_full; r = r_full; }
    if (f.t >= std::atof(opt["dark_from_s"].c_str()) && f.t < std::atof(opt["dark_to_s"].c_str())) { l = cv::Mat::zeros(l.size(), l.type()); r = cv::Mat::zeros(r.size(), r.type()); }
    { const double from = std::atof(opt["dark_from_s"].c_str()), ramp = std::atof(opt["ramp_s"].c_str());
      if (ramp > 0.0 && f.t >= from - ramp && f.t < from) {
        const double share = std::max(0.02, (from - f.t) / ramp), gain = std::min(8.0, 1.0 / share);
        static cv::RNG rng{5};
        for (cv::Mat* m : {&l, &r}) {
          cv::Mat v; m->convertTo(v, CV_32F, share * gain);
          cv::Mat n(v.size(), CV_32F); rng.fill(n, cv::RNG::NORMAL, 0.0, 2.55 * (gain - 1.0));
          v += n; v.convertTo(*m, CV_8U);
        }
      } }
    std::optional<Eigen::Matrix3d> prior;
    Eigen::Matrix3d turn = Eigen::Matrix3d::Identity();
    const bool initializing = !odometry.initialized();
    if (initializing) {
      // The selected pre-start window must actually be at rest.
      const double rest_start = std::atof(opt["rest_start_s"].c_str());
      while (next_imu < imu.size() && imu[next_imu].t < rest_start) ++next_imu;
      std::size_t rest_count = 0;
      while (next_imu < imu.size() && imu[next_imu].t <= stamp) { const Imu& s = imu[next_imu++]; gyro_bias += s.g; ++rest_count; odometry.addImu({static_cast<std::int64_t>(std::llround(s.t * 1e9)), s.g, s.a}); }
      if (rest_count < 2U) { std::fprintf(stderr, "not enough stationary IMU samples\n"); return 2; }
      gyro_bias /= static_cast<double>(rest_count); prior_imu = next_imu;
      const Truth t0 = truthAt(stamp); const Eigen::Matrix3d R = t0.q.toRotationMatrix();
      odometry.initialize(static_cast<std::int64_t>(std::llround(stamp * 1e9)), t0.p, std::atan2(R(1, 0), R(0, 0)));
    } else {
      while (prior_imu < imu.size() && imu[prior_imu].t <= previous_stamp) ++prior_imu;
      double from = previous_stamp;
      for (std::size_t i = prior_imu; i < imu.size(); ++i) {
        const Imu& s = imu[i]; const double until = std::min(s.t, stamp);
        const Eigen::Vector3d w = (s.g - gyro_bias) * (until - from); const double angle = w.norm();
        if (angle > 1e-12) turn = turn * Eigen::AngleAxisd{angle, w / angle}.toRotationMatrix();
        from = until;
        if (s.t >= stamp) break;
      }
      while (next_imu < imu.size() && imu[next_imu].t <= stamp) {
        const Imu& s = imu[next_imu++]; odometry.addImu({static_cast<std::int64_t>(std::llround(s.t * 1e9)), s.g, s.a});
        if (opt["baro"] == "1") { static std::mt19937 g{3}; std::normal_distribution<double> n{0.0, 0.25}; baro_h = 0.96 * baro_h + 0.04 * (-truthAt(s.t).p.z() + n(g)); odometry.addBarometricHeight(baro_h); }
      }
      // Production waits for a sample at or after the frame for interpolation.
      if (next_imu < imu.size()) { const Imu& s = imu[next_imu++]; odometry.addImu({static_cast<std::int64_t>(std::llround(s.t * 1e9)), s.g, s.a}); }
      if (opt["prior"] == "1") prior = camera_to_body.transpose() * turn.transpose() * camera_to_body;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<StereoFeatureObservation> observations = tracker.track(l, r, prior);
    const auto t1 = std::chrono::steady_clock::now();
    if (initializing) { previous_stamp = stamp; continue; }
    const VisualInertialEstimate e = odometry.addFrame(static_cast<std::int64_t>(std::llround(stamp * 1e9)), observations);
    const auto t2 = std::chrono::steady_clock::now();
    const Truth t = truthAt(stamp);
    std::fprintf(out, "%.9f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%zu,%zu,%zu,%zu,%zu,%.3f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%lld,%.12g\n", stamp,
                 e.position_ned_m.x(), e.position_ned_m.y(), e.position_ned_m.z(), e.body_to_ned.w(), e.body_to_ned.x(), e.body_to_ned.y(), e.body_to_ned.z(),
                 e.velocity_ned_mps.x(), e.velocity_ned_mps.y(), e.velocity_ned_mps.z(), t.p.x(), t.p.y(), t.p.z(), t.q.w(), t.q.x(), t.q.y(), t.q.z(),
                 observations.size(), e.candidate_features, e.used_features, e.gated_features, e.untriangulated_features, e.residual_rms_sigma,
                 std::chrono::duration<double, std::milli>(t1 - t0).count(), std::chrono::duration<double, std::milli>(t2 - t1).count(),
                 e.gyro_bias_radps.x(), e.gyro_bias_radps.y(), e.gyro_bias_radps.z(), e.accelerometer_bias_mps2.x(), e.accelerometer_bias_mps2.y(), e.accelerometer_bias_mps2.z(),
                 e.healthy ? 1 : 0, e.dead_reckoning ? 1 : 0, e.unaided_s, static_cast<long long>(e.imu_lag_ns), e.orientation_variance_rad2.z());
    ++written;
    gyro_bias = e.gyro_bias_radps;
    previous_stamp = stamp;
  }
  std::fclose(left); std::fclose(right); std::fclose(out);
  if (written == 0U) { std::fprintf(stderr, "no replay estimates were produced\n"); return 2; }
  return 0;
}
