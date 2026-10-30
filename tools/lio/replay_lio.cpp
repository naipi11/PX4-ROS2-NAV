#include "drone_city_nav/lidar_inertial_odometry.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace {
using drone_city_nav::LidarInertialImuSample;

struct Scan {
  std::int64_t stamp;
  std::uint64_t offset;
  std::uint64_t count;
};

template<std::size_t N>
std::array<std::string_view, N> fields(const std::string& line) {
  std::array<std::string_view, N> result{};
  std::string_view rest{line};
  for (std::size_t i = 0; i < N; ++i) {
    const auto comma = rest.find(',');
    if ((comma == std::string_view::npos) != (i + 1 == N))
      throw std::runtime_error("invalid CSV field count");
    result[i] = rest.substr(0, comma);
    if (comma != std::string_view::npos)
      rest.remove_prefix(comma + 1);
  }
  return result;
}

template<typename T> T number(std::string_view text) {
  T value{};
  const auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size())
    throw std::runtime_error("invalid numeric field: " + std::string{text});
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(value))
      throw std::runtime_error("non-finite numeric field");
  }
  return value;
}

void header(std::ifstream& file, const std::string& expected) {
  std::string line;
  if (!std::getline(file, line))
    throw std::runtime_error("missing CSV header");
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  if (line != expected)
    throw std::runtime_error("unexpected CSV header: " + line);
}

std::vector<LidarInertialImuSample> readImu(const std::filesystem::path& path) {
  std::ifstream file{path};
  if (!file)
    throw std::runtime_error("cannot open " + path.string());
  header(file, "stamp_ns,gx,gy,gz,ax,ay,az");
  std::vector<LidarInertialImuSample> samples;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const auto f = fields<7>(line);
    LidarInertialImuSample sample;
    sample.stamp_ns = number<std::int64_t>(f[0]);
    if (sample.stamp_ns < 0 ||
        (!samples.empty() && sample.stamp_ns <= samples.back().stamp_ns))
      throw std::runtime_error(
          "IMU timestamps are not strictly increasing nonnegative values");
    sample.gyro_radps = Eigen::Vector3d{number<double>(f[1]), number<double>(f[2]),
                                        number<double>(f[3])};
    sample.accelerometer_mps2 = Eigen::Vector3d{
        number<double>(f[4]), number<double>(f[5]), number<double>(f[6])};
    samples.push_back(sample);
  }
  if (file.bad() || samples.empty())
    throw std::runtime_error("empty or unreadable IMU input");
  return samples;
}

std::vector<Scan> readScans(const std::filesystem::path& path, std::uintmax_t bytes) {
  std::ifstream file{path};
  if (!file)
    throw std::runtime_error("cannot open " + path.string());
  header(file, "stamp_ns,offset_bytes,point_count");
  std::vector<Scan> scans;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const auto f = fields<3>(line);
    const Scan scan{number<std::int64_t>(f[0]), number<std::uint64_t>(f[1]),
                    number<std::uint64_t>(f[2])};
    if (scan.stamp < 0 || (!scans.empty() && scan.stamp <= scans.back().stamp))
      throw std::runtime_error(
          "scan timestamps are not strictly increasing nonnegative values");
    if (scan.count == 0 || scan.offset > bytes ||
        scan.count > (bytes - scan.offset) / 12 ||
        scan.offset >
            static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max()))
      throw std::runtime_error("empty or truncated scan data");
    scans.push_back(scan);
  }
  if (file.bad() || scans.empty())
    throw std::runtime_error("empty or unreadable scan input");
  return scans;
}

void readCloud(std::ifstream& file, const Scan& scan,
               std::vector<Eigen::Vector3d>& points,
               std::vector<unsigned char>& bytes) {
  file.seekg(static_cast<std::streamoff>(scan.offset));
  if (!file)
    throw std::runtime_error("cannot seek scan data");
  if (scan.count > points.max_size() || scan.count > bytes.max_size() / 12U ||
      scan.count >
          static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max()) / 12U)
    throw std::runtime_error("scan too large");
  const auto count = static_cast<std::size_t>(scan.count);
  bytes.resize(count * 12U);
  if (!file.read(reinterpret_cast<char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size())))
    throw std::runtime_error("truncated scan data");
  points.clear();
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::array<double, 3> xyz{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      const std::size_t offset = i * 12U + axis * 4U;
      const std::uint32_t bits =
          static_cast<std::uint32_t>(bytes[offset]) |
          (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U) |
          (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U) |
          (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
      xyz[axis] = static_cast<double>(std::bit_cast<float>(bits));
      if (!std::isfinite(xyz[axis]))
        throw std::runtime_error("non-finite scan point");
    }
    points.emplace_back(xyz[0], xyz[1], xyz[2]);
  }
}

int run(int argc, char** argv) {
  if (argc != 5 && argc != 6)
    throw std::runtime_error(
        "usage: replay_lio INPUT_DIR OUTPUT.csv REST_START_NS FIRST_SCAN_NS [END_NS]");
  const auto rest = number<std::int64_t>(argv[3]);
  const auto first = number<std::int64_t>(argv[4]);
  const auto end = argc == 6 ? number<std::int64_t>(argv[5])
                             : std::numeric_limits<std::int64_t>::max();
  if (rest < 0 || first < rest || end < first)
    throw std::runtime_error("invalid replay time range");
  const std::filesystem::path input{argv[1]};
  const auto imu = readImu(input / "imu.csv");
  const auto binary = input / "scans.bin";
  const auto scans = readScans(input / "scans.csv", std::filesystem::file_size(binary));
  std::ifstream cloud_file{binary, std::ios::binary};
  if (!cloud_file)
    throw std::runtime_error("cannot open scan data");
  std::size_t scan_index = 0;
  while (scan_index < scans.size() && scans[scan_index].stamp < first)
    ++scan_index;
  if (scan_index == scans.size() || scans[scan_index].stamp > end)
    throw std::runtime_error("no scans in replay range");
  std::size_t imu_index = 0;
  while (imu_index < imu.size() && imu[imu_index].stamp_ns < rest)
    ++imu_index;
  drone_city_nav::LidarInertialOdometry odometry{
      drone_city_nav::LidarInertialOdometryConfig{}};
  odometry.initialize(rest, Eigen::Vector3d::Zero(), 0.0);
  std::size_t fed = 0;
  while (imu_index < imu.size() && imu[imu_index].stamp_ns <= scans[scan_index].stamp) {
    odometry.addImu(imu[imu_index++]);
    ++fed;
  }
  if (fed < 2)
    throw std::runtime_error("first scan requires at least two rest IMU samples");
  std::ofstream output{argv[2]};
  if (!output)
    throw std::runtime_error("cannot open output CSV");
  output << "stamp_s,px,py,pz,qw,qx,qy,qz,healthy,initial_scan,imu_lag_ns,matched_"
            "fraction,residual_rms_m,degenerate_axes,gated_axes,scan_points,submap_"
            "points,keyframes,processing_ms\n"
         << std::setprecision(17);
  std::size_t estimates = 0, healthy = 0;
  std::vector<Eigen::Vector3d> points;
  std::vector<unsigned char> scan_bytes;
  for (; scan_index < scans.size() && scans[scan_index].stamp <= end; ++scan_index) {
    const auto& scan = scans[scan_index];
    if (estimates != 0) {
      while (imu_index < imu.size() && imu[imu_index].stamp_ns <= scan.stamp) {
        odometry.addImu(imu[imu_index++]);
        ++fed;
      }
      if (imu_index < imu.size()) {
        odometry.addImu(imu[imu_index++]);
        ++fed;
      }
    }
    readCloud(cloud_file, scan, points, scan_bytes);
    const auto start = std::chrono::steady_clock::now();
    const auto estimate = odometry.addScan(scan.stamp, points);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    output << static_cast<double>(estimate.stamp_ns) * 1e-9 << ','
           << estimate.position_ned_m.x() << ',' << estimate.position_ned_m.y() << ','
           << estimate.position_ned_m.z() << ',' << estimate.body_to_ned.w() << ','
           << estimate.body_to_ned.x() << ',' << estimate.body_to_ned.y() << ','
           << estimate.body_to_ned.z() << ',' << estimate.healthy << ','
           << (estimates == 0) << ',' << estimate.imu_lag_ns << ','
           << estimate.matched_fraction << ',' << estimate.residual_rms_m << ','
           << estimate.degenerate_axes << ',' << estimate.gated_axes << ','
           << estimate.scan_points << ',' << estimate.submap_points << ','
           << estimate.keyframes << ',' << ms << '\n';
    ++estimates;
    healthy += estimate.healthy ? 1U : 0U;
  }
  output.close();
  if (!output || estimates == 0)
    throw std::runtime_error("failed to write estimates");
  std::cerr << "IMU samples fed: " << fed << ", estimates: " << estimates
            << ", healthy: " << healthy
            << " (first scan is initialization, not registration)\n";
  return 0;
}
} // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "replay_lio: " << error.what() << '\n';
    return 1;
  }
}
