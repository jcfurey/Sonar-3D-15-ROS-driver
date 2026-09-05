// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/sonar_driver.hpp"
#include "stream_test_support.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using sonar3d::protocol::BitmapImageType;
using sonar3d::protocol::ProtocolVersion;
constexpr std::int64_t kOrigin = 1'700'000'000'000'000'000;

double cpu_seconds()
{
  timespec value{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value);
  return value.tv_sec + value.tv_nsec * 1.0e-9;
}

std::uint64_t checksum(const std::vector<std::uint8_t> & bytes)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto byte : bytes) {
    hash = (hash ^ byte) * 1099511628211ULL;
  }
  return hash;
}

template<typename Operation>
void measure(const std::string & label, Operation operation)
{
  constexpr int kIterations = 5000;
  std::size_t bytes = 0;
  for (int warmup = 0; warmup < 20; ++warmup) {
    bytes += operation();
  }
  const auto started = Clock::now();
  const auto cpu = cpu_seconds();
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    bytes += operation();
  }
  const auto cpu_elapsed = cpu_seconds() - cpu;
  const auto wall = std::chrono::duration<double>(Clock::now() - started).count();
  std::cout << label << " iterations=" << kIterations << " wall_us=" <<
    wall * 1.0e6 / kIterations << " cpu_us=" << cpu_elapsed * 1.0e6 / kIterations <<
    " bytes=" << bytes << '\n';
}

void conversions()
{
  std_msgs::msg::Header header;
  header.frame_id = "sonar3d_link";
  for (const auto fov : {40.0F, 90.0F}) {
    for (const bool sparse : {false, true}) {
      auto source = sonar3d::testing::range_image(0, sparse);
      source.horizontal_fov_degrees = fov;
      const auto label = " fov=" + std::to_string(static_cast<int>(fov)) +
        (sparse ? " sparse" : " dense");
      std::cout << "CHECKSUM" << label << " range=" <<
        checksum(sonar3d::conversions::make_range_image(source, header).data) << " cloud=" <<
        checksum(sonar3d::conversions::make_point_cloud(source, header).data) << '\n';
      measure("RANGE" + label, [&] {
          return sonar3d::conversions::make_range_image(source, header).data.size();
        });
      measure("CLOUD" + label, [&] {
          return sonar3d::conversions::make_point_cloud(source, header).data.size();
        });
      for (const auto version : {ProtocolVersion::RIP1, ProtocolVersion::RIP2}) {
        const auto packet = sonar3d::protocol::encode_packet(source, version);
        measure("DECODE RIP" + std::to_string(static_cast<int>(version)) + label, [&] {
            return std::get<sonar3d::protocol::RangeImage>(
              sonar3d::protocol::decode_packet(packet).message).pixels.size();
          });
      }
    }
  }
}

using Frame = std::vector<std::vector<std::uint8_t>>;

std::vector<Frame> packets(int rate, int count, ProtocolVersion version)
{
  std::vector<Frame> frames;
  for (int frame = 0; frame < count; ++frame) {
    const auto nanoseconds = kOrigin + frame * (1'000'000'000LL / rate);
    auto range = sonar3d::testing::range_image(frame);
    range.header.timestamp = sonar3d::testing::timestamp(nanoseconds);
    range.horizontal_fov_degrees = rate == 5 ? 90.0F : 40.0F;
    Frame encoded{sonar3d::protocol::encode_packet(range, version)};
    for (const auto type : {BitmapImageType::SIGNAL_STRENGTH_IMAGE,
        BitmapImageType::SHADED_IMAGE})
    {
      auto bitmap = sonar3d::testing::bitmap_image(frame, type);
      bitmap.header.timestamp = range.header.timestamp;
      encoded.push_back(sonar3d::protocol::encode_packet(bitmap, version));
    }
    sonar3d::protocol::ImuBatch imu;
    imu.sequence_id = frame;
    imu.sample_count = 100 / rate;
    for (std::uint32_t sample = 0; sample < imu.sample_count; ++sample) {
      imu.timestamps.push_back(sonar3d::testing::timestamp(
          nanoseconds - (imu.sample_count - 1 - sample) * 10'000'000LL));
      imu.specific_force.insert(imu.specific_force.end(), {1.0F, 2.0F, 3.0F});
      imu.rate_of_turn.insert(imu.rate_of_turn.end(), {4.0F, 5.0F, 6.0F});
    }
    encoded.push_back(sonar3d::protocol::encode_packet(imu));
    frames.push_back(std::move(encoded));
  }
  return frames;
}

int live(int rate, int count, const std::string & products, ProtocolVersion version, bool ipc)
{
  const auto frames = packets(rate, count, version);
  const auto port = sonar3d::testing::unused_udp_port();
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(ipc);
  options.arguments({"--ros-args", "-r", "__ns:=/sonar3d_benchmark"});
  options.append_parameter_override("configure_sonar", false);
  options.append_parameter_override("sonar_ip", "127.0.0.1");
  options.append_parameter_override("multicast_group", "239.255.96.15");
  options.append_parameter_override("multicast_port", port);
  options.append_parameter_override("multicast_interface", "127.0.0.1");
  options.append_parameter_override("diagnostics_period", 0.1);
  auto driver = std::make_shared<sonar3d::SonarDriver>(options);
  rclcpp::NodeOptions observer_options;
  observer_options.use_intra_process_comms(ipc);
  auto observer = std::make_shared<rclcpp::Node>(
    "observer", "/sonar3d_benchmark", observer_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(observer);
  std::uint64_t valid = 0, gaps = 0;
  const auto diagnostic_sub = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name != "/sonar3d_benchmark/sonar3d_driver: RIP stream") {
          continue;
        }
        for (const auto & value : status.values) {
          if (value.key == "valid_packets") {valid = std::stoull(value.value);}
          if (value.key == "sequence_gaps") {gaps = std::stoull(value.value);}
        }
      }
    });
  const bool want_range = products == "range" || products == "all";
  const bool want_cloud = products == "cloud" || products == "all";
  const bool want_other = products == "all";
  std::size_t ranges = 0, clouds = 0, bitmaps = 0, samples = 0;
  std::vector<Clock::time_point> sent(count);
  std::vector<double> latencies;
  const auto latency = [&](const std_msgs::msg::Header & header) {
      const auto index = (rclcpp::Time(header.stamp).nanoseconds() - kOrigin) /
        (1'000'000'000LL / rate);
      latencies.push_back(std::chrono::duration<double, std::milli>(
          Clock::now() - sent.at(index)).count());
    };
  const auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(100));
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  if (want_range) {
    subscriptions.push_back(observer->create_subscription<sensor_msgs::msg::Image>(
        "sonar_range_image", qos, [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ++ranges;
          latency(message->header);
        }));
  }
  if (want_cloud) {
    subscriptions.push_back(observer->create_subscription<sensor_msgs::msg::PointCloud2>(
        "sonar_point_cloud", qos, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          ++clouds;
          latency(message->header);
        }));
  }
  if (want_other) {
    for (const auto * topic : {"sonar_intensity_image", "sonar_shaded_image"}) {
      subscriptions.push_back(observer->create_subscription<sensor_msgs::msg::Image>(
          topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr) {++bitmaps;}));
    }
    subscriptions.push_back(observer->create_subscription<sensor_msgs::msg::Imu>(
        "sonar_imu", qos, [&](sensor_msgs::msg::Imu::ConstSharedPtr) {++samples;}));
  }
  const auto discovery_deadline = Clock::now() + 5s;
  while (true) {
    executor.spin_some();
    bool discovered = driver->count_subscribers("/diagnostics") > 0;
    for (const auto & subscription : subscriptions) {
      discovered &= driver->count_subscribers(subscription->get_topic_name()) > 0;
    }
    if (discovered) {break;}
    if (Clock::now() > discovery_deadline) {
      throw std::runtime_error("subscriber discovery timed out");
    }
    std::this_thread::sleep_for(2ms);
  }
  const sonar3d::testing::UdpSender sender;
  const auto period = std::chrono::nanoseconds(1'000'000'000 / rate);
  const auto started = Clock::now();
  const auto cpu = cpu_seconds();
  for (int frame = 0; frame < count; ++frame) {
    sent[frame] = Clock::now();
    for (const auto & packet : frames[frame]) {
      sender.send(port, packet, "239.255.96.15");
    }
    do {
      executor.spin_some();
      std::this_thread::sleep_for(500us);
    } while (Clock::now() < started + (frame + 1) * period);
  }
  const auto drain_deadline = Clock::now() + 200ms;
  while (Clock::now() < drain_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(500us);
  }
  const auto cpu_elapsed = cpu_seconds() - cpu;
  const auto wall = std::chrono::duration<double>(Clock::now() - started).count();
  std::sort(latencies.begin(), latencies.end());
  const double p99 = latencies.empty() ? 0.0 : latencies[(latencies.size() - 1) * 99 / 100];
  std::cout << "LIVE rate=" << rate << " frames=" << count << " products=" << products <<
    " RIP" << static_cast<int>(version) << " ipc=" << ipc << " cpu_percent=" <<
    100.0 * cpu_elapsed / wall << " valid=" << valid << " gaps=" << gaps <<
    " range=" << ranges << " cloud=" << clouds << " bitmap=" << bitmaps << " imu=" <<
    samples << " latency_p99_ms=" << p99 << '\n';
  return valid == count * 4U && gaps == 0 && ranges == (want_range ? count : 0U) &&
         clouds == (want_cloud ? count : 0U) && bitmaps == (want_other ? count * 2U : 0U) &&
         samples == (want_other ? count * 100U / rate : 0U) ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "conversions") {
      conversions();
    } else if (mode == "live" && argc == 7) {
      const int rate = std::stoi(argv[2]);
      const int count = std::stoi(argv[3]);
      const std::string products = argv[4];
      const int version = std::stoi(argv[5]);
      const std::string transport = argv[6];
      if ((rate != 5 && rate != 20) || count <= 0 || (version != 1 && version != 2) ||
        (products != "none" && products != "range" && products != "cloud" && products != "all") ||
        (transport != "dds" && transport != "ipc"))
      {
        throw std::invalid_argument("invalid live benchmark arguments");
      }
      result = live(rate, count, products, static_cast<ProtocolVersion>(version),
          transport == "ipc");
    } else {
      std::cerr << "Usage: benchmark_stream conversions\n"
        "       benchmark_stream live {5|20} FRAMES {none|range|cloud|all} {1|2} {dds|ipc}\n";
      result = 1;
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
