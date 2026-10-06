// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "http_test_server.hpp"
#include "sonar3d/http_client.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeSonarApi final : public sonar3d::SonarApi
{
public:
  void set_speed_of_sound(double value) override
  {
    speed = value;
    call("speed_of_sound");
  }
  void set_salinity(const std::string & value) override
  {
    call("salinity");
    salinity = value;
  }
  void set_acoustics_mode(const std::string & value) override
  {
    call("acoustics_mode");
    mode = value;
  }
  void set_range(double minimum, double maximum) override
  {
    call("range");
    range = {minimum, maximum};
  }
  void set_acoustics_enabled(bool value) override
  {
    acoustics = value;
    call("acoustics");
  }
  void set_udp_output(const sonar3d::UdpOutput & output) override
  {
    udp = output;
    call(output.mode == sonar3d::UdpOutputMode::UNICAST ? "unicast" : "multicast");
  }
  void set_imu_output_enabled(bool value) override
  {
    call("imu_output");
    imu_output = value;
  }
  void set_ntp_server(const std::string & value) override
  {
    call("ntp_server");
    ntp = value;
  }
  sonar3d::NtpSyncResult force_ntp_sync(double) override
  {
    call("ntp_sync");
    return {true, "synchronized", {}};
  }

  double speed{-1.0};
  std::string salinity;
  std::string mode;
  std::pair<double, double> range{};
  bool acoustics{};
  sonar3d::UdpOutput udp;
  bool imu_output{};
  std::string ntp;
  // Endpoints that fail with the given HTTP status.
  std::map<std::string, long> failures;  // NOLINT(runtime/int)
  std::vector<std::string> calls;

private:
  void call(const std::string & name)
  {
    if (const auto failure = failures.find(name); failure != failures.end()) {
      throw sonar3d::HttpStatusError(name, failure->second);
    }
    calls.push_back(name);
  }
};

TEST(HttpClient, AppliesDefaultConfigurationInOrder)
{
  FakeSonarApi api;

  const auto result = sonar3d::configure_sonar(api);

  EXPECT_EQ(result.applied, (std::vector<std::string>{"acoustics", "multicast"}));
  EXPECT_EQ(api.calls, result.applied);
  EXPECT_TRUE(result.unsupported.empty());
  EXPECT_TRUE(api.acoustics);
  EXPECT_DOUBLE_EQ(api.speed, -1.0);
}

TEST(HttpClient, AppliesEverySettingInDependencyOrder)
{
  FakeSonarApi api;
  sonar3d::ConfigurationRequest request;
  request.speed_of_sound = 0.0;
  request.salinity = "salt";
  request.acoustics_mode = "high-frequency";
  request.range_min = 0.5;
  request.range_max = 10.0;
  request.udp_output = {sonar3d::UdpOutputMode::UNICAST, "10.0.0.5", 4800};
  request.imu_output = true;
  request.ntp_server = "10.0.0.1";
  request.ntp_sync_timeout = 3.0;

  const auto result = sonar3d::configure_sonar(api, request);

  EXPECT_EQ(
    result.applied,
    (std::vector<std::string>{"ntp_server", "salinity", "speed_of_sound", "acoustics_mode",
      "range", "acoustics", "unicast", "imu_output", "ntp_sync"}));
  EXPECT_EQ(api.calls, result.applied);
  EXPECT_DOUBLE_EQ(api.speed, 0.0);
  EXPECT_EQ(api.salinity, "salt");
  EXPECT_EQ(api.mode, "high-frequency");
  EXPECT_EQ(api.range, (std::pair<double, double>{0.5, 10.0}));
  EXPECT_EQ(api.udp.unicast_destination_ip, "10.0.0.5");
  EXPECT_EQ(api.udp.unicast_destination_port, 4800);
  EXPECT_EQ(api.ntp, "10.0.0.1");
  ASSERT_TRUE(result.ntp_sync);
  EXPECT_TRUE(result.ntp_sync->success);
}

TEST(HttpClient, ReportsSettingsMissingFromOlderFirmware)
{
  FakeSonarApi api;
  for (const auto * name : {"ntp_server", "salinity", "acoustics_mode", "imu_output", "ntp_sync"}) {
    api.failures[name] = 404;
  }
  sonar3d::ConfigurationRequest request;
  request.salinity = "fresh";
  request.acoustics_mode = "low-frequency";
  request.imu_output = true;
  request.ntp_server = "auto";
  request.ntp_sync_timeout = 1.0;

  const auto result = sonar3d::configure_sonar(api, request);

  EXPECT_EQ(result.applied, (std::vector<std::string>{"acoustics", "multicast"}));
  EXPECT_EQ(
    result.unsupported,
    (std::vector<std::string>{"ntp_server", "salinity", "acoustics_mode", "imu_output",
      "ntp_sync"}));
  EXPECT_FALSE(result.ntp_sync);
}

TEST(HttpClient, RequiredSettingsAndOtherErrorsStillFail)
{
  FakeSonarApi api;
  api.failures["range"] = 404;
  sonar3d::ConfigurationRequest request;
  request.range_max = 5.0;
  EXPECT_THROW(static_cast<void>(sonar3d::configure_sonar(api, request)),
    sonar3d::HttpStatusError);

  FakeSonarApi other;
  other.failures["imu_output"] = 500;
  request = {};
  request.imu_output = true;
  EXPECT_THROW(static_cast<void>(sonar3d::configure_sonar(other, request)),
    sonar3d::HttpStatusError);

  FakeSonarApi busy;
  busy.failures["ntp_sync"] = 409;
  request = {};
  request.ntp_sync_timeout = 1.0;
  const auto result = sonar3d::configure_sonar(busy, request);
  ASSERT_TRUE(result.ntp_sync);
  EXPECT_FALSE(result.ntp_sync->success);
}

TEST(HttpClient, StopsCleanlyBetweenRequests)
{
  FakeSonarApi api;
  sonar3d::ConfigurationRequest request;
  request.speed_of_sound = 1491.0;
  request.imu_output = true;
  int checks{};
  const auto result = sonar3d::configure_sonar(api, request, [&checks]() {
        return checks++ >= 1;
    });

  EXPECT_EQ(result.applied, (std::vector<std::string>{"speed_of_sound"}));
}

TEST(HttpClient, RejectsInvalidConfiguration)
{
  const auto invalid = [](auto change) {
      sonar3d::ConfigurationRequest request;
      change(request);
      return [request] {sonar3d::validate_configuration(request, 1.0);};
    };
  EXPECT_THROW(invalid([](auto & r) {r.speed_of_sound = 999.0;})(), std::invalid_argument);
  EXPECT_THROW(invalid([](auto & r) {r.speed_of_sound = 2001.0;})(), std::invalid_argument);
  EXPECT_THROW(invalid([](auto & r) {r.salinity = "brackish";})(), std::invalid_argument);
  EXPECT_THROW(invalid([](auto & r) {r.acoustics_mode = "medium";})(), std::invalid_argument);
  EXPECT_THROW(
    invalid([](auto & r) {r.range_min = 5.0; r.range_max = 5.0;})(), std::invalid_argument);
  EXPECT_THROW(invalid([](auto & r) {r.range_max = -1.0;})(), std::invalid_argument);
  EXPECT_THROW(
    invalid([](auto & r) {r.udp_output = {sonar3d::UdpOutputMode::UNICAST, "0.0.0.0", 4747};})(),
    std::invalid_argument);
  EXPECT_THROW(invalid([](auto & r) {r.ntp_sync_timeout = -1.0;})(), std::invalid_argument);
  EXPECT_THROW(sonar3d::validate_configuration({}, 0.0), std::invalid_argument);
  EXPECT_NO_THROW(invalid([](auto & r) {r.speed_of_sound = 0.0;})());
}

TEST(HttpClient, ParsesVendorResponses)
{
  const auto about = sonar3d::parse_device_info(
    R"json({"chipid":"0x12345678","hardware_revision":6,"is_ready":true,"product_id":21045,)json"
    R"json("product_name":"Sonar 3D-15","variant":"","version":"1.8.0 (abc)",)json"
    R"json("version_short":"1.8.0"})json");
  EXPECT_EQ(about.chip_id, "0x12345678");
  EXPECT_EQ(about.version_short, "1.8.0");
  EXPECT_EQ(about.hardware_revision, 6);
  EXPECT_TRUE(about.ready);

  const auto status = sonar3d::parse_status(
    R"({"api":{"id":"api-normal","message":"Integration API is operational.",)"
    R"("operational":true,"status":"ok"},"temperature":{"id":"temp-high","message":"Hot",)"
    R"("operational":true,"status":"warning"},"systems_check":{"id":"ok","message":"OK",)"
    R"("operational":true,"status":"ok"},"time":null})");
  ASSERT_EQ(status.size(), 3U);
  EXPECT_EQ(status.at("temperature").status, "warning");
  EXPECT_EQ(status.at("api").message, "Integration API is operational.");

  EXPECT_DOUBLE_EQ(sonar3d::parse_temperature("31.5"), 31.5);

  const auto time = sonar3d::parse_time_status(
    R"({"system_time":"2026-10-06T19:00:00Z","ntp_synced":false,"ntp_synced_to":"",)"
    R"("ntp_seconds_since_last_sync":null})");
  EXPECT_FALSE(time.ntp_synced);
  EXPECT_FALSE(time.seconds_since_last_sync);

  const auto sync = sonar3d::parse_ntp_sync(
    R"({"success":true,"message":"ok","status":{"system_time":"2026-10-06T19:00:00Z",)"
    R"("ntp_synced":true,"ntp_synced_to":"10.0.0.1","ntp_seconds_since_last_sync":2}})");
  EXPECT_TRUE(sync.success);
  EXPECT_EQ(sync.status.ntp_synced_to, "10.0.0.1");
  EXPECT_EQ(sync.status.seconds_since_last_sync, 2);

  EXPECT_THROW(static_cast<void>(sonar3d::parse_temperature("hot")), std::runtime_error);
  EXPECT_THROW(static_cast<void>(sonar3d::parse_device_info("{}")), std::runtime_error);
}

TEST(HttpSonarApi, SendsJsonBodiesTheVendorApiExpects)
{
  sonar3d::testing::HttpServer server;
  for (const auto * path : {"/acoustics/speed_of_sound", "/acoustics/salinity",
      "/acoustics/mode", "/acoustics/range", "/acoustics/enabled", "/udp",
      "/output/imu-batch/enabled", "/time/ntp"})
  {
    server.respond(std::string("POST /api/v1/integration") + path, {});
  }
  sonar3d::HttpSonarApi api("127.0.0.1", 2s, {}, server.port());
  sonar3d::ConfigurationRequest request;
  request.speed_of_sound = 0.0;
  request.salinity = "salt";
  request.acoustics_mode = "low-frequency";
  request.range_min = 0.5;
  request.range_max = 12.0;
  request.udp_output = {sonar3d::UdpOutputMode::UNICAST, "10.0.0.5", 4800};
  request.imu_output = true;
  request.ntp_server = "auto";

  const auto result = sonar3d::configure_sonar(api, request);
  EXPECT_TRUE(result.unsupported.empty());

  std::map<std::string, std::string> bodies;
  for (const auto & sent : server.requests()) {
    EXPECT_EQ(sent.method, "POST");
    bodies[sent.path.substr(std::string("/api/v1/integration").size())] = sent.body;
  }
  EXPECT_EQ(bodies["/time/ntp"], R"({"ntp_address":"auto"})");
  EXPECT_EQ(bodies["/acoustics/salinity"], R"("salt")");
  EXPECT_EQ(bodies["/acoustics/speed_of_sound"], "0.0");
  EXPECT_EQ(bodies["/acoustics/mode"], R"("low-frequency")");
  EXPECT_EQ(bodies["/acoustics/range"], R"({"max":12.0,"min":0.5})");
  EXPECT_EQ(bodies["/acoustics/enabled"], "true");
  EXPECT_EQ(
    bodies["/udp"],
    R"({"mode":"unicast","unicast_destination_ip":"10.0.0.5","unicast_destination_port":4800})");
  EXPECT_EQ(bodies["/output/imu-batch/enabled"], "true");
}

TEST(HttpSonarApi, ReadsStatusAndReportsMissingEndpoints)
{
  sonar3d::testing::HttpServer server;
  server.respond("GET /api/v1/integration/temperature", {200, "27.25", 0ms});
  server.respond(
    "GET /api/v1/integration/time/status",
    {200, R"({"system_time":"2026-10-06T19:00:00Z","ntp_synced":true,"ntp_synced_to":"10.0.0.1",)"
      R"("ntp_seconds_since_last_sync":12})", 0ms});
  sonar3d::HttpSonarApi api("127.0.0.1", 2s, {}, server.port());

  EXPECT_DOUBLE_EQ(api.temperature(), 27.25);
  EXPECT_EQ(api.time_status().seconds_since_last_sync, 12);
  try {
    static_cast<void>(api.status());
    FAIL() << "expected a 404";
  } catch (const sonar3d::HttpStatusError & error) {
    EXPECT_EQ(error.status(), 404);
  }
}

TEST(HttpSonarApi, CancellationAbortsASlowRequest)
{
  sonar3d::testing::HttpServer server;
  server.respond("POST /api/v1/integration/acoustics/speed_of_sound", {204, "", 3s});
  std::atomic<bool> cancelled{false};
  sonar3d::HttpSonarApi api("127.0.0.1", 10s, [&cancelled] {return cancelled.load();},
    server.port());
  std::thread cancel([&cancelled] {
      std::this_thread::sleep_for(100ms);
      cancelled = true;
    });
  const auto started = std::chrono::steady_clock::now();
  EXPECT_THROW(api.set_speed_of_sound(1500.0), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 2s);
  cancel.join();
}

}  // namespace
