// Copyright 2026 Pixmoving, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <pix_hooke_driver/control_command.hpp>
#include <pix_hooke_driver/control_converter.hpp>

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using can_msgs::msg::Frame;
using pix_hooke_driver_msgs::msg::V2aDriveStaFb;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;

namespace
{
struct Sample
{
  double t_ms;  // time since actuation commands stopped, negative before that
  uint32_t id;
  double value;  // throttle for 0x130, brake for 0x131, in %
};

double decode10(const Frame & f, int lo)
{
  return ((f.data[lo] | ((f.data[lo + 1] & 0x3) << 8)) * 0.1);
}

// Run control_converter and control_command together. Autoware sends 50% throttle at 30 Hz,
// then stops sending actuation commands. Gear command, gear feedback and operation mode keep
// coming. Returns every 0x130 / 0x131 frame on the CAN output.
std::vector<Sample> run(double drive_ms, double after_ms)
{
  auto converter = std::make_shared<pix_hooke_driver::control_converter::ControlConverter>();
  auto command = std::make_shared<pix_hooke_driver::control_command::ControlCommand>();
  auto io = std::make_shared<rclcpp::Node>("test_io");

  auto actuation_pub =
    io->create_publisher<ActuationCommandStamped>("/control/command/actuation_cmd", 1);
  auto gear_pub = io->create_publisher<GearCommand>("/control/command/gear_cmd", 1);
  auto fb_pub = io->create_publisher<V2aDriveStaFb>("/pix_hooke/v2a_drivestafb", 1);
  auto mode_pub = io->create_publisher<OperationModeState>("/api/operation_mode/state", 1);

  using clock = std::chrono::steady_clock;
  const auto start = clock::now();
  const auto stop = start + std::chrono::duration<double, std::milli>(drive_ms);
  const auto end = stop + std::chrono::duration<double, std::milli>(after_ms);
  auto ms_since_stop = [&]() {
      return std::chrono::duration<double, std::milli>(clock::now() - stop).count();
    };

  std::vector<Sample> out;
  auto can_sub = io->create_subscription<Frame>(
    "output/can_tx", 10, [&](const Frame::ConstSharedPtr f) {
      if (f->id == 0x130) {out.push_back({ms_since_stop(), f->id, decode10(*f, 3)});}
      if (f->id == 0x131) {out.push_back({ms_since_stop(), f->id, decode10(*f, 1)});}
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(converter);
  exec.add_node(command);
  exec.add_node(io);

  auto next_actuation = start;
  auto next_status = start;
  while (clock::now() < end) {
    const auto now = clock::now();
    if (now < stop && now >= next_actuation) {
      ActuationCommandStamped a;
      a.header.stamp = io->now();
      a.actuation.accel_cmd = 0.5;
      a.actuation.brake_cmd = 0.0;
      actuation_pub->publish(a);
      next_actuation += std::chrono::microseconds(33333);
    }
    if (now >= next_status) {
      GearCommand g;
      g.stamp = io->now();
      g.command = GearCommand::DRIVE;
      gear_pub->publish(g);
      V2aDriveStaFb fb;
      fb.header.stamp = io->now();
      fb.vcu_chassis_gear_fb = pix_hooke_driver::control_converter::VCU_CHASSISGEARFB_D;
      fb_pub->publish(fb);
      OperationModeState m;
      m.stamp = io->now();
      m.mode = OperationModeState::AUTONOMOUS;
      m.is_autoware_control_enabled = true;
      mode_pub->publish(m);
      next_status += std::chrono::milliseconds(20);
    }
    exec.spin_some(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  return out;
}
}  // namespace

// While Autoware sends commands, the commanded throttle goes out on the bus.
// Once it stops, the last throttle must not keep going out until control_command's
// 1000 ms timeout, and a brake command must follow.
TEST(ActuationTimeout, ThrottleStopsAfterActuationStops)
{
  const auto samples = run(1000.0, 1500.0);

  int throttle_before = 0;
  int throttle_late = 0;
  int brake_after = 0;
  double last_throttle_ms = -1.0;
  double first_brake_ms = -1.0;
  for (const auto & s : samples) {
    if (s.id == 0x130 && s.t_ms < 0.0 && s.t_ms > -500.0 && s.value > 49.0) {throttle_before++;}
    if (s.id == 0x130 && s.t_ms > 500.0 && s.value > 0.0) {throttle_late++;}
    if (s.id == 0x131 && s.t_ms > 0.0 && s.value >= 20.0) {brake_after++;}
    if (s.id == 0x130 && s.value > 0.0) {last_throttle_ms = s.t_ms;}
    if (s.id == 0x131 && s.t_ms > 0.0 && s.value >= 20.0 && first_brake_ms < 0.0) {
      first_brake_ms = s.t_ms;
    }
  }
  std::printf(
    "last nonzero throttle frame %.1f ms, first brake frame %.1f ms after the stop\n",
    last_throttle_ms, first_brake_ms);
  EXPECT_GT(throttle_before, 0) << "commanded throttle never reached the bus";
  EXPECT_EQ(throttle_late, 0) << "throttle frames still sent more than 500 ms after the last command";
  EXPECT_GT(brake_after, 0) << "no brake frame after actuation commands stopped";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
