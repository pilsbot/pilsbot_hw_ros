// Fake serial devices for the driver tests: pseudo-terminals stand in for
// the hoverboard and the Head MCU.
#pragma once

#include <dirent.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/handle.hpp>

// The driver header relies on these being included first (as in pilsbot_driver.cpp).
#include "HoverboardAPI.h"
#include "cobsr.h"
#include "../src/head_mcu/include/data_frame.hpp"
#include "pilsbot_driver/pilsbot_driver.h"

namespace fake_devices
{
inline hardware_interface::HardwareInfo pilsbot_hardware_on(
  const std::string & hoverboard, const std::string & head_mcu)
{
  hardware_interface::HardwareInfo info;
  info.name = "PilsbotSystem";
  info.type = "system";
  info.hardware_class_type = "pilsbot_driver/PilsbotDriver";
  info.hardware_parameters = {
    {"hoverboard_tty_device", hoverboard},
    {"head_mcu_tty_device", head_mcu},
    {"wheel_radius", "0.125"},
    {"steering_calibration", "50702:-1.56601,32358:0.0,14017:1.56601"},
  };
  for (const auto * name : {"front_wheel_left_joint", "front_wheel_right_joint"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    hardware_interface::InterfaceInfo velocity;
    velocity.name = "velocity";
    joint.command_interfaces.push_back(velocity);
    joint.state_interfaces.push_back(velocity);
    hardware_interface::InterfaceInfo position;
    position.name = "position";
    joint.state_interfaces.push_back(position);
    info.joints.push_back(joint);
  }
  return info;
}

// A pseudo-terminal that opens like a serial device. The test talks to it
// through `controller`; the driver opens `name`.
struct SilentDevice
{
  SilentDevice() {openpty(&controller, &device, name, nullptr, nullptr);}
  ~SilentDevice() {close(device); close(controller);}
  int controller{-1};
  int device{-1};
  char name[64]{};
};

// Counts "power button pressed" commands (output bit 2 on) the driver sends
// to the Head MCU while it exists. Reads in the background, so presses made
// inside a blocking driver call (on_activate, on_deactivate) are seen too.
class PowerButtonWatch
{
public:
  explicit PowerButtonWatch(SilentDevice & head_mcu)
  : fd_(head_mcu.controller), thread_([this] {run();}) {}
  ~PowerButtonWatch() {stop_ = true; thread_.join();}

  int presses() const {return presses_;}

  // Waits up to timeout_ms for at least `count` presses.
  bool wait_for_presses(int count, int timeout_ms) const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (presses_ < count && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return presses_ >= count;
  }

private:
  void run()
  {
    head_mcu::Command cmd;
    size_t have = 0;
    while (!stop_) {
      struct pollfd pfd{fd_, POLLIN, 0};
      if (::poll(&pfd, 1, 20) <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }
      const auto n = ::read(fd_, reinterpret_cast<char *>(&cmd) + have, sizeof(cmd) - have);
      if (n <= 0) {
        continue;
      }
      have += static_cast<size_t>(n);
      if (have == sizeof(cmd)) {
        if (cmd.type == head_mcu::Command::setOutputFrame && cmd.frame.digital0_8.as_bit.bit2) {
          ++presses_;
        }
        have = 0;
      }
    }
  }

  int fd_;
  std::atomic<bool> stop_{false};
  std::atomic<int> presses_{0};
  std::thread thread_;
};

// A hoverboard "read value response" for the hall sensors (code 0x02), as the
// firmware sends it after a requestRead(sensHall), encoded like
// hbprotocol's protocol_send_raw (no-ACK message, COBS/R, checksum).
inline std::vector<uint8_t> hoverboard_hall_message(
  int32_t speed0_mm_per_s, int32_t speed1_mm_per_s, uint8_t ci)
{
  PROTOCOL_HALL_DATA_STRUCT hall[2];
  std::memset(hall, 0, sizeof(hall));
  hall[0].HallSpeed_mm_per_s = speed0_mm_per_s;
  hall[1].HallSpeed_mm_per_s = speed1_mm_per_s;

  const uint8_t cmd = PROTOCOL_CMD_READVALRESPONSE & 0x7F;  // no-ACK
  std::vector<uint8_t> plain{HoverboardAPI::Codes::sensHall};
  const auto * bytes = reinterpret_cast<const uint8_t *>(hall);
  plain.insert(plain.end(), bytes, bytes + sizeof(hall));
  uint8_t checksum = static_cast<uint8_t>(-cmd - ci - static_cast<uint8_t>(sizeof(hall)));
  for (const auto b : plain) {
    checksum -= b;
  }
  plain.push_back(checksum);

  std::vector<uint8_t> encoded(plain.size() + 2);
  const auto result = cobsr_encode(encoded.data(), encoded.size(), plain.data(), plain.size());
  encoded.resize(result.out_len);

  std::vector<uint8_t> message{PROTOCOL_SOM, cmd, ci, static_cast<uint8_t>(encoded.size())};
  message.insert(message.end(), encoded.begin(), encoded.end());
  return message;
}

inline void send(SilentDevice & device, const std::vector<uint8_t> & data)
{
  ASSERT_EQ(::write(device.controller, data.data(), data.size()),
    static_cast<ssize_t>(data.size()));
  ::tcdrain(device.controller);
}

// The driver's own file descriptor for a fake device (found via /proc).
inline int driver_fd_of(const SilentDevice & device)
{
  int found = -1;
  DIR * dir = opendir("/proc/self/fd");
  while (dirent * entry = readdir(dir)) {
    const int fd = std::atoi(entry->d_name);
    if (fd <= 2 || fd == device.device || fd == device.controller || fd == dirfd(dir)) {
      continue;
    }
    char target[256]{};
    const std::string link = std::string("/proc/self/fd/") + entry->d_name;
    if (readlink(link.c_str(), target, sizeof(target) - 1) > 0 &&
      std::string(target) == device.name)
    {
      found = fd;
    }
  }
  closedir(dir);
  return found;
}

inline double state_value(
  std::vector<hardware_interface::StateInterface> & states, const std::string & name)
{
  for (auto & state : states) {
    if (state.get_name() == name) {
      return state.get_value();
    }
  }
  ADD_FAILURE() << "no state interface " << name;
  return NAN;
}
}  // namespace fake_devices
