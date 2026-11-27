// The driver's internal timeouts must not follow wall-clock steps (issue #8).
// The Nano has no RTC: chrony/NTP/GNSS step the system clock after boot.
// Stepping it forward must not fake a hoverboard timeout (power press while
// driving); stepping it back must not hide a real one.
//
// This executable defines clock_gettime() (exported with ENABLE_EXPORTS), so
// rcutils' system time in the driver sees a wall clock the test can step.
// CLOCK_MONOTONIC and all other clocks are passed through unchanged.

#include <gtest/gtest.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fake_devices.hpp"

namespace
{
std::atomic<int64_t> wall_clock_offset_ns{0};
}

extern "C" int clock_gettime(clockid_t clock_id, struct timespec * tp)
{
  const auto result = static_cast<int>(::syscall(SYS_clock_gettime, clock_id, tp));
  if (result == 0 && clock_id == CLOCK_REALTIME) {
    const int64_t ns = static_cast<int64_t>(tp->tv_sec) * 1000000000LL + tp->tv_nsec +
      wall_clock_offset_ns.load();
    tp->tv_sec = ns / 1000000000LL;
    tp->tv_nsec = ns % 1000000000LL;
  }
  return result;
}

namespace
{
using fake_devices::hoverboard_hall_message;
using fake_devices::pilsbot_hardware_on;
using fake_devices::PowerButtonWatch;
using fake_devices::SilentDevice;

const rclcpp::Time now{1, 0};
const rclcpp::Duration period{0, 20000000};

class ClockSteps : public ::testing::Test
{
protected:
  void SetUp() override
  {
    wall_clock_offset_ns = 0;
    ASSERT_EQ(driver_.on_init(pilsbot_hardware_on(hoverboard_.name, head_mcu_.name)),
      hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(driver_.on_activate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    driver_.on_error(rclcpp_lifecycle::State());  // close without a power press
    wall_clock_offset_ns = 0;
  }

  SilentDevice hoverboard_;
  SilentDevice head_mcu_;
  pilsbot_driver::PilsbotDriver driver_;
  uint8_t ci_{2};
};
}  // namespace

TEST(WallClock, can_be_stepped_by_the_test)
{
  const auto before = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  wall_clock_offset_ns = 3600LL * 1000000000LL;
  const auto after = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  wall_clock_offset_ns = 0;
  EXPECT_GT((after - before).seconds(), 3599.0);
}

TEST_F(ClockSteps, a_forward_step_does_not_fake_a_hoverboard_timeout)
{
  PowerButtonWatch power_button(head_mcu_);
  fake_devices::send(hoverboard_, hoverboard_hall_message(0, 0, ci_++));
  ASSERT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);

  wall_clock_offset_ns = 60LL * 1000000000LL;  // NTP catches up a minute
  EXPECT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);
  EXPECT_FALSE(power_button.wait_for_presses(1, 1500));
}

TEST_F(ClockSteps, a_backward_step_does_not_hide_a_real_timeout)
{
  PowerButtonWatch power_button(head_mcu_);
  fake_devices::send(hoverboard_, hoverboard_hall_message(0, 0, ci_++));
  ASSERT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);

  wall_clock_offset_ns = -60LL * 1000000000LL;
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));  // hoverboard silent
  EXPECT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(power_button.wait_for_presses(1, 2000));
}
