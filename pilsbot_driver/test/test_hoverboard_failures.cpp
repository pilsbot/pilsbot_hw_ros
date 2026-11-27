// Hoverboard failures must not switch the hoverboard off mid-drive (issue #6).
// Foxy ignored read() errors; on Humble an ERROR stops the hardware through
// on_error. So a bad sample or a failed serial read reports OK, and the power
// button is pressed only by the 1 s timeout and by a clean shutdown.

#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fake_devices.hpp"

namespace
{
using fake_devices::hoverboard_hall_message;
using fake_devices::pilsbot_hardware_on;
using fake_devices::PowerButtonWatch;
using fake_devices::SilentDevice;
using fake_devices::state_value;

const rclcpp::Time now{1, 0};
const rclcpp::Duration period{0, 20000000};
constexpr double wheel_radius = 0.125;  // as in pilsbot_hardware_on

class ActiveHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(driver_.on_init(pilsbot_hardware_on(hoverboard_.name, head_mcu_.name)),
      hardware_interface::CallbackReturn::SUCCESS);
    states_ = driver_.export_state_interfaces();
    ASSERT_EQ(driver_.on_activate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    driver_.on_deactivate(rclcpp_lifecycle::State());
  }

  double left_speed() {return state_value(states_, "front_wheel_left_joint/velocity");}
  double right_speed() {return state_value(states_, "front_wheel_right_joint/velocity");}

  SilentDevice hoverboard_;
  SilentDevice head_mcu_;
  pilsbot_driver::PilsbotDriver driver_;
  std::vector<hardware_interface::StateInterface> states_;
};
}  // namespace

TEST_F(ActiveHardware, a_plausible_sample_updates_the_wheel_speeds)
{
  fake_devices::send(hoverboard_, hoverboard_hall_message(1000, -500, 2));
  EXPECT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(left_speed(), 1.0 / wheel_radius);
  EXPECT_DOUBLE_EQ(right_speed(), -0.5 / wheel_radius);
}

TEST_F(ActiveHardware, an_emi_sample_is_dropped_and_reports_ok)
{
  fake_devices::send(hoverboard_, hoverboard_hall_message(1000, -500, 2));
  ASSERT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);

  // Over 10 m/s: an EMI spike. Keep the last plausible values.
  fake_devices::send(hoverboard_, hoverboard_hall_message(20000, -500, 3));
  EXPECT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(left_speed(), 1.0 / wheel_radius);
  EXPECT_DOUBLE_EQ(right_speed(), -0.5 / wheel_radius);
}

TEST_F(ActiveHardware, a_serial_read_error_reports_ok)
{
  // Make the driver's next read() on the hoverboard fail (EBADF) by putting
  // a write-only descriptor in place of its tty.
  const int driver_fd = fake_devices::driver_fd_of(hoverboard_);
  ASSERT_GE(driver_fd, 0);
  const int write_only = ::open("/dev/null", O_WRONLY);
  ASSERT_EQ(::dup2(write_only, driver_fd), driver_fd);
  ::close(write_only);

  EXPECT_EQ(driver_.read(now, period), hardware_interface::return_type::OK);
}

TEST_F(ActiveHardware, on_error_does_not_press_the_power_button)
{
  // Data arrived less than 1 s ago, so the hoverboard counts as connected.
  PowerButtonWatch power_button(head_mcu_);
  EXPECT_EQ(driver_.on_error(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  EXPECT_EQ(power_button.presses(), 0);
}

TEST_F(ActiveHardware, a_clean_deactivation_switches_a_connected_hoverboard_off)
{
  PowerButtonWatch power_button(head_mcu_);
  EXPECT_EQ(driver_.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(power_button.wait_for_presses(1, 500));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(power_button.presses(), 1);
}

TEST(MissingHoverboard, activation_fails_without_pressing_the_power_button)
{
  // The press cannot help opening a tty; repeated activation attempts would
  // otherwise toggle the hoverboard every few seconds.
  SilentDevice head_mcu;
  PowerButtonWatch power_button(head_mcu);
  pilsbot_driver::PilsbotDriver driver;
  ASSERT_EQ(driver.on_init(pilsbot_hardware_on("/dev/does-not-exist-hoverboard", head_mcu.name)),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(driver.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(power_button.presses(), 0);
}
