// Losing the Head MCU mid-drive (issue #7): while it is disconnected the
// steering angle and the endstops read NaN, so the controller halts the
// wheels instead of steering on a stale angle, and the driver keeps
// reconnecting until it is stopped.

#include <gtest/gtest.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <dirent.h>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fake_devices.hpp"

namespace
{
using fake_devices::pilsbot_hardware_on;
using fake_devices::SilentDevice;
using fake_devices::state_value;
using namespace std::chrono_literals;

constexpr uint16_t raw_straight_ahead = 32358;  // calibrated to 0.0 rad in pilsbot_hardware_on

// A Head MCU behind a udev-like symlink that can be unplugged and plugged
// back in (the pty behind it changes, the path stays).
class PluggableHeadMcu
{
public:
  PluggableHeadMcu()
  {
    char dir_template[] = "/tmp/pilsbot_head_mcu_XXXXXX";
    dir_ = mkdtemp(dir_template);
    path_ = dir_ + "/head_mcu";
    plug();
  }
  ~PluggableHeadMcu()
  {
    unplug();
    rmdir(dir_.c_str());
  }

  const std::string & path() const {return path_;}

  void plug()
  {
    pty_ = std::make_unique<SilentDevice>();
    ASSERT_EQ(symlink(pty_->name, path_.c_str()), 0);
  }

  void unplug()
  {
    unlink(path_.c_str());
    pty_.reset();
  }

  void send_frame(uint16_t raw_angle, bool endstop_l, bool endstop_r)
  {
    head_mcu::Frame frame{};
    frame.analog0 = raw_angle;
    frame.digital0_8.as_bit.bit0 = endstop_l;
    frame.digital0_8.as_bit.bit1 = endstop_r;
    ASSERT_EQ(::write(pty_->controller, &frame, sizeof(frame)), static_cast<ssize_t>(sizeof(frame)));
  }

private:
  std::string dir_;
  std::string path_;
  std::unique_ptr<SilentDevice> pty_;
};

class HeadMcu : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(driver_.on_init(pilsbot_hardware_on(hoverboard_.name, head_mcu_.path())),
      hardware_interface::CallbackReturn::SUCCESS);
    states_ = driver_.export_state_interfaces();
  }

  void TearDown() override
  {
    driver_.on_error(rclcpp_lifecycle::State());  // close without a power press
  }

  void activate()
  {
    ASSERT_EQ(driver_.on_activate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  double steering() {return state_value(states_, "steering_axle_joint/position");}
  double endstop_l() {return state_value(states_, "head_mcu/endstop_l");}
  double endstop_r() {return state_value(states_, "head_mcu/endstop_r");}

  // Feeds frames until the driver reports a steering angle, up to timeout.
  bool steering_known_within(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      head_mcu_.send_frame(raw_straight_ahead, false, false);
      std::this_thread::sleep_for(20ms);
      if (!std::isnan(steering())) {
        return true;
      }
    }
    return false;
  }

  bool steering_lost_within(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (std::isnan(steering()) && std::isnan(endstop_l()) && std::isnan(endstop_r())) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  SilentDevice hoverboard_;
  PluggableHeadMcu head_mcu_;
  pilsbot_driver::PilsbotDriver driver_;
  std::vector<hardware_interface::StateInterface> states_;
};

// Interrupts every thread but the caller's with a no-op signal handler,
// so a poll() in the Head MCU thread returns -1 (EINTR).
void interrupt_other_threads()
{
  struct sigaction action{};
  action.sa_handler = [](int) {};
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;  // no SA_RESTART
  sigaction(SIGUSR1, &action, nullptr);

  const pid_t self = static_cast<pid_t>(syscall(SYS_gettid));
  DIR * dir = opendir("/proc/self/task");
  while (dirent * entry = readdir(dir)) {
    const pid_t tid = std::atoi(entry->d_name);
    if (tid > 0 && tid != self) {
      syscall(SYS_tgkill, getpid(), tid, SIGUSR1);
    }
  }
  closedir(dir);
}

std::chrono::milliseconds time_of(const std::function<void()> & action)
{
  const auto start = std::chrono::steady_clock::now();
  action();
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start);
}
}  // namespace

TEST_F(HeadMcu, steering_and_endstops_are_unknown_until_the_first_frame)
{
  EXPECT_TRUE(std::isnan(steering()));
  EXPECT_TRUE(std::isnan(endstop_l()));
  EXPECT_TRUE(std::isnan(endstop_r()));
  activate();
  EXPECT_TRUE(std::isnan(steering()));
  EXPECT_TRUE(std::isnan(endstop_l()));
  EXPECT_TRUE(std::isnan(endstop_r()));
}

TEST_F(HeadMcu, frames_set_steering_and_endstops)
{
  activate();
  ASSERT_TRUE(steering_known_within(1s));
  EXPECT_DOUBLE_EQ(steering(), 0.0);
  EXPECT_EQ(endstop_l(), 0.0);
  EXPECT_EQ(endstop_r(), 0.0);
}

TEST_F(HeadMcu, values_do_not_survive_a_reactivation)
{
  activate();
  ASSERT_TRUE(steering_known_within(1s));
  ASSERT_EQ(driver_.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  activate();
  EXPECT_TRUE(std::isnan(steering()));
  EXPECT_TRUE(std::isnan(endstop_l()));
  EXPECT_TRUE(std::isnan(endstop_r()));
}

TEST_F(HeadMcu, unplugged_mid_run_reads_nan_and_reconnects_after_replug)
{
  activate();
  ASSERT_TRUE(steering_known_within(1s));

  head_mcu_.unplug();
  EXPECT_TRUE(steering_lost_within(500ms));
  EXPECT_EQ(driver_.read(rclcpp::Time(1, 0), rclcpp::Duration(0, 20000000)),
    hardware_interface::return_type::OK);

  // Longer than the old 3 reconnect attempts: the driver must still be trying.
  std::this_thread::sleep_for(4500ms);
  EXPECT_TRUE(std::isnan(steering()));

  head_mcu_.plug();
  EXPECT_TRUE(steering_known_within(3s));
  EXPECT_DOUBLE_EQ(steering(), 0.0);
  EXPECT_EQ(endstop_l(), 0.0);
}

TEST_F(HeadMcu, deactivation_is_quick_while_reconnecting)
{
  activate();
  ASSERT_TRUE(steering_known_within(1s));
  head_mcu_.unplug();
  ASSERT_TRUE(steering_lost_within(500ms));
  std::this_thread::sleep_for(300ms);  // inside the reconnect wait

  const auto took = time_of([this] {driver_.on_deactivate(rclcpp_lifecycle::State());});
  EXPECT_LT(took.count(), 500);
}

TEST_F(HeadMcu, an_interrupted_poll_does_not_block_shutdown)
{
  activate();
  ASSERT_TRUE(steering_known_within(1s));
  std::this_thread::sleep_for(200ms);  // the Head MCU is now silent

  for (int i = 0; i < 5; ++i) {
    interrupt_other_threads();
    std::this_thread::sleep_for(30ms);
  }

  // on_error stops like on_deactivate but without the (1 s) power press.
  auto deactivation = std::async(std::launch::async, [this] {
        return driver_.on_error(rclcpp_lifecycle::State());
      });
  const bool finished = deactivation.wait_for(1s) == std::future_status::ready;
  EXPECT_TRUE(finished) << "shutdown hangs: Head MCU thread blocked in read()";
  if (!finished) {
    head_mcu_.send_frame(raw_straight_ahead, false, false);  // unblock it for the teardown
  }
  deactivation.wait();
}
