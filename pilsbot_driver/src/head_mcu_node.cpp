#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pilsbot_driver_msgs/msg/steering_axle_sensors_stamped.hpp"

#include "./head_mcu/include/data_frame.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


using namespace std::chrono_literals;
using namespace pilsbot_driver_msgs::msg;

class Head_MCU_node : public rclcpp::Node
{
  typedef std::vector<double> CalibrationList;  // interpreted as pairs
  struct Parameter {
    CalibrationList calibration;
    std::string devicename;
    unsigned baud_rate;
    unsigned publish_rate;
  } params_;

  int serial_fd = -1;

  SteeringAxleSensorsStamped state_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<SteeringAxleSensorsStamped>::SharedPtr out_;

  public:
  Head_MCU_node() : Node("head_mcu_node")
    {
      this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud_rate", 115200);
      this->declare_parameter<int>("publish_rate", 10);
      this->declare_parameter<CalibrationList>("calibration_val",CalibrationList());

      out_ = this->create_publisher<SteeringAxleSensorsStamped>("/head_mcu", 10);

      params_.devicename = this->get_parameter("serial_port").as_string();
      params_.baud_rate = this->get_parameter("baud_rate").as_int();
      params_.publish_rate = this->get_parameter("publish_rate").as_int();
      std::cout << params_.publish_rate << std::endl;
      assert(params_.publish_rate > 0);
      //params_.calibration = this->get_parameter("calibration_val").as_what?

      if ((serial_fd = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot open serial device %s to head_mcu!",
                     params_.devicename.c_str());
        return;
      }

      struct termios tio;
      tcgetattr(serial_fd, &tio);
      tio.c_cflag = CS8 | CLOCAL | CREAD;
      tio.c_iflag = IGNPAR;
      tio.c_oflag = 0;
      tio.c_lflag = 0;
      //tcflush(serial_fd, TCIFLUSH);
      cfsetispeed(&tio, params_.baud_rate);
      tcsetattr(serial_fd, TCSANOW, &tio);

      RCLCPP_INFO(this->get_logger(),
          "Opened device %s with baudrate %d", params_.devicename.c_str(),params_.baud_rate);

      stop_ = false;
      reading_funtion_ = std::thread(&Head_MCU_node::read_serial, this);

      timer_ = this->create_wall_timer(std::chrono::duration<double>(1./params_.publish_rate),
          [&](){out_->publish(state_);});

      RCLCPP_INFO(this->get_logger(),
          "created timer for publisher at a period of %lfs", 1./params_.publish_rate);
    }

  ~Head_MCU_node() {
    ::close(serial_fd); // then read returns
    timer_->cancel();
    stop_ = true;
    if(reading_funtion_.joinable()) {
      reading_funtion_.join();
    }
  }

  private:

  //std::mutex state_m_; // no mutex needed, as newer data is always better
  std::atomic<bool> stop_;
  std::thread reading_funtion_;

  void set_update_period_of_target(head_mcu::UpdatePeriodMs period_ms) {
    if(::write(serial_fd, &period_ms, sizeof(head_mcu::UpdatePeriodMs)) < 0){
      RCLCPP_ERROR(this->get_logger(),
          "could not set update period of %d ms", period_ms);
    }
    RCLCPP_INFO(this->get_logger(),
        "Successfully set update period of %d ms", period_ms);

  }

  void read_serial() {
    RCLCPP_INFO(this->get_logger(),
        "serial connection thread started");

    set_update_period_of_target(ceil(1000/params_.publish_rate));

    while(!stop_) {
      head_mcu::Frame frame;
      // todo: regarding sync, maybe add a preamble to return to sync once lost
      int ret = ::read(serial_fd, &frame, sizeof(head_mcu::Frame));
      if(ret == sizeof(head_mcu::Frame)) {
        auto& s = state_.sensors;
        s.steering_angle_raw = frame.analog0;
        s.endstop_l ^= frame.digital0_8.as_bit.bit0;
        s.endstop_r ^= frame.digital0_8.as_bit.bit1;
        state_.stamp = this->now();
      } else if(ret > 0) {
        RCLCPP_WARN(this->get_logger(),
            "serial connection out of sync!");
      } else {
        RCLCPP_ERROR(this->get_logger(),
            "serial connection closed or something: %d", ret);

        std::this_thread::sleep_for(std::chrono::milliseconds(500/params_.publish_rate));
      }
    }
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //todo: reading thread
  rclcpp::spin(std::make_shared<Head_MCU_node>());
  rclcpp::shutdown();
  return 0;
}
