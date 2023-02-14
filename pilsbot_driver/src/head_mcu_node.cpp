#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <pilsbot_driver_msgs/msg/steering_axle_sensors_stamped.hpp>

#include "./head_mcu/include/data_frame.hpp"
#include <pilsbot_driver/linear_interpolation.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>


using namespace std::chrono_literals;
using namespace pilsbot_driver_msgs::msg;
using linear_interpolator::CalibrationListSerialized;

class Head_MCU_node : public rclcpp::Node
{
  struct Parameter {
    std::string devicename;
    unsigned baud_rate;
    unsigned publish_rate;
  } params_;

  typedef LinearInterpolator<unsigned, double> PotInterpolator;
  PotInterpolator interpolator_;

  int serial_fd_ = -1;
  std::atomic<bool> stop_;

  SteeringAxleSensorsStamped state_;
  std::mutex state_m_;
  bool state_updated_;
  std::condition_variable state_updated_cv_;

  std::thread reading_function_;
  std::thread publishing_function_;


  rclcpp::Publisher<SteeringAxleSensorsStamped>::SharedPtr out_;

  public:
  Head_MCU_node() : Node("head_mcu_node")
    {
      this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud_rate", 115200);
      this->declare_parameter<int>("publish_rate", 10);
      // TODO: Add publisher for MCP portexpander to enable/disable power to board
      this->declare_parameter<CalibrationListSerialized>("calibration_val",
          CalibrationListSerialized());

      out_ = this->create_publisher<SteeringAxleSensorsStamped>("/head_mcu", 10);

      params_.devicename = this->get_parameter("serial_port").as_string();
      params_.baud_rate = this->get_parameter("baud_rate").as_int();
      params_.publish_rate = this->get_parameter("publish_rate").as_int();
      assert(params_.publish_rate > 0);

      if(!interpolator_.deserialize(this->get_parameter("calibration_val").as_double_array())) {
        RCLCPP_ERROR(this->get_logger(), "calibration values could not be loaded");
      }

      if(!open_serial_port()) {
        throw std::runtime_error("Could not open serial port " + params_.devicename);
      }

      set_serial_properties();
      RCLCPP_INFO(this->get_logger(),
          "Opened device %s with baudrate %d", params_.devicename.c_str(),params_.baud_rate);

      stop_ = false;
      publishing_function_ = std::thread(&Head_MCU_node::publish_state, this);
      reading_function_ = std::thread(&Head_MCU_node::read_serial, this);
    }

  ~Head_MCU_node() {
    stop();
  }

  void stop() {
    ::close(serial_fd_); // then read returns
    stop_ = true;
    state_updated_cv_.notify_all(); // consumer may wait on update
    if(reading_function_.joinable()) {
      reading_function_.join();
    }
    if(publishing_function_.joinable()) {
      publishing_function_.join();
    }
  }

  private:

  bool open_serial_port() {
    if ((serial_fd_ = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY)) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Cannot open serial device %s to head_mcu!",
                   params_.devicename.c_str());
      return false;
    }
    return true;
  }

  void set_serial_properties() {
    tcflush(serial_fd_, TCIOFLUSH); // flush previous bytes

    struct termios tio;
    if(tcgetattr(serial_fd_, &tio) < 0)
      perror("tcgetattr");

    tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tio.c_oflag &= ~(ONLCR | OCRNL);
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    switch (params_.baud_rate)
    {
    case 9600:   cfsetospeed(&tio, B9600);   break;
    case 19200:  cfsetospeed(&tio, B19200);  break;
    case 38400:  cfsetospeed(&tio, B38400);  break;
    case 115200: cfsetospeed(&tio, B115200); break;
    case 230400: cfsetospeed(&tio, B230400); break;
    case 460800: cfsetospeed(&tio, B460800); break;
    case 500000: cfsetospeed(&tio, B500000); break;
    default:
      RCLCPP_WARN(this->get_logger(),
          "Baudrate of %d not supported, using 115200!", params_.baud_rate);
      cfsetospeed(&tio, B115200);
      break;
    }
    cfsetispeed(&tio, cfgetospeed(&tio));

    if(tcsetattr(serial_fd_, TCSANOW, &tio) < 0) {
      RCLCPP_ERROR(this->get_logger(),
          "Could not set terminal attributes!");
      perror("tcsetattr");
    }
  }

  void set_update_period_of_target(head_mcu::UpdatePeriodMs period_ms) {
    if(::write(serial_fd_, &period_ms, sizeof(head_mcu::UpdatePeriodMs)) < 0){
      RCLCPP_ERROR(this->get_logger(),
          "could not set update period of %d ms", period_ms);
    }
    RCLCPP_INFO(this->get_logger(),
        "Probably successfully set update period of %d ms", period_ms);
  }

  void read_serial() {

    set_update_period_of_target(ceil(1000/params_.publish_rate));

    RCLCPP_INFO(this->get_logger(),
        "serial connection thread started");
    while(!stop_) {
      head_mcu::Frame frame;
      int ret = ::read(serial_fd_, &frame, sizeof(head_mcu::Frame));
      if(ret == sizeof(head_mcu::Frame)) {
        auto& s = state_.sensors;
        s.steering_angle_raw = frame.analog0;
        s.endstop_l = frame.digital0_8.as_bit.bit0;
        s.endstop_r = frame.digital0_8.as_bit.bit1;
        state_.stamp = this->now();

        {
          std::lock_guard<std::mutex> lk(state_m_); // why exacyly this?
          state_updated_ = true;
        }
        state_updated_cv_.notify_one();  // might as well be "all", we just have one consumer

      } else if(ret > 0) {
        RCLCPP_WARN(this->get_logger(),
            "serial connection out of sync!");
        // todo: maybe reopen serial, this resets the controller
      } else {
        RCLCPP_ERROR(this->get_logger(),
            "serial connection closed or something: %d", ret);
        return;
      }
    }
  }

  void publish_state() {
    auto local_state = state_;
    while(!stop_) {
      {
        // Wait until new data was received via serial
        std::unique_lock<std::mutex> lk(state_m_);
        state_updated_cv_.wait(lk, [&]{return state_updated_ || stop_;});
        if(stop_)
          return;

        local_state = state_;

        state_updated_ = false; //consumed info
        //unlock happens implicitly
      }

      local_state.sensors.steering_angle_normalized =
          interpolator_(local_state.sensors.steering_angle_raw);
      out_->publish(local_state);

    }
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Head_MCU_node>();

  /*
  signal(SIGINT, [](int sig){
    std::cout << "received signal " << sig << ", stopping." << std::endl;
    node->stop();}
  );
  */

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
