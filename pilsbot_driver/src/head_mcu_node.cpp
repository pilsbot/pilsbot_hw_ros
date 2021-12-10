#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pilsbot_driver_msgs/msg/steering_axle_sensors_stamped.hpp"


#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


using namespace std::chrono_literals;
using namespace pilsbot_driver_msgs::msg;

class Head_MCU_node : public rclcpp::Node
{
  static constexpr char PREFIX[] = "/head_mcu";

  typedef std::vector<double> CalibrationList;  // interpreted as pairs
  struct Parameter {
    CalibrationList calibration;
    std::string devicename;
    unsigned publish_rate;
  } params_;

  int serial_fd = -1;

  SteeringAxleSensorsStamped state_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<SteeringAxleSensorsStamped>::SharedPtr out_;

  public:
  Head_MCU_node() : Node("sensor_normalizer")
    {
      this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
      this->declare_parameter<int>("publish_rate", 100);
      this->declare_parameter<CalibrationList>("calibration_val",CalibrationList());


      out_ = this->create_publisher<SteeringAxleSensorsStamped>("/head_mcu", 10);

      params_.devicename = this->get_parameter("serial_port").as_string();
      params_.publish_rate = this->get_parameter("publish_rate").as_int();
      assert(params_.publish_rate > 0);
      //params_.calibration = this->get_parameter("calibration_val").as_what?

      if ((serial_fd = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot open serial device %s to head_mcu!",
                     params_.devicename);
        return;
      }

      struct termios tio;
      tcgetattr(serial_fd, &tio);
      tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
      tio.c_iflag = IGNPAR;
      tio.c_oflag = 0;
      tio.c_lflag = 0;
      tcflush(serial_fd, TCIFLUSH);
      tcsetattr(serial_fd, TCSANOW, &tio);


      timer_ = this->create_wall_timer(std::chrono::duration<double>(1./params_.publish_rate),
          [&](){out_->publish(state_);});
    }

  private:

    void read_serial() {
      //todo async read into
      while(true) {
        auto& s = state_.sensors;
        s.steering_angle_raw = 10;
        s.endstop_l ^= 1;
        s.endstop_r ^= 1;
        state_.stamp = this->now();
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
