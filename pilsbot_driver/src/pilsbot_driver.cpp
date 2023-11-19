#include "HoverboardAPI.h"
#include "./head_mcu/include/data_frame.hpp"
#include <pilsbot_driver/pilsbot_driver.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>  //ntoh
#include <unistd.h>

int serialWrite(unsigned char* data, int len)
{
  return ::write(hoverboard_fd, data, len);
}

namespace pilsbot_driver
{
using std::placeholders::_1;

PilsbotDriver::~PilsbotDriver()
{
  stop();
  delete api;
}

std::vector<hardware_interface::StateInterface>
PilsbotDriver::export_state_interfaces()
{
  // at this point, we know there are just two joints (see on_configure)

  std::vector<hardware_interface::StateInterface> state_interfaces;

  bool have_single_right_wheel = false;
  bool have_single_left_wheel = false;
  bool have_single_steering_axle = false;

  for(auto& joint : info_.joints) {
    if(joint.name.find("left") != std::string::npos) {
      if(!have_single_left_wheel) {
        RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: interpreting joint %s as powered left wheel", joint.name.c_str());
        have_single_left_wheel = true;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: Ignoring additional left wheel %s", joint.name.c_str());
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &wheels_[0].curr_position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[0].curr_speed));

    }
    else if (joint.name.find("right") != std::string::npos) {
      if(!have_single_right_wheel) {
        RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: interpreting joint %s as powered right wheel", joint.name.c_str());
        have_single_right_wheel = true;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: Ignoring additional right wheel %s", joint.name.c_str());
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &wheels_[1].curr_position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[1].curr_speed));
    }
    else if (joint.name == "steering_axle_joint") { // TODO: Make this name configurable
      if(!have_single_steering_axle) {
        RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: interpreting joint %s as steering_axle joint", joint.name.c_str());
        have_single_right_wheel = true;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
             "StateIF: Ignoring additional steering_axle %s", joint.name.c_str());
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &axle_sensors_.steering_angle_normalized));
    }
  }

  // export sensor state interface
  for (auto& sens : info_.sensors)
  {
    if (sens.name == "hoverboard_api") {
      for(auto& interface : sens.state_interfaces) {
        if(interface.name == "voltage") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.voltage));
        }
        else if (interface.name == "avg_amperage_motor.0") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.avg_amperage_motor0));
        }
        else if (interface.name == "avg_amperage_motor.1") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.avg_amperage_motor1));
        }
        else if (interface.name == "tx_bufferlevel") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &hoverboard_sensors_.txBufferLevel));
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
               "Not offering interface %s in sensor %s",
               interface.name, sens.name);
        }
      }
    } else if (sens.name == "head_mcu") {
      for(auto& interface : sens.state_interfaces) {
        if(interface.name == "steering_angle_raw") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.steering_angle_raw));
        }
        else if (interface.name == "endstop_l") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.endstop_l));
        }
        else if (interface.name == "endstop_r") {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
               sens.name, interface.name, &axle_sensors_.endstop_r));
        }
        else {
          RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
               "Not offering interface %s in sensor %s",
               interface.name, sens.name);
        }
      }
    }else {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
           "Not offering sensor %s", sens.name);
    }
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
PilsbotDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  bool have_single_right_wheel = false;
  bool have_single_left_wheel = false;
  for (auto& joint : info_.joints)
  {
    if(joint.name.find("left") != std::string::npos) {
      if(!have_single_left_wheel) {
        RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
             "CommandIF: interpreting joint %s as powered left wheel", joint.name.c_str());
        have_single_left_wheel = true;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[0].commanded_turning_rate));
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
             "CommandIF: Ignoring additional left wheel %s", joint.name.c_str());
      }
    }
    else if (joint.name.find("right") != std::string::npos) {
      if(!have_single_right_wheel) {
        RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
             "CommandIF: interpreting joint %s as powered right wheel", joint.name.c_str());
        have_single_right_wheel = true;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &wheels_[1].commanded_turning_rate));
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
             "CommandIF: Ignoring additional right wheel %s", joint.name.c_str());
      }
    }
  }

  return command_interfaces;
}


hardware_interface::return_type PilsbotDriver::configure(const hardware_interface::HardwareInfo &info)
{
  clock = rclcpp::Clock();

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  else
  {

    // TODO: check if correct joints are passed in config
    wheels_.resize(2, WheelStatus());   // hardcoded: only support two controllable wheels
    hoverboard_sensors_ = HoverboardSensors();

    /*
     * TODO: Load parameters from Hardware-Info (URDF)
     * And for deep hardware params (serial port, baud, ...)
     * load a yaml via rclcpp yaml-parser referenced in urdf
     */
    params_ = Params();

    if(!interpolator_.deserialize(params_.head_mcu.calibration_val)) {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
             "calibration values could not be loaded!");
    }

    return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type PilsbotDriver::start()
{
  if (!try_setup_serial(WhichSerial::head_mcu))
  {
    return hardware_interface::return_type::ERROR;
  }
  if (!try_setup_serial(WhichSerial::hoverboard))
  {
    return hardware_interface::return_type::ERROR;
  }

  last_serial_read = clock.now();
  last_write_tick = clock.now();

  // ok, go on little bird
  stop_ = false;
  reading_function_ = std::thread(&PilsbotDriver::read_from_head_mcu, this);

  for (auto& wheel : wheels_)
  {
    wheel = WheelStatus();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::stop()
{
  stop_ = true;
  if (hoverboard_fd != -1)
    close(hoverboard_fd);
  if (head_mcu_fd != -1)
    close(head_mcu_fd);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PilsbotDriver::read()
{

  if (hoverboard_fd == -1) {
    RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("PilsbotDriver"), clock, 1000,
        "Filedescriptor to hoverboard invalid!");
    return hardware_interface::return_type::ERROR;
  }
  api->requestRead(HoverboardAPI::Codes::sensHall);
  api->requestRead(HoverboardAPI::Codes::sensElectrical);

  unsigned char c;
  int i = 0, r = 0;
  while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length) {
    api->protocolPush(c);
  }

  if (i > 0) {
    last_serial_read = clock.now();
  }

  if (r < 0 && errno != EAGAIN)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PilsbotDriver"), clock, 1000,
        "hoverboard: Reading from serial %s failed: %d",
        params_.hoverboard.tty_device.c_str(), r);
    perror("hoverboard read");

    //TODO: Is there a better way for controller to know this return value (ERROR)?
    axle_sensors_.steering_angle_normalized = NAN;
    return hardware_interface::return_type::ERROR;
  }

  if ((clock.now() - last_serial_read) > rclcpp::Duration(1, 0))
  {
    RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("PilsbotDriver"), clock, 1000,
        "hoverboard: Timeout reading from serial %s. Last connection: %d",
        params_.hoverboard.tty_device.c_str(), last_serial_read.seconds());
    //TODO: Is there a better way for controller to know this return value (ERROR)?
    axle_sensors_.steering_angle_normalized = NAN;
    return hardware_interface::return_type::ERROR;
  }

  // steering angle & co are updated in thread

  // sensory
  hoverboard_sensors_.voltage = api->getBatteryVoltage();
  hoverboard_sensors_.avg_amperage_motor0 = api->getMotorAmpsAvg(0);
  hoverboard_sensors_.avg_amperage_motor1 = api->getMotorAmpsAvg(1);
  hoverboard_sensors_.txBufferLevel = api->getTxBufferLevel();

  // Convert mm/s to rad/s
  double sens_speed0 = (api->getSpeed0_mms() / 1000.0);
  double sens_speed1 = (api->getSpeed1_mms() / 1000.0);

  // Basic sanity check, speed should be less than 10 m/s
  // Sometimes, it seems during EMI peaks, we're getting ridiculous values here
  // Don't know what to do with it, just ignoring for now
  if (fabs(sens_speed0) < 10 && fabs(sens_speed1) < 10) {
    wheels_[0].curr_speed = sens_speed0 / params_.wheel_radius;
    wheels_[0].curr_position = (api->getPosition0_mm() / 1000.0) / params_.wheel_radius;
    wheels_[1].curr_speed = sens_speed1 / params_.wheel_radius;
    wheels_[1].curr_position = (api->getPosition1_mm() / 1000.0) / params_.wheel_radius;
    return hardware_interface::return_type::OK;
  }
  else
    return hardware_interface::return_type::ERROR;

}

hardware_interface::return_type PilsbotDriver::write()
{
  if (hoverboard_fd == -1) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PilsbotDriver"), clock, 1000,
        "hoverboard: Attempt to write on closed serial");
    return hardware_interface::return_type::ERROR;
  }

  auto& pid_settings = params_.hoverboard.pid;

  auto delta_t = clock.now() - last_write_tick;
  pid_settings.dt = delta_t.seconds();

  // Convert rad/s to mm/s
  double target_speed_l = wheels_[0].commanded_turning_rate * params_.wheel_radius * 1000;
  double target_speed_r = wheels_[1].commanded_turning_rate * params_.wheel_radius * 1000;

  double actual_speed_l = api->getSpeed0_mms();
  double actual_speed_r = api->getSpeed1_mms();

  if(axle_sensors_.endstop_l || axle_sensors_.endstop_r) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PilsbotDriver"), clock, 1000,
        "Hit Endstop! If this happens frequently, consider stop doing that.");
    wheel_controller_l = PID();
    wheel_controller_r = PID();
  }

  int set_pwm_l = wheel_controller_l.calculate(target_speed_l, actual_speed_l, pid_settings);
  int set_pwm_r = wheel_controller_r.calculate(target_speed_r, actual_speed_r, pid_settings);

  // note l and r are switched!!!!
  api->sendDifferentialPWM(set_pwm_r, set_pwm_l, PROTOCOL_SOM_NOACK);

  api->protocolTick();
  last_write_tick = clock.now();

  return hardware_interface::return_type::OK;
}

// TODO: call this with external, timed, thread
void PilsbotDriver::tick()
{
  api->protocolTick();
}


bool PilsbotDriver::try_setup_serial(const PilsbotDriver::WhichSerial which)
{
  unsigned retries = 0;
  bool still_unsuccessful = true;

  int* fd = nullptr;
  const char* path = nullptr;
  const char* name = nullptr;

  switch (which)
  {
    case WhichSerial::hoverboard:
      fd = &hoverboard_fd;
      path = params_.hoverboard.tty_device.c_str();
      name = "pilsbot motor control board";
      break;
    case WhichSerial::head_mcu:
      fd = &head_mcu_fd;
      path = params_.head_mcu.tty_device.c_str();
      name = "head mcu steering angle sensor board";
      break;
    default:
      throw new std::runtime_error("Invalid serial device to setup");
  }

  while(retries < params_.serial_connect_retries)
  {
    if ((*fd = ::open(path, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
                   "Cannot open serial device %s to hoverboard_api",
                   path);
    } else {
      still_unsuccessful = false;
    }

    if(still_unsuccessful) {
      retries++;
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
                   "retrying connection for %s (%d of %d)",
                   name, retries, params_.serial_connect_retries);

      if (which == WhichSerial::hoverboard)
      {
        // We are trying to connect to hoverboard, and it did not succeed.
        // let's try to be smart about it.
        if (head_mcu_fd) // ASSUME head_mcu connection was successful
        {
          set_head_mcu_pin(true);
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          set_head_mcu_pin(false);
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      // success
      break;
    }
  }

  if(still_unsuccessful) {
    RCLCPP_FATAL(rclcpp::get_logger("PilsbotDriver"),
                 "Could not open serial device %s to %s",
                 path, name);
    if (*fd != -1)
      ::close(*fd);
    return false;
  }

  // we found and opened the thing

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see
  // http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):

  if (which == WhichSerial::hoverboard)
  {
    // HB specific
    struct termios options;
    tcgetattr(*fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(*fd, TCIFLUSH);
    if(tcsetattr(*fd, TCSANOW, &options) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
          "hoverboard: Could not set terminal attributes!");
      perror("tcsetattr");
    }

    api = new HoverboardAPI(serialWrite);
  }
  else if (which == WhichSerial::head_mcu)
  {
    // head_mcu specific
    // configuring head_mcu UART
    tcflush(*fd, TCIOFLUSH); // flush previous bytes

    struct termios tio;
    if(tcgetattr(*fd, &tio) < 0)
      perror("head_mcu tcgetattr");

    tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tio.c_oflag &= ~(ONLCR | OCRNL);
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    switch (params_.head_mcu.baudrate)
    {
    case 9600:   cfsetospeed(&tio, B9600);   break;
    case 19200:  cfsetospeed(&tio, B19200);  break;
    case 38400:  cfsetospeed(&tio, B38400);  break;
    case 115200: cfsetospeed(&tio, B115200); break;
    case 230400: cfsetospeed(&tio, B230400); break;
    case 460800: cfsetospeed(&tio, B460800); break;
    case 500000: cfsetospeed(&tio, B500000); break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
          "head_mcu: Baudrate of %d not supported, using 115200!", params_.head_mcu.baudrate);
      cfsetospeed(&tio, B115200);
      break;
    }
    cfsetispeed(&tio, cfgetospeed(&tio));

    if(tcsetattr(*fd, TCSANOW, &tio) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
          "head_mcu: Could not set terminal attributes!");
      perror("tcsetattr");
    }
  }
  return true;
}

bool PilsbotDriver::set_update_period_of_head_mcu(head_mcu::UpdatePeriodMs period_ms)
{
  head_mcu::Command cmd;
  memset(&cmd, 0, sizeof(decltype(cmd)));

  cmd.magic = head_mcu::Command::MAGIC;
  cmd.type = head_mcu::Command::setUpdatePeriod;
  cmd.updatePeriod_ms = ::htons(period_ms);

  if(::write(head_mcu_fd, &cmd, sizeof(decltype(cmd))) < 0){
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
        "could not set update period of %d ms", period_ms);
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
      "Probably successfully set head mcu update period of %d ms", period_ms);
  return true;
}

bool PilsbotDriver::set_head_mcu_pin(const bool val)
{
  head_mcu::Command cmd;
  memset(&cmd, 0, sizeof(decltype(cmd)));

  cmd.magic = head_mcu::Command::MAGIC;
  cmd.type = head_mcu::Command::setOutputFrame;
  cmd.frame.digital0_8.as_bit.bit2 = val;

  if(::write(head_mcu_fd, &cmd, sizeof(decltype(cmd))) < 0){
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
        "could not set pin to %s", val ? "on" : "off");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
      "Probably successfully set pin to %s", val ? "on" : "off");
  return true;
}

void PilsbotDriver::read_from_head_mcu()
{
  if(!set_update_period_of_head_mcu(params_.head_mcu.update_period_ms)){
    RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
        "Head_mcu: could not set update period of %d ms", params_.head_mcu.update_period_ms);
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("PilsbotDriver"),
      "head_mcu serial connection thread started");
  while(!stop_)
  {
    head_mcu::Frame frame;
    auto ret = ::read(head_mcu_fd, &frame, sizeof(head_mcu::Frame));
    if(ret == sizeof(head_mcu::Frame))
    {
      // Assume a valid frame
      axle_sensors_.steering_angle_raw = frame.analog0;
      axle_sensors_.endstop_l = frame.digital0_8.as_bit.bit0;
      axle_sensors_.endstop_r = frame.digital0_8.as_bit.bit1;
      axle_sensors_.steering_angle_normalized =
            interpolator_(frame.analog0);
    }
    else
    {
      if(ret > 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("PilsbotDriver"),
            "Head_mcu: serial connection out of sync!");
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("PilsbotDriver"),
          "Head_mcu: serial connection closed or something: %d", errno);
      }
      // restart serial connection, perhaps this helps.
      if (!stop_)
      {
        close(head_mcu_fd); // re-connect should re-start controller
        try_setup_serial(WhichSerial::head_mcu);
      }
    }
  }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pilsbot_driver::PilsbotDriver,
  hardware_interface::SystemInterface
)
