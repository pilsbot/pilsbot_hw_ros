#include "HoverboardAPI.h"
#include "../src/pid-controller/pid.h"

#include <string>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <math.h>


using namespace std;


int hoverboard_fd = -1;
unsigned constexpr serial_connect_retries = 10;
constexpr int max_length = 1024;

int serialWrite(unsigned char* data, int len)
{
  return ::write(hoverboard_fd, data, len);
}

void setup_serial(string tty_device)
{
  unsigned retries = 0;
  bool still_ho_unsuccessful = true;
  while(retries < 10) {
    if ((hoverboard_fd = ::open(tty_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      cerr << "Cannot open serial device "<< tty_device <<" to hoverboard_api" << endl;;
    } else {
      still_ho_unsuccessful = false;
    }
    if(still_ho_unsuccessful) {
      retries++;
      cout << "retrying connection ("<< retries << " of "<< serial_connect_retries<< ")" << endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } else {
      break;
    }
  }

  if(still_ho_unsuccessful) {
    cerr << "Could not open serial device "<<tty_device<<" to pilsbot controller board" << endl;
    return;
  }

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see
  // http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  struct termios options;
  tcgetattr(hoverboard_fd, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(hoverboard_fd, TCIFLUSH);
  tcsetattr(hoverboard_fd, TCSANOW, &options);
}

void setSpeedTest(HoverboardAPI* api)
{
  // directly send configured PID values
  api->sendPIDControl(200,
      1,
      1,
      10);

  double speedl = 500;   //mm/s
  double speedr = 200;   //mm/s

  while(1){
    api->requestRead(HoverboardAPI::Codes::sensHall);
    api->requestRead(HoverboardAPI::Codes::sensElectrical);

    unsigned char c;
    int i = 0, r = 0;
    while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length) {
      api->protocolPush(c);
    }

    api->sendSpeedData(speedl, speedr, 600, 1);
    api->protocolTick();

    cout << "Voltage: " << api->getBatteryVoltage() << "V, Current Speeds: " << api->getSpeed0_mms() << "mm/s, " << api->getSpeed1_mms() << "mm/s\r";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void pwmPIDTest(HoverboardAPI* api)
{
  PID left, right;
  auto settings = PID::Settings {
    .Kp = .15, .Ki = .5,  .Kd = .01,
    .dt = 1, .max = 250, .min = NAN //500 is PWM (0-1000)
  };

  auto last_tick = std::chrono::system_clock::now();

  double speed_l = 400;   //mm/s
  double speed_r = 60;   //mm/s
  std::chrono::duration<double> target_delta_t = 50ms;

  bool has_requested_this_round = false;
  while(1){
    unsigned char c;
    int i = 0, r = 0;
    while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length) {
      api->protocolPush(c);
    }
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> delta_t = now - last_tick;

    if(!has_requested_this_round && delta_t >= target_delta_t/2)
    {
      api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);
      //api->requestRead(HoverboardAPI::Codes::sensElectrical, PROTOCOL_SOM_NOACK);
      has_requested_this_round = true;
    }

    if(delta_t >= target_delta_t) {

      double actual_speed_l = api->getSpeed1_mms();
      double actual_speed_r = api->getSpeed0_mms();

      last_tick = now;
      settings.dt = delta_t.count();

      double set_pwm_l = left.calculate(speed_l, actual_speed_l, settings);
      double set_pwm_r = right.calculate(speed_r, actual_speed_r, settings);

      cout << "Delta t: " << setw(8) << delta_t.count() << "s " <<
              "Speed l " << actual_speed_l << "mm/s (target " << speed_l << ", pwm " << setw(7) << set_pwm_l << ") "
              "Speed r " << actual_speed_r << "mm/s (target " << speed_r << ", pwm " << setw(7) << set_pwm_l << ")\n";
      api->sendDifferentialPWM(set_pwm_l, set_pwm_r, PROTOCOL_SOM_NOACK);

      has_requested_this_round = false;
    }

    api->protocolTick();
  }
}

int main() {

  setup_serial("/dev/ttyHoverboard");

  auto api = new HoverboardAPI(serialWrite);

  //setSpeedTest(api);
  pwmPIDTest(api);

  if (hoverboard_fd != -1)
    close(hoverboard_fd);
  delete api;
}

