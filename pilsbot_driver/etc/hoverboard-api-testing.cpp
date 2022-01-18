#include "HoverboardAPI.h"
#include "../src/pid-controller/pid.h"
#include "quick_arg_parser.hpp"

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

double target_speed_l = 0;   //mm/s
double target_speed_r = 0;   //mm/s

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

void setSpeedTest(HoverboardAPI* api, PID::Settings& pid_settings)
{
  if(!isnan(pid_settings.Kp)) {
    cout << "using provided PIDs" << endl;
    // directly send configured PID values
    api->sendPIDControl(pid_settings.Kp,
                        pid_settings.Ki,
                        pid_settings.Kd,
                        10);
  } else {
    // directly send configured PID values
    api->sendPIDControl(200,
                        1,
                        1,
                        10);
  }


  while(1){
    api->requestRead(HoverboardAPI::Codes::sensHall);
    api->requestRead(HoverboardAPI::Codes::sensElectrical);

    unsigned char c;
    int i = 0, r = 0;
    while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length) {
      api->protocolPush(c);
    }

    api->sendSpeedData(target_speed_l, target_speed_r, 600, 1);
    api->protocolTick();

    cout << "Voltage: " << api->getBatteryVoltage() << "V, Current Speeds: " << api->getSpeed0_mms() << "mm/s, " << api->getSpeed1_mms() << "mm/s\r";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

//i know, ugly, but this is only a test
bool scheduleRead = false;
void pwmPIDSpeedTest(HoverboardAPI* api, PID::Settings& pid_settings)
{
  PID left, right;
  auto settings = PID::getDefault();
  settings.Kp = .25; settings.Ki = 1; settings.Kd = .01;
  settings.max = 300; //min/max is PWM (0-1000)
  settings.max_dv = 200; settings.overshoot_integral_adaptation = .5;

  if(!isnan(pid_settings.Kp)) {
    cout << "Using provided PID values" << endl;
    settings = pid_settings;
  }

  auto last_tick = std::chrono::system_clock::now();

  std::chrono::duration<double> target_delta_t = 50ms;

  if(scheduleRead) {
  //Period is in ms
    unsigned period_ms = target_delta_t.count()*(1000/2);
    cout << "Scheduling Hall info to read every " << period_ms << "ms" << endl;
    api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, period_ms); //, PROTOCOL_SOM_ACK);
  }

  while(1){
    unsigned char c;
    int i = 0, r = 0;
    while ((r = ::read(hoverboard_fd, &c, 1)) > 0 && i++ < max_length) {
      api->protocolPush(c);
    }

    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> delta_t = now - last_tick;

    if(delta_t >= target_delta_t) {

      double actual_speed_l = api->getSpeed0_mms();
      double actual_speed_r = api->getSpeed1_mms();

      last_tick = now;
      settings.dt = delta_t.count();

      int set_pwm_l = left.calculate(target_speed_l, actual_speed_l, settings);
      int set_pwm_r = right.calculate(target_speed_r, actual_speed_r, settings);

      cout << "Delta t: " << setw(9) << std::left << delta_t.count() << "s " << std::right <<
              "Speed l " << setw(5) << actual_speed_l << "mm/s (target " << target_speed_l << ", pwm " << setw(4) << set_pwm_l << ") "
              "Speed r " << setw(5) << actual_speed_r << "mm/s (target " << target_speed_r << ", pwm " << setw(4) << set_pwm_r << ")\n";
      api->sendDifferentialPWM(set_pwm_r, set_pwm_l, PROTOCOL_SOM_NOACK);   //!!!!

      if(!scheduleRead) {
        api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);
      }
    }

    api->protocolTick();
  }
}

void pwmPIDPositionTest(HoverboardAPI* api, PID::Settings& pid_settings)
{
  PID left, right;
  auto settings = PID::getDefault();
  settings.Kp = .15; settings.Ki = .5; settings.Kd = .01;
  settings.max = 250; //min/max is PWM (0-1000)
  settings.max_dv = 50;     // strongly dampening due to twitchy speed readout

  if(!isnan(pid_settings.Kp)) {
    cout << "Using provided PID values" << endl;
    cout << "Kp: " << pid_settings.Kp << endl;
    settings = pid_settings;
  }

  auto last_tick = std::chrono::system_clock::now();
  double last_pos_mm_l = api->getPosition1_mm();
  double last_pos_mm_r = api->getPosition0_mm();

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

    if(delta_t >= target_delta_t)
    {
      if(!has_requested_this_round) {
        api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);
        //api->requestRead(HoverboardAPI::Codes::sensElectrical, PROTOCOL_SOM_NOACK);
        has_requested_this_round = true;
      } else {
        // waiting for the update to come in
        double curr_pos_mm_l = api->getPosition0_mm();
        double curr_pos_mm_r = api->getPosition1_mm();

        if(curr_pos_mm_l != last_pos_mm_l ||
           curr_pos_mm_r != last_pos_mm_r ||
           delta_t >= target_delta_t*2 ){

          last_tick = now;
          settings.dt = delta_t.count();

          double actual_speed_l = (curr_pos_mm_l - last_pos_mm_l) / delta_t.count();
          double actual_speed_r = (curr_pos_mm_r - last_pos_mm_r) / delta_t.count();

          int set_pwm_l = left.calculate(target_speed_l, actual_speed_l, settings);
          int set_pwm_r = right.calculate(target_speed_r, actual_speed_r, settings);

          cout << "Delta t: " << setw(9) << delta_t.count() << "s " <<
                  "Speed l " << setw(8) << actual_speed_l << "mm/s (target " << target_speed_l << ", pwm " << setw(7) << set_pwm_l << ") "
                  "Speed r " << setw(8) << actual_speed_r << "mm/s (target " << target_speed_r << ", pwm " << setw(7) << set_pwm_r << ")\n";
          api->sendDifferentialPWM(set_pwm_r, set_pwm_l, PROTOCOL_SOM_NOACK); //!!!!

          last_pos_mm_l = curr_pos_mm_l;
          last_pos_mm_r = curr_pos_mm_r;
          has_requested_this_round = false;
        }
      }
    }


    api->protocolTick();
  }
}

struct Args : MainArguments<Args> {
    std::string serial = option("port", 'p', "hoverbaord port") = "/dev/ttyHoverboard";
    std::string test = option("test", 't', "One of the tests") = "speed";
    vector<double> pid = option("pid", '\0', "PID values 1,2,3") = vector<double>{};
    double speed_l = option("speed_l", 'l', "Speed L in mm/s") = 300;
    double speed_r = option("speed_r", 'r', "Speed R in mm/s") = 100;
    bool schedule = option("schedule", 's', "if to use scheduling or active polling") = false;
};

int main(int argc, char** argv) {
  Args args{{argc, argv}};

  target_speed_l = args.speed_l;   //mm/s
  target_speed_r = args.speed_r;   //mm/s
  scheduleRead = args.schedule;

  setup_serial(args.serial);
  auto api = new HoverboardAPI(serialWrite);

  PID::Settings pid_settings; pid_settings.Kd = NAN;

  if(args.pid.size() != 3) {
    cout << "No/wrong PID format: using test default PIDs" << endl;
  } else {
    pid_settings = PID::getDefault();
    pid_settings.Kp = args.pid[0]; pid_settings.Ki = args.pid[1];
    pid_settings.Kd = args.pid[2]; pid_settings.max = 250; //min/max is PWM (0-1000)
    pid_settings.max_dv = 50;
  }

  if(args.test == "hoverboard") {
    cout << "running hoverboard test" << endl;
    setSpeedTest(api, pid_settings);
  }
  else if (args.test == "speed"){
    cout << "running own PID controller test (using speed sensor)" << endl;
    pwmPIDSpeedTest(api, pid_settings);
  }
  else if (args.test == "position"){
    cout << "running own PID controller test (using position sensor)" << endl;
    pwmPIDPositionTest(api, pid_settings);
  }
  else {
    cerr << "Invalid test " << args.test << endl;
    cerr << "Choose one of:" << endl;
    cerr << "\thoverboard" << endl;
    cerr << "\tspeed" << endl;
    cerr << "\tposition" << endl;
  }

  if (hoverboard_fd != -1)
    close(hoverboard_fd);
  delete api;
}

