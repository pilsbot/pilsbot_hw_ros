#include "HoverboardAPI.h"

#include <string>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>


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

int main() {

  setup_serial("/dev/ttyHoverboard");

  auto api = new HoverboardAPI(serialWrite);

  // directly send configured PID values
  api->sendPIDControl(200,
      1,
      1,
      10);

  double speedl = 50;   //mm/s
  double speedr = 50;   //mm/s

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


  if (hoverboard_fd != -1)
    close(hoverboard_fd);
  delete api;
}

