#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 
#include <errno.h>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "mymsg_msgs/msg/uwb.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SensorReal: public rclcpp::Node
{
  public:
    SensorReal(int ser)
    : Node("sensor_real")
    {
      publisher_ = this->create_publisher<mymsg_msgs::msg::Uwb>("/my_topic/range_aoa", 10);
      this->aoa = 0.0;
      this->range = 1.0;
      this->ser = ser;
      timer_ = this->create_wall_timer(
      100ms, std::bind(&SensorReal::timer_callback, this));
    }
    // Allocate memory for read buffer, set size according to your needs
    char read_buf [256];
    float aoa;
    float range;
    int ser;

  private:
    void timer_callback()
    {
      read(ser, &this->read_buf, sizeof(this->read_buf));
      char aoadel[] = "LAoA_deg\":";
      char rangedel[] = "D_cm\":";
      int lenad = strlen(aoadel);
      int lenrd = strlen(rangedel); 
      char* oa_Ptr;
      char* or_Ptr;
      auto msg = mymsg_msgs::msg::Uwb();

      oa_Ptr = strstr(read_buf, aoadel);
      or_Ptr = strstr(read_buf, rangedel);

      if (oa_Ptr != NULL and or_Ptr != NULL) {
        float aoa_st = atof(strtok(oa_Ptr + lenad,","));
        float range_st = atof(strtok(or_Ptr + lenrd,","));
        if (aoa_st != NULL and range_st != NULL) {
          this->aoa = aoa_st;
          this->range = range_st + 10.0;
          msg.aoa = this->aoa * M_PI / 180;
          msg.range = this->range / 100;
          RCLCPP_INFO(this->get_logger(), "Range: %.2f, aoa %.2f", this->range, this->aoa);
          publisher_->publish(msg);
        }
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mymsg_msgs::msg::Uwb>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  system("sudo /home/giorgio/usbch.sh");
  int ser = open("/dev/ttyACM0", O_RDWR);
  

  // Check for errors
  if (ser < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  struct termios tty;

  if(tcgetattr(ser, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(ser, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Write to serial port
  unsigned char msg[] = "initf 4 2400 100 \r\n";
  write(ser, msg, sizeof(msg));

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorReal>(ser));
  rclcpp::shutdown();

  return 0;
}
