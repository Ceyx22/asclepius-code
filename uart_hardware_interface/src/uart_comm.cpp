
#include "uart_hardware_interface/uart_comm.hpp"
#include <fcntl.h>      // File control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <errno.h>      // Error number definitions
#include <string.h>     // String function definitions
#include <iostream>

namespace uart_hardware_interface
{

UARTComm::UARTComm(const std::string &port_name, int baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate), fd_(-1), is_open_(false)
{
}

UARTComm::~UARTComm()
{
  closePort();
}

bool UARTComm::openPort()
{
  fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1)
  {
    std::cerr << "Error opening port " << port_name_ << ": " << strerror(errno) << std::endl;
    return false;
  }
  else
  {
    fcntl(fd_, F_SETFL, 0);
  }

  if (!configurePort())
  {
    close(fd_);
    fd_ = -1;
    return false;
  }

  is_open_ = true;
  return true;
}

void UARTComm::closePort()
{
  if (is_open_)
  {
    close(fd_);
    fd_ = -1;
    is_open_ = false;
  }
}

bool UARTComm::configurePort()
{
  if (tcgetattr(fd_, &tty_) != 0)
  {
    std::cerr << "Error getting port attributes: " << strerror(errno) << std::endl;
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (baud_rate_)
  {
  case 9600:
    baud = B9600;
    break;
  case 19200:
    baud = B19200;
    break;
  case 38400:
    baud = B38400;
    break;
  case 57600:
    baud = B57600;
    break;
  case 115200:
    baud = B115200;
    break;
  default:
    std::cerr << "Unsupported baud rate: " << baud_rate_ << std::endl;
    return false;
  }

  cfsetospeed(&tty_, baud);
  cfsetispeed(&tty_, baud);

  // 8N1 Mode
  tty_.c_cflag &= ~PARENB;    // No parity
  tty_.c_cflag &= ~CSTOPB;    // 1 stop bit
  tty_.c_cflag &= ~CSIZE;
  tty_.c_cflag |= CS8;        // 8 data bits

  tty_.c_cflag &= ~CRTSCTS;   // No hardware flow control
  tty_.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

  // Non-canonical mode
  tty_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Disable software flow control
  tty_.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Raw output
  tty_.c_oflag &= ~OPOST;

  // Set read timeout
  tty_.c_cc[VMIN] = 0;
  tty_.c_cc[VTIME] = 10; // 1-second read timeout

  // Flush port, then apply attributes
  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
  {
    std::cerr << "Error setting port attributes: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool UARTComm::writeData(const std::string &data)
{
  if (!is_open_)
  {
    std::cerr << "Port is not open" << std::endl;
    return false;
  }

  ssize_t bytes_written = write(fd_, data.c_str(), data.length());
  if (bytes_written < 0)
  {
    std::cerr << "Error writing to port: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool UARTComm::writeData(const uint8_t *data, size_t size)
{
  if (!is_open_)
  {
    std::cerr << "Port is not open" << std::endl;
    return false;
  }

  ssize_t bytes_written = write(fd_, data, size);
  if (bytes_written < 0)
  {
    std::cerr << "Error writing to port: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool UARTComm::isOpen() const
{
  return is_open_;
}

} // namespace uart_hardware_interface
