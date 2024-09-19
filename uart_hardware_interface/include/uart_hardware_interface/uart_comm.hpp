
#ifndef UART_HARDWARE_INTERFACE__UART_COMM_HPP_
#define UART_HARDWARE_INTERFACE__UART_COMM_HPP_

#include <string>
#include <termios.h>

namespace uart_hardware_interface
{

class UARTComm
{
public:
  UARTComm(const std::string &port_name, int baud_rate);
  ~UARTComm();

  bool openPort();
  void closePort();
  bool writeData(const std::string &data);
  bool writeData(const uint8_t *data, size_t size);

  bool isOpen() const;

private:
  std::string port_name_;
  int baud_rate_;
  int fd_; // File descriptor for the serial port
  struct termios tty_; // Terminal control structure
  bool is_open_;

  bool configurePort();
};

} // namespace uart_hardware_interface

#endif // UART_HARDWARE_INTERFACE__UART_COMM_HPP_
