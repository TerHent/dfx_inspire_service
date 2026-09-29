#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(const std::string& port, speed_t baudrate, int timeout_ms = 2)
  : fd_(-1)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    close_port();
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  void close_port()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  ssize_t send(const uint8_t* data, size_t len)
  {
    if (fd_ < 0) return -1;
    tcflush(fd_, TCIFLUSH); // Discard stale bytes in RX buffer before sending
    ssize_t ret = ::write(fd_, data, len);
    return ret;
  }

  ssize_t recv(uint8_t* data, size_t len)
  {
    if (fd_ < 0) return -1;
    ssize_t total_read = 0;
    while (total_read < len)
    {
      FD_ZERO(&rSet_);
      FD_SET(fd_, &rSet_);
      struct timeval tv = timeout_;

      int sel_ret = select(fd_ + 1, &rSet_, NULL, NULL, &tv);
      if (sel_ret < 0) {
        return -1; // error
      }
      else if (sel_ret == 0) {
        break; // timeout, return what we got so far
      }

      ssize_t read_bytes = ::read(fd_, data + total_read, len - total_read);
      if (read_bytes <= 0) {
        break; // error or EOF
      }
      total_read += read_bytes;
    }
    return total_read;
  }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  bool Init(const std::string& port, speed_t baudrate)
  {
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      return false;
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    int ret = tcgetattr(fd_, &option);
    if (ret != 0)
    {
      close(fd_);
      fd_ = -1;
      return false;
    }

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
    if (ret != 0)
    {
      close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  int fd_ = -1;
  fd_set rSet_;
  timeval timeout_;

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

#endif // SERIAL_PORT_H