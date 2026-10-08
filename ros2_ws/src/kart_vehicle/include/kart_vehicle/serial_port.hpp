#pragma once
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
namespace kart_vehicle {
class SerialPort {
public:
  explicit SerialPort(const std::string & path) {
    fd_=::open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (fd_<0) {fail("open");}
    try {
      if (::ioctl(fd_,TIOCEXCL)<0) {fail("exclusive serial access");}
      termios tty{};
      if (::tcgetattr(fd_,&tty)<0) {fail("tcgetattr");}
      ::cfmakeraw(&tty); tty.c_cflag |= CLOCAL|CREAD;
#ifdef CRTSCTS
      tty.c_cflag &= ~CRTSCTS;
#endif
      ::cfsetispeed(&tty,B115200); ::cfsetospeed(&tty,B115200);
      tty.c_cc[VMIN]=0; tty.c_cc[VTIME]=0;
      if (::tcsetattr(fd_,TCSANOW,&tty)<0 || ::tcflush(fd_,TCIOFLUSH)<0) {fail("configure serial");}
      write_line("\n"); // Terminate any incomplete host frame from a previous connection.
    } catch (...) {::close(fd_); fd_=-1; throw;}
  }
  ~SerialPort() {if(fd_>=0) {::ioctl(fd_,TIOCNXCL); ::close(fd_);}}
  SerialPort(const SerialPort &)=delete;
  SerialPort & operator=(const SerialPort &)=delete;
  void write_line(const std::string & line) {
    const auto count=::write(fd_,line.data(),line.size());
    if (count<0) {fail("serial write");}
    if (static_cast<size_t>(count)!=line.size()) {throw std::runtime_error("partial serial write");}
  }
  std::vector<std::string> read_lines() {
    char data[512];
    // Bound CPU and storage even if the device sends malformed continuous data.
    for (int i=0;i<16;++i) {
      const auto n=::read(fd_,data,sizeof(data));
      if (n<0) {if(errno==EAGAIN || errno==EWOULDBLOCK) {break;} fail("serial read");}
      if (n==0) {break;}
      buffer_.append(data,static_cast<size_t>(n));
      if(buffer_.size()>8192) {throw std::runtime_error("serial receive overflow");}
    }
    std::vector<std::string> result;
    size_t end;
    while ((end=buffer_.find('\n'))!=std::string::npos) {
      result.push_back(buffer_.substr(0,end)); buffer_.erase(0,end+1);
    }
    if(buffer_.size()>512) {throw std::runtime_error("unterminated serial frame");}
    return result;
  }
private:
  [[noreturn]] static void fail(const char * operation) {
    throw std::runtime_error(std::string(operation)+": "+std::strerror(errno));
  }
  int fd_{-1}; std::string buffer_;
};
}
