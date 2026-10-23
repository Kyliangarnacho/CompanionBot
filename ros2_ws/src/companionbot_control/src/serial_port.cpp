#include "companionbot_control/serial_port.hpp"
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <thread>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

namespace companionbot {
uint64_t monotonic_ns() {
  timespec t{};
  if(clock_gettime(CLOCK_MONOTONIC,&t)) throw std::system_error(errno,std::generic_category(),"clock_gettime");
  return uint64_t(t.tv_sec)*1000000000ULL+uint64_t(t.tv_nsec);
}
SerialPort::SerialPort(const std::string &path,unsigned baud):baud_(baud) {
  speed_t speed;
  switch(baud) {
    case 115200: speed=B115200; break;
    case 460800: speed=B460800; break;
    case 921600: speed=B921600; break;
    default: throw std::invalid_argument("supported baud: 115200, 460800, 921600");
  }
  fd_=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
  if(fd_<0) throw std::system_error(errno,std::generic_category(),"open serial "+path);
  if(ioctl(fd_,TIOCEXCL)) { int e=errno; close(fd_); fd_=-1; throw std::system_error(e,std::generic_category(),"serial exclusive ownership"); }
  termios t{};
  if(tcgetattr(fd_,&t)) { int e=errno; close(fd_); fd_=-1; throw std::system_error(e,std::generic_category(),"tcgetattr"); }
  cfmakeraw(&t); t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~(CRTSCTS|CSTOPB|PARENB);
  t.c_cflag=(t.c_cflag&~CSIZE)|CS8; t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
  cfsetispeed(&t,speed); cfsetospeed(&t,speed);
  if(tcsetattr(fd_,TCSANOW,&t) || tcflush(fd_,TCIOFLUSH)) {
    int e=errno; close(fd_); fd_=-1; throw std::system_error(e,std::generic_category(),"configure serial");
  }
}
SerialPort::~SerialPort() { if(fd_>=0) { (void)tcflush(fd_,TCOFLUSH); close(fd_); } }
std::size_t SerialPort::read_some(uint8_t *data,std::size_t cap,int timeout_ms) {
  pollfd p{fd_,POLLIN,0}; int result=poll(&p,1,timeout_ms);
  if(result<0) { if(errno==EINTR) return 0; throw std::system_error(errno,std::generic_category(),"serial poll"); }
  if(p.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("serial disconnected");
  if(!result) return 0;
  ssize_t n=read(fd_,data,cap);
  if(n<0) { if(errno==EINTR || errno==EAGAIN) return 0; throw std::system_error(errno,std::generic_category(),"serial read"); }
  return static_cast<std::size_t>(n);
}
std::chrono::nanoseconds SerialPort::transmission_time(std::size_t size) const {
  // 8N1: one start + eight data + one stop bit. Round up, never down.
  return std::chrono::nanoseconds((uint64_t(size)*10000000000ULL+baud_-1)/baud_);
}
void SerialPort::write_all(const uint8_t *data,std::size_t size,std::chrono::steady_clock::time_point deadline) {
  std::size_t offset=0;
  const auto wire_time=transmission_time(size);
  if(std::chrono::steady_clock::now()+wire_time>=deadline)
    throw std::runtime_error("serial TX cannot fit line budget before deadline");
  while(offset<size) {
    if(std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("serial TX deadline exceeded; frame discarded");
    ssize_t n=write(fd_,data+offset,size-offset);
    if(n>0) { offset+=static_cast<std::size_t>(n); continue; }
    if(n<0 && errno!=EINTR && errno!=EAGAIN) throw std::system_error(errno,std::generic_category(),"serial write");
    pollfd p{fd_,POLLOUT,0};
    int result=poll(&p,1,1);
    if(result<0 && errno!=EINTR) throw std::system_error(errno,std::generic_category(),"serial TX poll");
    if(p.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("serial TX disconnected");
  }
  // A successful write only means kernel acceptance. Bound driver backlog and
  // spend at least one full frame's line budget before accepting another frame.
  // TIOCOUTQ may omit a USB adapter's private FIFO; MCU deadlines remain final.
  const auto earliest_done=std::chrono::steady_clock::now()+wire_time;
  for(;;) {
    int queued=0;
    if(ioctl(fd_,TIOCOUTQ,&queued)) throw std::system_error(errno,std::generic_category(),"serial output queue");
    const auto now=std::chrono::steady_clock::now();
    if(now>=deadline) throw std::runtime_error("serial TX drain deadline exceeded; session must restart");
    if(!queued && now>=earliest_done) break;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}
}  // namespace companionbot
