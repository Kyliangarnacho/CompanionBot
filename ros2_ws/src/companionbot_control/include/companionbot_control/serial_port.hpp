#ifndef COMPANIONBOT_CONTROL_SERIAL_PORT_HPP
#define COMPANIONBOT_CONTROL_SERIAL_PORT_HPP
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace companionbot {
class SerialPort {
 public:
  explicit SerialPort(const std::string &path, unsigned baud=921600);
  ~SerialPort();
  SerialPort(const SerialPort &)=delete;
  SerialPort &operator=(const SerialPort &)=delete;
  std::size_t read_some(uint8_t *data, std::size_t capacity, int timeout_ms);
  std::chrono::nanoseconds transmission_time(std::size_t size) const;
  void write_all(const uint8_t *data, std::size_t size,
                 std::chrono::steady_clock::time_point deadline);
 private:
  int fd_=-1;
  unsigned baud_;
};
uint64_t monotonic_ns();
}  // namespace companionbot
#endif
