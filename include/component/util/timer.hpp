#ifndef TIMER_HPP
#define TIMER_HPP
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../tools/log/easylogging++.h"
class cpu_timer {
 public:
  explicit cpu_timer(const std::string &n = "") : name(n) {
    start = std::chrono::steady_clock::now();
  }
  ~cpu_timer() {
    auto end = std::chrono::steady_clock::now();
    auto dur =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    uint32_t s = dur.count() / 1000;
    uint32_t ms = dur.count() % 1000;
    LOG(TRACE) << name << " use time: " << s << "(s) " << ms << "(ms)\n\n";
  }

 private:
  std::string name{""};
  std::chrono::time_point<std::chrono::steady_clock> start;
};

inline std::string get_time_fmt(std::chrono::system_clock::time_point t) {
  auto tm = std::chrono::system_clock::to_time_t(t);
  std::stringstream time;
  auto p = std::chrono::duration_cast<std::chrono::milliseconds>(
               t.time_since_epoch())
               .count();
  time << std::put_time(std::localtime(&tm), "%Y-%m-%dT%H:%M:%S.") << p % 1000
       << "Z";
  return time.str();
}
inline std::string get_date_fmt() {
  auto t = std::chrono::system_clock::now();
  auto tm = std::chrono::system_clock::to_time_t(t);
  std::stringstream time;
  auto p = std::chrono::duration_cast<std::chrono::milliseconds>(
               t.time_since_epoch())
               .count();
  time << std::put_time(std::localtime(&tm), "%Y-%m-%d");
  return time.str();
}

#endif