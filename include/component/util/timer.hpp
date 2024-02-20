#ifndef TIMER_HPP
#define TIMER_HPP
#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>

#include "../tools/log/easylogging++.h"
class cpu_timer {
 public:
  explicit cpu_timer(const std::string& n = "") : name(n) {
    start = std::chrono::steady_clock::now();
  }
  ~cpu_timer() {
    auto end = std::chrono::steady_clock::now();
    auto dur =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    uint32_t s = dur.count() / 1000;
    uint32_t ms = dur.count() % 1000;
    CLOG(INFO, "timer") << name << " use time: " << s << "(s) " << ms
                        << "(ms)\n\n";
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

inline std::optional<std::chrono::system_clock::time_point> get_time_from_str(
    const std::string& msg) {
  std::regex match_reg{R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}.\d{1,10}Z)"};
  std::regex search_reg{
      R"(((\d{4}-\d{2}-\d{2})T(\d{2}:\d{2}:\d{2}).(\d{1,10})Z))"};
  if (!std::regex_match(msg, match_reg)) {
    return std::nullopt;
  }
  std::smatch cmatch;
  std::regex_search(msg, cmatch, search_reg);
  std::chrono::milliseconds ms(std::stoi(*(cmatch.end() - 1)));
  std::tm t = {};
  std::istringstream ss{msg};
  ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
  long timeSinceEpoch = std::mktime(&t);
  auto duration = std::chrono::seconds(timeSinceEpoch);
  auto time_point = std::chrono::system_clock::time_point(duration) + ms;
  return time_point;
}

#endif