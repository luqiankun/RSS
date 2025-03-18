#ifndef TIMER_HPP
#define TIMER_HPP
#include <time.h>

#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>
#include <utility>

#include "../../../include/3rdparty/uuid/uuid.hpp"
#include "../../3rdparty/log/easylogging++.h"
constexpr auto timer_log{"timer"};
static std::mutex timer_mutex;
class cpu_timer {
 public:
  explicit cpu_timer(std::string n = "")
      : name(std::move(n)), start(std::chrono::steady_clock::now()) {}
  ~cpu_timer() {
    auto end = std::chrono::steady_clock::now();
    auto dur =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    uint32_t s = dur.count() / 1000000;
    uint32_t ms = dur.count() / 1000;
    uint32_t us = dur.count() % 1000;
    CLOG(INFO, timer_log) << name << " use time: " << s << "(s) " << ms
                          << "(ms) " << us << "(us)\n";
  }

 private:
  std::string name;
  std::chrono::time_point<std::chrono::steady_clock> start;
};

inline std::string get_time_fmt(std::chrono::system_clock::time_point t) {
  const auto tm = std::chrono::system_clock::to_time_t(t);
  std::stringstream time;
  const auto p = std::chrono::duration_cast<std::chrono::microseconds>(
                     t.time_since_epoch())
                     .count();
  time << std::put_time(std::localtime(&tm), "%Y-%m-%dT%H:%M:%S.")
       << p % 1000000 << "Z";
  return time.str();
}
inline std::string get_time_fmt_utc(std::chrono::system_clock::time_point t) {
  const auto tm = std::chrono::system_clock::to_time_t(t);
  std::stringstream time;
  const auto p = std::chrono::duration_cast<std::chrono::microseconds>(
                     t.time_since_epoch())
                     .count();
  time << std::put_time(std::gmtime(&tm), "%Y-%m-%dT%H:%M:%S.") << p % 1000000
       << "Z";
  return time.str();
}

inline std::optional<std::chrono::system_clock::time_point> get_time_from_str(
    const std::string &msg) {
  // const std::regex match_reg{
  //     R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{1,10}Z)"};
  const std::regex search_reg{
      R"((\d{4}-\d{2}-\d{2})T(\d{2}:\d{2}:\d{2})\.(\d{1,10})Z)"};
  try {
    std::smatch cmatch;
    if (!std::regex_search(msg, cmatch, search_reg)) {
      return std::nullopt;
    }
    // LOG(INFO) << cmatch.size();
    assert(cmatch.size() == 4);
    int tail = std::stoi(*(cmatch.end() - 1));
    std::chrono::microseconds us;
    if (tail >= 1000) {
      us = std::chrono::microseconds(tail);
    } else {
      us = std::chrono::microseconds(tail * 1000);
    }
    std::tm t = {};  // localtime();
    std::istringstream ss{msg};
    ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
    auto t2 = std::mktime(&t);  // utctime
    int offset = std::localtime(&t2)->tm_hour - std::gmtime(&t2)->tm_hour;
    auto time_point = std::chrono::system_clock::from_time_t(t2) + us +
                      std::chrono::hours(offset);
    return time_point;
  } catch (std::exception &ec) {
    CLOG(ERROR, timer_log) << ec.what();
    return std::nullopt;
  }
}
inline static uuids::uuid get_uuid() {
  static std::random_device rd;
  static auto seed_data = std::array<int, std::mt19937::state_size>{};
  std::generate(std::begin(seed_data), std::end(seed_data), std::ref(rd));
  std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
  static std::mt19937 generator(seq);
  static uuids::uuid_random_generator gen{generator};
  uuids::uuid const id = gen();
  return id;
}
inline static std::chrono::system_clock::time_point get_now_utc_time() {
  auto now = std::chrono::system_clock::now();
  return now;
}
#endif