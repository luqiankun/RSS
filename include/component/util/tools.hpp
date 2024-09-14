#ifndef TIMER_HPP
#define TIMER_HPP
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
  const auto p = std::chrono::duration_cast<std::chrono::milliseconds>(
                     t.time_since_epoch())
                     .count();
  time << std::put_time(std::localtime(&tm), "%Y-%m-%dT%H:%M:%S.") << p % 1000
       << "Z";
  return time.str();
}
inline std::string get_date_fmt() {
  const auto t = std::chrono::system_clock::now();
  const auto tm = std::chrono::system_clock::to_time_t(t);
  std::stringstream time;
  // auto p = std::chrono::duration_cast<std::chrono::milliseconds>(
  //              t.time_since_epoch())
  //              .count();
  time << std::put_time(std::gmtime(&tm), "%Y-%m-%d");
  return time.str();
}

inline std::optional<std::chrono::system_clock::time_point>
get_time_from_str(const std::string &msg) {
  const std::regex match_reg{
      R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}.\d{1,10}Z)"};
  const std::regex search_reg{
      R"(((\d{4}-\d{2}-\d{2})T(\d{2}:\d{2}:\d{2}).(\d{1,10})Z))"};
  if (!std::regex_match(msg, match_reg)) {
    return std::nullopt;
  }
  try {
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
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count() %
            1000;
  std::time_t currentTime = std::time(nullptr);
  std::tm *localTm = std::gmtime(&currentTime);
  std::time_t utcTime = std::mktime(localTm);
  auto p =
      std::chrono::time_point<std::chrono::system_clock>::clock::from_time_t(
          utcTime) +
      std::chrono::milliseconds(ms);
  return p;
}
#endif