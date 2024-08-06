#ifndef RSSOBJECT_HPP
#define RSSOBJECT_HPP
#include <condition_variable>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stack>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "./util/timer.hpp"
constexpr auto rss_log{"rss"};
constexpr auto dispatch_log{"dispatch"};
constexpr auto allocate_log{"allocate"};
constexpr auto order_log{"order"};
constexpr auto planner_log{"planner"};
constexpr auto driver_log{"driver"};
constexpr auto mqtt_log{"mqtt"};

class RSSObject {
 public:
  RSSObject() = delete;
  virtual ~RSSObject() = default;
  explicit RSSObject(const std::string& n) : name(n) {
    std::hash<std::string> hash_fn;
    name_hash = hash_fn(n);
  }

 public:
  std::string name;
  size_t name_hash;
  std::map<std::string, std::string> properties_readonly;
  std::map<std::string, std::string> properties;
};
using RSSObjectPtr = std::shared_ptr<RSSObject>;
#endif