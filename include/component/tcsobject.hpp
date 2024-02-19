#ifndef TCSOBJECT_HPP
#define TCSOBJECT_HPP
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

class TCSObject {
 public:
  TCSObject() = delete;
  virtual ~TCSObject() = default;
  explicit TCSObject(const std::string& n) : name(n) {
    std::hash<std::string> hash_fn;
    name_hash = hash_fn(n);
  }

 public:
  std::string name;
  size_t name_hash;
  std::map<std::string, std::string> properties_readonly;
  std::map<std::string, std::string> properties;
};
using TCSObjectPtr = std::shared_ptr<TCSObject>;
#endif