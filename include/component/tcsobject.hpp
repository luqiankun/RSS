#ifndef TCSOBJECT_HPP
#define TCSOBJECT_HPP
#include <condition_variable>
#include <deque>
#include <eigen3/Eigen/Eigen>
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
  explicit TCSObject(const std::string& n) : name(n) {}

 public:
  std::string name;
  std::map<std::string, std::string> properties_readonly;
  std::map<std::string, std::string> properties;
};
using TCSObjectPtr = std::shared_ptr<TCSObject>;
#endif