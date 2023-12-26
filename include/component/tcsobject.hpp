#ifndef TCSOBJECT_HPP
#define TCSOBJECT_HPP
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>
class TCSObject {
 public:
  TCSObject() = delete;
  explicit TCSObject(const std::string& n) : name(n) {}

 public:
  std::string name;
  std::map<std::string, std::string> properties_readonly;
  std::map<std::string, std::string> properties;
};
using TCSObjectPtr = std::shared_ptr<TCSObject>;
#endif