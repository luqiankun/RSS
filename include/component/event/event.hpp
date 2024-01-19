#ifndef EVENTBASE_HPP
#define EVENTBASE_HPP
#include "../tcsobject.hpp"
namespace event {
class Event : public TCSObject {
 public:
  using TCSObject::TCSObject;
  virtual ~Event() = default;
  std::chrono::steady_clock::time_point event_time;
  std::string event_type;
  std::weak_ptr<TCSObject> event_source;
  std::map<std::string, std::string> msg;
};
}  // namespace event
#endif