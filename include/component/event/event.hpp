#ifndef EVENTBASE_HPP
#define EVENTBASE_HPP
#include "../tcsobject.hpp"
class EventBase : public TCSObject,
                  public ::std::enable_shared_from_this<EventBase> {
 public:
  using TCSObject::TCSObject;
  virtual std::map<std::string, std::string> get_event_msg() = 0;

 public:
  std::string event_time;
  std::string event_type;
  std::weak_ptr<TCSObject> event_source;
};
#endif