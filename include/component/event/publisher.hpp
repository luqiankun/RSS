#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include "event.hpp"
namespace event {
class Subscriber;
class Publisher : public TCSObject,
                  public std::enable_shared_from_this<Publisher> {
 public:
  using TCSObject::TCSObject;
  void add_suber(const std::shared_ptr<Subscriber>&);
  void remove_suber(const std::shared_ptr<Subscriber>&);
  void publish(const std::shared_ptr<Event>& e);

 protected:
  std::map<std::string, std::weak_ptr<Subscriber>> subers;
};
}  // namespace event
#endif