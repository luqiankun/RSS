#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP
#include <utility>

#include "publisher.hpp"
namespace event {
class Subscriber : public TCSObject,
                   public ::std::enable_shared_from_this<Subscriber> {
 public:
  using TCSObject::TCSObject;
  Subscriber(const std::string& n,
             std::function<void(const std::shared_ptr<Event>&)> cb)
      : TCSObject(n) {
    event_handler = std::move(cb);
  }
  void subscriber(const std::shared_ptr<Publisher>&);
  void unsubscriber(const std::shared_ptr<Publisher>&);
  void on_event(const std::shared_ptr<Event>&);

 private:
  std::function<void(const std::shared_ptr<Event>&)> event_handler;
};
}  // namespace event
#endif