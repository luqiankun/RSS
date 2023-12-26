#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP
#include "publisher.hpp"
class Subscriber : public TCSObject,
                   public ::std::enable_shared_from_this<Subscriber> {
 public:
  using TCSObject::TCSObject;
  void subscriber(const std::shared_ptr<Publisher>&);
  void unsubscriber(const std::shared_ptr<Publisher>&);
  virtual void on_event(const std::shared_ptr<EventBase>&);
};
#endif