#include "../../../include/component/event/publisher.hpp"

#include "../../../include/component/event/subscriber.hpp"

void Publisher::add_suber(const std::shared_ptr<Subscriber>& s) {
  subers[s->name] = s;
}

void Publisher::remove_suber(const std::shared_ptr<Subscriber>& s) {
  auto p = subers.find(s->name);
  if (p != subers.end()) {
    subers.erase(p);
  }
}
void Publisher::publish(const std::shared_ptr<EventBase>& e) {
  for (auto& x : subers) {
    auto p = x.second.lock();
    if (p) {
      p->on_event(e);
    }
  }
}