#include "../../../include/component/event/publisher.hpp"

#include "../../../include/component/event/subscriber.hpp"
namespace event {

void Publisher::add_suber(const std::shared_ptr<Subscriber>& s) {
  subers[s->name] = s;
}

void Publisher::remove_suber(const std::shared_ptr<Subscriber>& s) {
  auto p = subers.find(s->name);
  if (p != subers.end()) {
    subers.erase(p);
  }
}
void Publisher::publish(const std::shared_ptr<Event>& e) {
  for (auto it = subers.begin(); it != subers.end();) {
    auto p = it->second.lock();
    if (p) {
      p->on_event(e);
      it++;
    } else {
      it = subers.erase(it);
    }
  }
}
}  // namespace event