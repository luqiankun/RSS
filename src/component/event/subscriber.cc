#include "../../../include/component/event/subscriber.hpp"
namespace event {

void Subscriber::subscriber(const std::shared_ptr<Publisher>& pub) {
  pub->add_suber(shared_from_this());
}

void Subscriber::unsubscriber(const std::shared_ptr<Publisher>& pub) {
  pub->remove_suber(shared_from_this());
}

void Subscriber::on_event(const std::shared_ptr<Event>& e) {
  if (event_handler) {
    event_handler(e);
  }
}
}  // namespace event