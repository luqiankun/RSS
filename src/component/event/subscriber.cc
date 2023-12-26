#include "../../../include/component/event/subscriber.hpp"
void Subscriber::subscriber(const std::shared_ptr<Publisher>& pub) {
  pub->add_suber(shared_from_this());
}

void Subscriber::unsubscriber(const std::shared_ptr<Publisher>& pub) {
  pub->remove_suber(shared_from_this());
}

void Subscriber::on_event(const std::shared_ptr<EventBase>& e) {
  std::cout << "--------rece event--------\n";
  std::cout << "event_name: " << e->name << "\n"
            << "event_time: " << e->event_time << "\n"
            << "event_type: " << e->event_type << "\n";
  std::cout << "-------------------------\n\n";
}