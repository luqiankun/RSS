#include "../../../include/component/event/publisher.hpp"
#include "../../../include/component/event/subscriber.hpp"

int main() {
  auto pub = std::make_shared<event::Publisher>("pub1");
  auto event = std::make_shared<event::Event>("event1");
  event->msg["brief"] = "test";
  event->event_time = std::chrono::steady_clock::now();
  auto sub = std::make_shared<event::Subscriber>(
      "sub1", [](const std::shared_ptr<event::Event>& e) {
        std::cout << "--------rece event--------\n";
        std::cout << "event_name: " << e->name << "\n"
                  << "event_time: " << e->event_time.time_since_epoch().count()
                  << "\n"
                  << "event_type: " << e->event_type << "\n";
        for (auto& x : e->msg) {
          std::cout << "msg[" << x.first << "]: " << x.second << "\n";
        }
        std::cout << "-------------------------\n\n";
      });
  sub->subscriber(pub);
  pub->publish(event);
  std::cin.get();
}
