#include "../../../include/component/event/publisher.hpp"
#include "../../../include/component/event/subscriber.hpp"

int main() {
  class TestEvent : public EventBase {
   public:
    using EventBase::EventBase;
    std::map<std::string, std::string> get_event_msg() override {
      auto res = std::map<std::string, std::string>();
      res["brief"] = "test";
      return res;
    }
  };
  class SUB : public Subscriber {
    using Subscriber::Subscriber;
    void on_event(const std::shared_ptr<EventBase>& e) override {
      auto msg = e->get_event_msg();
      std::cout << "--------rece event--------\n";
      std::cout << "event_name: " << e->name << "\n"
                << "event_time: " << e->event_time << "\n"
                << "event_type: " << e->event_type << "\n";
      for (auto& x : msg) {
        std::cout << "msg[" << x.first << "]: " << x.second << "\n";
      }
      std::cout << "-------------------------\n\n";
    }
  };
  auto pub = std::make_shared<Publisher>("pub1");
  auto event = std::make_shared<TestEvent>("event1");
  event->event_time = "20231011-12-23-45.345";
  auto sub = std::make_shared<SUB>("sub1");
  sub->subscriber(pub);
  pub->publish(event);
  std::cin.get();
}
