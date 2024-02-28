#ifndef MYMQTT_HPP
#define MYMQTT_HPP
#include <mqtt/client.h>

#include <regex>

class MqttClient : public mqtt::async_client {
 public:
  using mqtt::async_client::async_client;
  void set_func(std::string path, message_handler func) { cbs[path] = func; }
  void cancle_func(std::string path) {
    if (cbs.find(path) != cbs.end()) {
      cbs.erase(path);
    }
  }
  void on() {
    this->set_message_callback([&](mqtt::const_message_ptr msg) {
      auto topic = msg->get_topic();
      for (auto& x : cbs) {
        std::string reg = x.first;
        std::regex pattern{reg};
        if (std::regex_match(topic, pattern)) {
          if (x.second) {
            x.second(msg);
          }
        }
      }
    });
  }
  virtual ~MqttClient() {
    if (is_connected()) {
      auto ptr = disconnect();
      ptr->wait();
    }
  }

 protected:
  std::unordered_map<std::string, message_handler> cbs;
};

#endif