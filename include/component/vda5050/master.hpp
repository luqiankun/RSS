#ifndef VDA5050_MASTER_HPP
#define VDA5050_MASTER_HPP
#include <mqtt/client.h>

#include <regex>

#include "../data/order/orderquence.hpp"
#include "../util/tools.hpp"
#include "./vda5050insact.hpp"
#include "./vda5050order.hpp"
#include "./vda5050state.hpp"
namespace vda5050 {
class MqttClient : public mqtt::async_client {
 public:
  using mqtt::async_client::async_client;
  void set_func(std::string path, message_handler func) { cbs[path] = func; }
  void cancle_func(const std::string& path) {
    auto it = cbs.find(path);
    if (it != cbs.end()) {
      cbs.erase(it);
    }
  }
  void on() {
    this->set_message_callback([&](mqtt::const_message_ptr msg) {
      // std::cerr << msg->get_topic() << std::endl;
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

 protected:
  std::unordered_map<std::string, message_handler> cbs;
};
class VehicleMaster {
 public:
  VehicleMaster() = delete;
  VehicleMaster(const std::string& interface_name,
                const std::string& serial_number, const std::string& version,
                const std::string& manufacturer)
      : interface_name(interface_name),
        serial_number(serial_number),
        version(version),
        manufacturer(manufacturer) {}
  void set_mqtt_ops(const std::string& name, const std::string& user_name,
                    const std::string& password, const std::string& server_ip,
                    int server_port = 1883, bool ssl = false) {
    con_ops.set_password(password);
    con_ops.set_user_name(user_name);
    con_ops.set_connect_timeout(2);
    if (ssl) {
      auto addr = "ssl://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    } else {
      auto addr = "tcp://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    }
    con_ops.set_automatic_reconnect(true);
    // con_ops.set_keep_alive_interval(5);
    con_ops.set_connect_timeout(2);
  }
  void set_mqtt_ops(const std::string& name, const std::string& server_ip,
                    int server_port = 1883, bool ssl = false) {
    if (ssl) {
      auto addr = "ssl://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    } else {
      auto addr = "tcp://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    }

    con_ops.set_automatic_reconnect(true);
    // con_ops.set_keep_alive_interval(5);
    con_ops.set_connect_timeout(2);
  }
  void start() {
    if (mqtt_client) {
      mqtt_client->on();
      mqtt_client->set_connected_handler([&](std::string) {
        master_state = MasterMqttStatus::ONLINE;
        CLOG(INFO, mqtt_log)
            << "mqtt_serial_number:" << serial_number << " master ONLINE";
        int ver = static_cast<int>(std::stod(version));
        auto vda_version = "v" + std::to_string(ver);
        mqtt_client->subscribe(interface_name + "/" + vda_version + "/" +
                                   manufacturer + "/" + serial_number + "/#",
                               0);
      });
      mqtt_client->set_connection_lost_handler([&](std::string) {
        master_state = MasterMqttStatus::CONNECTIONBROKEN;
        CLOG(WARNING, mqtt_log) << "mqtt_serial_number:" << serial_number
                                << " master CONNECTIONBROKEN";
      });
      mqtt_client->set_disconnected_handler(
          [&](const mqtt::properties&, mqtt::ReasonCode) {
            master_state = MasterMqttStatus::OFFLINE;
            CLOG(INFO, mqtt_log)
                << "mqtt_serial_number:" << serial_number << " master OFFLINE";
          });
      mqtt_client->connect(con_ops);
    }
  }
  void stop() { master_state = MasterMqttStatus::OFFLINE; }
  virtual ~VehicleMaster() { stop(); }

 public:
  std::string interface_name;
  std::string serial_number;
  std::string version;
  std::string manufacturer;

 public:
  std::shared_ptr<MqttClient> mqtt_client;
  mqtt::connect_options con_ops;
  vda5050::MasterMqttStatus master_state{vda5050::MasterMqttStatus::OFFLINE};
};
}  // namespace vda5050
#endif
