#ifndef VDA5050_MASTER_HPP
#define VDA5050_MASTER_HPP
#include "../data/order/orderquence.hpp"
#include "./vda5050insact.hpp"
#include "./vda5050order.hpp"
#include "./vda5050state.hpp"

namespace vda5050 {

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
                    int server_port = 1883, bool ssl = false);
  void set_mqtt_ops(const std::string& name, const std::string& server_ip,
                    int server_port = 1883, bool ssl = false);
  void start();
  void stop();
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
