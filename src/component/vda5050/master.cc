#include "../../../include/component/vda5050/master.hpp"

#include "../../../include/component/tools/log/easylogging++.h"
namespace vda5050 {
void vda5050::VehicleMaster::set_mqtt_ops(const std::string& name,
                                          const std::string& user_name,
                                          const std::string& password,
                                          const std::string& server_ip,
                                          int server_port, bool ssl) {
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

void vda5050::VehicleMaster::set_mqtt_ops(const std::string& name,
                                          const std::string& server_ip,
                                          int server_port, bool ssl) {
  if (ssl) {
    auto addr = "ssl://" + server_ip + ":" + std::to_string(server_port);
    mqtt_client = std::make_shared<MqttClient>(addr, name);
  } else {
    auto addr = "tcp://" + server_ip + ":" + std::to_string(server_port);
    MqttClient cli(addr, name);
    mqtt_client = std::make_shared<MqttClient>(addr, name);
  }

  con_ops.set_automatic_reconnect(true);
  // con_ops.set_keep_alive_interval(5);
  con_ops.set_connect_timeout(2);
}

void VehicleMaster::start() {
  if (mqtt_client) {
    mqtt_client->on();
    mqtt_client->set_connected_handler([&](std::string) {
      master_state = MasterMqttStatus::ONLINE;
      CLOG(INFO, mqtt_log) << "mqtt_serial_number:" << serial_number
                           << " master ONLINE";
      mqtt_client->subscribe("/" + interface_name + "/" + version + "/" +
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

void VehicleMaster::stop() {
  if (master_state != MasterMqttStatus::OFFLINE) {
    master_state = MasterMqttStatus::OFFLINE;
  }
}

}  // namespace vda5050
