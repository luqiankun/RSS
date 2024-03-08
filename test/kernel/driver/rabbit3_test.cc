#include "../../../include/component/tools/mqtt/mqtt.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/component/vda5050/vda5050order.hpp"
#include "../../../include/component/vda5050/vda5050state.hpp"

class SimRabbit3 {
 public:
  enum class Status { IDLE, ERROR, RUNNING };
  SimRabbit3(const std::string& interface_name,
             const std::string& serial_number, const std::string& version,
             const std::string& manufacturer)
      : interface_name(interface_name),
        serial_number(serial_number),
        version(version),
        manufacturer(manufacturer) {
    vda_state.version = version;
    vda_state.manufacturer = manufacturer;
    vda_state.serial_number = serial_number;
    vda_state.order_id = "";
    vda_state.header_id = 0;
    vda_state.timestamp = get_time_fmt(std::chrono::system_clock::now());
    vda_state.order_update_id = 0;
    vda_state.last_node_id = "P3";
    vda_state.agv_position = vda5050::state::AgvPosition();
    vda_state.agv_position.value().x = 64387;
    vda_state.agv_position.value().y = -107450;
    vda_state.agv_position.value().theta = 0;
    vda_state.agv_position.value().map_id = "lyg";
    vda_state.last_node_seq_id = 0;
    vda_state.driving = false;
    vda_state.battery_state.battery_charge = 90;
    vda_state.battery_state.charging = false;
    vda_state.operating_mode = vda5050::state::OperMode::AUTOMATIC;
    vda_state.safetystate.estop = vda5050::state::SafetyStatus::AUTOACK;
    vda_state.safetystate.field_violation = false;
  }
  void process_msgs();
  void set_mqtt_ops(const std::string& name, const std::string& server_ip,
                    int server_port = 1883, bool ssl = false) {
    if (ssl) {
      auto addr = "mqtts://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    } else {
      auto addr = "mqtt://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<MqttClient>(addr, name);
    }
    con_ops.set_automatic_reconnect(true);
    con_ops.set_keep_alive_interval(5);
    con_ops.set_connect_timeout(2);
    auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                  "/" + serial_number + "/";
    mqtt::will_options will_ops;
    will_ops.set_qos(0);
    will_ops.set_retained(true);
    will_ops.set_topic(prefix + "connection");
    nlohmann::json will;
    will["connectionState"] = "CONNECTIONBROKEN";
    will["headerId"] = 0;
    will["manufacturer"] = manufacturer;
    will["serialNumber"] = serial_number;
    will["version"] = version;
    will["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
    will_ops.set_payload(will.dump());
    con_ops.set_will(will_ops);
  }
  void start() {
    if (mqtt_client) {
      th_pro = std::thread([&] {
        while (true) {
          std::unique_lock<std::mutex> lock(pro_mut);
          con.wait(lock, [&] {
            return !order_msgs.empty() || !instantaction_msgs.empty();
          });
          while (!instantaction_msgs.empty() || !order_msgs.empty()) {
            if (!instantaction_msgs.empty()) {
              auto act = instantaction_msgs.front();
              instantaction_msgs.pop();
              try {
                auto src = nlohmann::json::parse(act->get_payload());
                auto err = vda5050::order_validate(src);
                if (err.empty()) {
                  // do
                }
              } catch (nlohmann::json::exception& ec) {
              }
            } else if (!order_msgs.empty()) {
              auto step = order_msgs.front();
              order_msgs.pop();
              // LOG(INFO) << step->get_topic();
              try {
                auto src = nlohmann::json::parse(step->get_payload());
                auto err = vda5050::order_validate(src);
                if (err.empty()) {
                  // do
                  auto ord = vda5050::order::VDA5050Order(src);
                  if (vda_state.order_id != ord.order_id) {
                    // new order
                    auto state = vda5050::state::VDA5050State(vda_state);
                    state.order_id = ord.order_id;
                    state.errors.clear();
                    state.nodestates.clear();
                    state.edgestates.clear();
                    state.actionstates.clear();
                    //
                    if ((ord.nodes.size() - ord.edges.size()) != 1) {
                      auto err = vda5050::state::Error();
                      err.error_level = vda5050::state::ErrorLevel::FATAL;
                      err.error_type = "node size not match edge size";
                      state.errors.push_back(err);
                      vda_state = state;
                    } else if (ord.edges.size() > 0) {
                      // state update
                      for (int i = 0; i < ord.nodes.size(); i++) {
                        auto n1 = vda5050::state::NodeState();
                        n1.node_id = ord.nodes.at(i).node_id;
                        n1.released = false;
                        n1.sequence_id = ord.nodes.at(i).sequence_id;
                        n1.node_position = vda5050::state::NodePosition();
                        n1.node_position.value().x =
                            ord.nodes.at(i).node_position.value().x;
                        n1.node_position.value().y =
                            ord.nodes.at(i).node_position.value().y;
                        state.nodestates.push_back(n1);
                      }
                      vda_state = state;
                      // move
                      for (int i = 0; i < ord.nodes.size() - 1; i++) {
                        status = Status::RUNNING;
                        auto start_node = ord.nodes.at(i);
                        auto end_node = ord.nodes.at(i + 1);
                        LOG(INFO) << "move from " << start_node.node_id
                                  << " to " << end_node.node_id;
                        float s_x = start_node.node_position.value().x;
                        float s_y = start_node.node_position.value().y;
                        float e_x = end_node.node_position.value().x;
                        float e_y = end_node.node_position.value().y;
                        // move
                        auto vx = 5000 * cos(std::atan2(e_y - s_y, e_x - s_x));
                        auto vy = 5000 * sin(std::atan2(e_y - s_y, e_x - s_x));
                        LOG(INFO) << "Vx " << vx << " Vy " << vy;
                        auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                         (e_y - s_y) * (e_y - s_y)) *
                                    1000 / 5000;  // ms
                        state.driving = true;
                        state.nodestates.at(i).released = true;

                        for (auto i = 0; i < time; i = i + 50) {
                          state.agv_position.value().x = s_x + vx / 1000 * i;
                          state.agv_position.value().y = s_y + vy / 1000 * i;
                          std::this_thread::sleep_for(
                              std::chrono::milliseconds(50));
                          vda_state = state;
                        }
                        state.agv_position.value().x = e_x;
                        state.agv_position.value().y = e_y;
                        state.nodestates.at(i).released = true;
                        state.nodestates.at(i + 1).released = true;
                        state.driving = false;
                        state.last_node_id = end_node.node_id;
                        status = Status::IDLE;
                        vda_state = state;
                      }
                    } else {
                      // only action
                      // state update
                      for (int i = 0; i < ord.nodes.size(); i++) {
                        auto n1 = vda5050::state::NodeState();
                        n1.node_id = ord.nodes.at(i).node_id;
                        n1.released = false;
                        n1.sequence_id = ord.nodes.at(i).sequence_id;
                        n1.node_position = vda5050::state::NodePosition();
                        n1.node_position.value().x =
                            ord.nodes.at(i).node_position.value().x;
                        n1.node_position.value().y =
                            ord.nodes.at(i).node_position.value().y;
                        state.nodestates.push_back(n1);
                        for (auto& x : ord.nodes.at(i).actions) {
                          auto act = vda5050::state::ActionState();
                          act.action_id = x.action_id;
                          act.action_status =
                              vda5050::state::ActionStatus::WAITING;
                          state.actionstates.push_back(act);
                        }
                      }
                      vda_state = state;
                      // action
                      status = Status::RUNNING;
                      for (int i = 0; i < ord.nodes.at(0).actions.size(); i++) {
                        float s_x = ord.nodes.at(0).node_position.value().x;
                        float s_y = ord.nodes.at(0).node_position.value().y;
                        float e_x =
                            std::get<float>(ord.nodes.at(0)
                                                .actions.at(i)
                                                .action_parameters.value()
                                                .at(0)
                                                .value);
                        float e_y =
                            std::get<float>(ord.nodes.at(0)
                                                .actions.at(i)
                                                .action_parameters.value()
                                                .at(1)
                                                .value);
                        auto vx = 5000 * cos(std::atan2(e_y - s_y, e_x - s_x));
                        auto vy = 5000 * sin(std::atan2(e_y - s_y, e_x - s_x));
                        LOG(INFO) << "Vx " << vx << " Vy " << vy;
                        auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                         (e_y - s_y) * (e_y - s_y)) *
                                    1000 / 5000;  // ms
                        state.driving = true;
                        state.nodestates.at(0).released = true;

                        for (auto i = 0; i < time; i = i + 50) {
                          if (!state.agv_position.has_value()) {
                            state.agv_position = vda5050::state::AgvPosition();
                          }
                          state.agv_position.value().x = s_x + vx / 1000 * i;
                          state.agv_position.value().y = s_y + vy / 1000 * i;
                          std::this_thread::sleep_for(
                              std::chrono::milliseconds(50));
                          vda_state = state;
                        }
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(500));
                        state.agv_position.value().x = s_x;
                        state.agv_position.value().y = s_y;
                        state.driving = false;
                        state.last_node_id = ord.nodes.at(0).node_id;
                        state.actionstates.at(i).action_status =
                            vda5050::state::ActionStatus::FINISHED;
                        status = Status::IDLE;
                        vda_state = state;
                        LOG(INFO) << "action end";
                      }
                    }
                  } else if (vda_state.order_id == src["orderId"]) {
                    // update order
                  }
                } else {
                  for (auto& x : err) {
                    LOG(ERROR) << x;
                  }
                }
              } catch (nlohmann::json::exception& ec) {
              }
            }
          }
        }
      });
      th_state = std::thread([&] {
        int send_id{0};
        while (true) {
          if (mqtt_client->is_connected()) {
            auto prefix = "/" + interface_name + "/" + version + "/" +
                          manufacturer + "/" + serial_number + "/";
            mqtt::message_ptr msg = std::make_shared<mqtt::message>();
            msg->set_qos(0);
            msg->set_topic(prefix + "state");
            vda_state.header_id = send_id++;
            vda_state.timestamp =
                get_time_fmt(std::chrono::system_clock::now());
            msg->set_payload(vda_state.to_json().dump());
            mqtt_client->publish(msg);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      });
      mqtt_client->on();
      mqtt_client->set_connected_handler([&](std::string) {
        LOG(INFO) << serial_number << "  ONLINE";

        auto prefix = "/" + interface_name + "/" + version + "/" +
                      manufacturer + "/" + serial_number + "/";
        nlohmann::json ctx;
        ctx["connectionState"] = "ONLINE";
        ctx["headerId"] = 0;
        ctx["manufacturer"] = manufacturer;
        ctx["serialNumber"] = serial_number;
        ctx["version"] = version;
        ctx["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
        mqtt::message_ptr msg = std::make_shared<mqtt::message>();
        msg->set_qos(0);
        msg->set_retained(true);
        msg->set_payload(ctx.dump());
        msg->set_topic(prefix + "connection");
        mqtt_client->publish(msg);
        //
        mqtt_client->subscribe("/" + interface_name + "/" + version + "/" +
                                   manufacturer + "/" + serial_number + "/#",
                               0);
      });
      mqtt_client->set_connection_lost_handler([&](std::string) {
        LOG(WARNING) << serial_number << "  CONNECTIONBROKEN";
      });
      mqtt_client->set_disconnected_handler(
          [&](const mqtt::properties&, mqtt::ReasonCode) {
            LOG(INFO) << serial_number << " master OFFLINE";
          });
      mqtt_client->connect(con_ops);
      auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                    "/" + serial_number + "/";
      mqtt_client->set_func(prefix + "order", [&](mqtt::const_message_ptr t) {
        // LOG(INFO) << t->get_topic();
        order_msgs.push(t);
        con.notify_one();
      });
      // mqtt_client->set_func(".*", [&](mqtt::const_message_ptr t) {
      //   LOG(INFO) << t->get_topic();
      // });
      mqtt_client->set_func(prefix + "instantAction",
                            [&](mqtt::const_message_ptr t) {
                              instantaction_msgs.push(t);
                              con.notify_one();
                            });
    }
  }

 public:
  mqtt::connect_options con_ops;
  std::shared_ptr<MqttClient> mqtt_client;
  std::queue<mqtt::const_message_ptr> order_msgs;
  std::queue<mqtt::const_message_ptr> instantaction_msgs;
  vda5050::state::VDA5050State vda_state;
  std::thread th_state;
  std::thread th_pro;
  std::mutex pro_mut;
  std::condition_variable con;
  Status status{Status::IDLE};
  //
  std::string broker_ip;
  int broker_port;
  std::string interface_name;
  std::string serial_number;
  std::string version;
  std::string manufacturer;
  //
};
INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv) {
  SimRabbit3 agv("uagv", "tx1", "2.0", "rw");
  agv.set_mqtt_ops(agv.serial_number, "192.168.0.39");
  agv.start();
  std::cin.get();
}
