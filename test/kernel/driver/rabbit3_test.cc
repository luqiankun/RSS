#include "../../../include/component/tools/mqtt/mqtt.hpp"
#include "../../../include/component/util/taskpool.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/component/vda5050/vda5050insact.hpp"
#include "../../../include/component/vda5050/vda5050order.hpp"
#include "../../../include/component/vda5050/vda5050state.hpp"

class SimRabbit3 {
 public:
  enum class Status { RUNNING, PASUED };
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
    con_ops.set_keep_alive_interval(5000);
    con_ops.set_connect_timeout(2);
    auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                  "/" + serial_number + "/";
    mqtt::will_options will_ops;
    will_ops.set_qos(0);
    will_ops.set_retained(true);
    will_ops.set_topic(prefix + "connection");
    jsoncons::json will;
    will["connectionState"] = "CONNECTIONBROKEN";
    will["headerId"] = 0;
    will["manufacturer"] = manufacturer;
    will["serialNumber"] = serial_number;
    will["version"] = version;
    will["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
    will_ops.set_payload(will.as_string());
    con_ops.set_will(will_ops);
  }
  void start() {
    if (mqtt_client) {
      th_state = std::thread([&] {
        int send_id{0};
        auto prefix = "/" + interface_name + "/" + version + "/" +
                      manufacturer + "/" + serial_number + "/";
        while (true) {
          if (mqtt_client->is_connected()) {
            try {
              mqtt::message_ptr msg = mqtt::make_message(
                  prefix + "state", vda_state.to_json().as_string(), 0, false);
              vda_state.header_id = send_id++;
              vda_state.timestamp =
                  get_time_fmt(std::chrono::system_clock::now());
              // LOG(WARNING) << msg->get_payload_ref();
              mqtt_client->publish(msg)->wait();

            } catch (mqtt::exception& ec) {
              LOG(ERROR) << ec.get_error_str();
            }
            // LOG(INFO) << vda_state.order_id << " " << vda_state.header_id;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      });
      mqtt_client->on();
      mqtt_client->set_connected_handler([&](std::string) {
        LOG(INFO) << serial_number << "  ONLINE";

        auto prefix = "/" + interface_name + "/" + version + "/" +
                      manufacturer + "/" + serial_number + "/";
        jsoncons::json ctx;
        ctx["connectionState"] = "ONLINE";
        ctx["headerId"] = 0;
        ctx["manufacturer"] = manufacturer;
        ctx["serialNumber"] = serial_number;
        ctx["version"] = version;
        ctx["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
        mqtt::message_ptr msg = std::make_shared<mqtt::message>();
        msg->set_qos(0);
        msg->set_retained(true);
        msg->set_payload(ctx.as_string());
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
      const std::string prefix = "/" + interface_name + "/" + version + "/" +
                                 manufacturer + "/" + serial_number + "/";
      mqtt_client->set_func(prefix + "order", [&](mqtt::const_message_ptr t) {
        // LOG(INFO) << t->get_topic();
        current_msg = mqtt::message_ptr_builder()
                          .topic(t->get_topic())
                          .properties(t->get_properties())
                          .qos(t->get_qos())
                          .retained(t->is_retained())
                          .payload(t->get_payload())
                          .finalize();
        move_task.async_run([&] {
          try {
            auto src = jsoncons::json::parse(current_msg->get_payload());
            // LOG(INFO) << current_msg->to_string();
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
                state.actionstates.clear();  //

                //
                if ((ord.nodes.size() - ord.edges.size()) != 1) {
                  auto err = vda5050::state::Error();
                  err.error_level = vda5050::state::ErrorLevel::FATAL;
                  err.error_type = "node size not match edge size";
                  state.errors.push_back(err);
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = state;
                } else if (ord.edges.size() > 0) {
                  // state update
                  for (int i = 0; i < ord.nodes.size(); i++) {
                    auto n1 = vda5050::state::NodeState();
                    n1.node_id = ord.nodes.at(i).node_id;
                    n1.released = ord.nodes.at(i).released;
                    n1.sequence_id = ord.nodes.at(i).sequence_id;
                    n1.node_position = vda5050::state::NodePosition();
                    n1.node_position.value().x =
                        ord.nodes.at(i).node_position.value().x;
                    n1.node_position.value().y =
                        ord.nodes.at(i).node_position.value().y;
                    state.nodestates.push_back(n1);
                  }
                  // edge
                  for (int i = 0; i < ord.edges.size(); i++) {
                    auto e1 = vda5050::state::EdgeState();
                    e1.edge_id = ord.edges.at(i).edge_id;
                    e1.released = ord.edges.at(i).released;
                    e1.sequence_id = ord.edges.at(i).sequence_id;
                    if (ord.edges.at(i).edge_description.has_value()) {
                      e1.edge_description =
                          ord.edges.at(i).edge_description.value();
                    }
                    state.edgestates.push_back(e1);
                  }
                  vda_state = state;
                  while (status == Status::PASUED) {
                    std::chrono::milliseconds(50);
                  }
                  // move
                  state.nodestates.erase(state.nodestates.begin());
                  for (int i = 0; i < ord.nodes.size() - 1; i++) {
                    auto start_node = ord.nodes.at(i);
                    auto end_node = ord.nodes.at(i + 1);
                    LOG(INFO) << "move from " << start_node.node_id << " to "
                              << end_node.node_id;
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
                    for (auto i = 0; i < time; i = i + 50) {
                      while (status == Status::PASUED) {
                        std::chrono::milliseconds(50);
                      }
                      state.agv_position.value().x = s_x + vx / 1000 * i;
                      state.agv_position.value().y = s_y + vy / 1000 * i;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(50));
                      vda_state = state;
                    }
                    state.agv_position.value().x = e_x;
                    state.agv_position.value().y = e_y;
                    state.nodestates.erase(state.nodestates.begin());
                    state.edgestates.erase(state.edgestates.begin());
                    state.driving = false;
                    state.last_node_id = end_node.node_id;
                    vda_state = state;
                  }
                } else {
                  // only action
                  // state update
                  for (int i = 0; i < ord.nodes.size(); i++) {
                    auto n1 = vda5050::state::NodeState();
                    n1.node_id = ord.nodes.at(i).node_id;
                    n1.released = ord.nodes.at(i).released;
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
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state.actionstates.push_back(act);
                    }
                  }
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = state;
                  lock.unlock();
                  // action
                  for (int i = 0; i < ord.nodes.at(0).actions.size(); i++) {
                    float s_x = ord.nodes.at(0).node_position.value().x;
                    float s_y = ord.nodes.at(0).node_position.value().y;
                    float e_x = std::get<double>(ord.nodes.at(0)
                                                     .actions.at(i)
                                                     .action_parameters.value()
                                                     .at(0)
                                                     .value);
                    float e_y = std::get<double>(ord.nodes.at(0)
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
                    state.nodestates.erase(state.nodestates.begin());

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    state.agv_position.value().x = s_x;
                    state.agv_position.value().y = s_y;
                    state.driving = false;
                    state.last_node_id = ord.nodes.at(0).node_id;
                    state.actionstates.at(i).action_status =
                        vda5050::state::ActionStatus::FINISHED;
                    std::unique_lock<std::mutex> lock(mut);
                    vda_state = state;
                    lock.unlock();
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
          } catch (jsoncons::json_exception& ec) {
            LOG(ERROR) << ec.what();
          }
        });
      });
      // mqtt_client->set_func(".*", [&](mqtt::const_message_ptr t) {
      //   LOG(INFO) << t->get_topic();
      // });
      mqtt_client->set_func(
          prefix + "instantAction", [&](mqtt::const_message_ptr t) {
            current_act_msg = mqtt::message_ptr_builder()
                                  .topic(t->get_topic())
                                  .properties(t->get_properties())
                                  .qos(t->get_qos())
                                  .retained(t->is_retained())
                                  .payload(t->get_payload())
                                  .finalize();
            action_task.async_run([&] {
              // LOG(INFO) << current_act_msg->get_payload();
              try {
                auto src =
                    jsoncons::json::parse(current_act_msg->get_payload());
                auto err = vda5050::instantaction_validate(src);
                if (err.empty()) {
                  // do
                  auto act = vda5050::instantaction::InstantAction(src);
                  for (auto& x : act.actions) {
                    auto st = vda5050::state::ActionState();
                    st.action_id = x.action_id;
                    if (x.action_description) {
                      st.action_description = x.action_description.value();
                    }
                    st.action_status = vda5050::state::ActionStatus::FINISHED;
                    std::unique_lock<std::mutex> lock(mut);
                    vda_state.actionstates.push_back(st);
                    mqtt::message_ptr msg = mqtt::make_message(
                        "/" + interface_name + "/" + version + "/" +
                            manufacturer + "/" + serial_number + "/" + "state",
                        vda_state.to_json().as_string(), 0, false);
                    vda_state.header_id = vda_state.header_id + 1;
                    vda_state.timestamp =
                        get_time_fmt(std::chrono::system_clock::now());
                    // LOG(WARNING) << msg->get_payload_ref();
                    mqtt_client->publish(msg)->wait();
                    if (x.action_type ==
                        vda5050::instantaction::ActionType::startPause) {
                      if (status == Status::RUNNING) {
                        status = Status::PASUED;
                      }
                    } else if (x.action_type ==
                               vda5050::instantaction::ActionType::stopPause) {
                      if (status == Status::PASUED) {
                        status = Status::RUNNING;
                      }
                    }
                  }
                  LOG(INFO) << "instant action end";
                } else {
                  std::for_each(err.begin(), err.end(),
                                [](std::string e) { LOG(ERROR) << e; });
                }
              } catch (jsoncons::json_exception& ec) {
              }
            });
          });
    }
  }
  ~SimRabbit3() {
    action_task.stop();
    move_task.stop();
  }

 public:
  mqtt::connect_options con_ops;
  std::shared_ptr<MqttClient> mqtt_client;
  std::queue<mqtt::const_message_ptr> order_msgs;
  std::queue<mqtt::const_message_ptr> instantaction_msgs;
  vda5050::state::VDA5050State vda_state;
  std::thread th_state;
  std::thread th_pro;
  fa::taskpool_t move_task{1};
  fa::taskpool_t action_task{1};
  std::mutex mut;
  std::mutex pro_mut;
  std::condition_variable con;
  Status status{Status::RUNNING};
  //
  std::string broker_ip;
  int broker_port;
  std::string interface_name;
  std::string serial_number;
  std::string version;
  std::string manufacturer;
  //
  mqtt::message_ptr current_msg;
  mqtt::message_ptr current_act_msg;
};
INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv) {
  el::Loggers::getLogger("timer");
  SimRabbit3 agv("uagv", "tx1", "2.0", "rw");
  agv.set_mqtt_ops(agv.serial_number, "192.168.0.39");
  agv.start();
  std::cin.get();
}
