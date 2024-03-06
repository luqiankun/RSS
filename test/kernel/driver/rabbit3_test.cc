#include "../../../include/component/tools/mqtt/mqtt.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
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
    state["version"] = version;
    state["manufacturer"] = manufacturer;
    state["serialNumber"] = serial_number;
    state["orderId"] = "";
    state["headerId"] = 0;
    state["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
    state["orderUpdateId"] = 0;
    state["lastNodeId"] = "P3";
    state["agvPosition"]["x"] = 64387;
    state["agvPosition"]["y"] = -107450;
    state["agvPosition"]["theta"] = 0;
    state["agvPosition"]["mapId"] = "";
    state["agvPosition"]["positionInitialized"] = false;
    state["lastNodeSequenceId"] = 0;
    state["nodeStates"] = nlohmann::json::array();
    state["edgeStates"] = nlohmann::json::array();
    state["driving"] = false;
    state["actionStates"] = nlohmann::json::array();
    state["batteryState"]["batteryCharge"] = 90;
    state["batteryState"]["charging"] = false;
    state["operatingMode"] = "AUTOMATIC";
    state["errors"] = nlohmann::json::array();
    state["safetyState"]["eStop"] = "AUTOACK";
    state["safetyState"]["fieldViolation"] = false;
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
              LOG(INFO) << step->get_topic();
              try {
                auto src = nlohmann::json::parse(step->get_payload());
                auto err = vda5050::order_validate(src);
                if (err.empty()) {
                  // do
                  if (state["orderId"] != src["order"]) {
                    // new order
                    state["orderId"] = src["orderId"];
                    //
                    if (src["nodes"].size() == 2) {
                      LOG(INFO) << "move ....";
                      status = Status::RUNNING;
                      std::string start_node = src["nodes"].at(0)["nodeId"];
                      std::string end_node = src["nodes"].at(1)["nodeId"];
                      float s_x =
                          src["nodes"].at(0)["nodePosition"]["x"].get<float>();
                      float s_y =
                          src["nodes"].at(0)["nodePosition"]["y"].get<float>();
                      float e_x =
                          src["nodes"].at(1)["nodePosition"]["x"].get<float>();
                      float e_y =
                          src["nodes"].at(1)["nodePosition"]["y"].get<float>();
                      nlohmann::json n1;
                      n1["nodeId"] = start_node;
                      n1["released"] = false;
                      n1["sequenceId"] = 0;
                      n1["nodePosition"] = src["nodes"].at(0)["nodePosition"];
                      nlohmann::json n2;
                      n2["nodeId"] = end_node;
                      n2["released"] = false;
                      n2["sequenceId"] = 0;
                      n2["nodePosition"] = src["nodes"].at(1)["nodePosition"];
                      state["nodeStates"].clear();
                      state["nodeStates"].push_back(n1);
                      state["nodeStates"].push_back(n2);
                      state["lastNodeId"] = start_node;
                      nlohmann::json e1;
                      e1["edgeId"] = src["edges"].at(0)["edgeId"];
                      e1["sequenceId"] = 0;
                      e1["released"] = false;
                      state["edgeStates"].clear();
                      state["edgeStates"].push_back(e1);
                      state["agvPosition"]["x"] = s_x;
                      state["agvPosition"]["y"] = s_y;
                      state["agvPosition"]["theta"] = 0;
                      state["agvPosition"]["mapId"] = "";
                      state["agvPosition"]["positionInitialized"] = false;

                      // move
                      auto vx = 5000 * cos(std::atan2(e_y - s_y, e_x - s_x));
                      auto vy = 5000 * sin(std::atan2(e_y - s_y, e_x - s_x));
                      LOG(INFO) << "Vx " << vx << " Vy " << vy;
                      auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                       (e_y - s_y) * (e_y - s_y)) *
                                  1000 / 5000;  // ms
                      state["driving"] = true;
                      state["nodeStates"].at(0)["released"] = true;
                      for (auto i = 0; i < time; i = i + 50) {
                        state["agvPosition"]["x"] = s_x + vx / 5000 * i;
                        state["agvPosition"]["y"] = s_y + vy / 5000 * i;
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(50));
                      }
                      state["agvPosition"]["x"] = e_x;
                      state["agvPosition"]["y"] = e_y;
                      state["nodeStates"].at(1)["released"] = true;
                      state["edgeStates"].at(0)["released"] = true;
                      state["driving"] = false;
                      state["lastNodeId"] = end_node;
                      status = Status::IDLE;
                      LOG(INFO) << "move end";

                    } else if (src["nodes"].size() == 1) {
                      LOG(INFO) << "action";
                      state["edgeStates"].clear();
                      // action
                      status = Status::RUNNING;
                      std::string start_node = src["nodes"].at(0)["nodeId"];
                      float s_x =
                          src["nodes"].at(0)["nodePosition"]["x"].get<float>();
                      float s_y =
                          src["nodes"].at(0)["nodePosition"]["y"].get<float>();
                      nlohmann::json n1;
                      n1["nodeId"] = start_node;
                      n1["released"] = false;
                      n1["sequenceId"] = 0;
                      n1["nodePosition"] = src["nodes"].at(0)["nodePosition"];
                      state["nodeStates"].clear();
                      state["nodeStates"].push_back(n1);
                      state["lastNodeId"] = start_node;
                      float e_x = src["nodes"]
                                      .at(0)["actions"]
                                      .at(0)["actionParameters"]
                                      .at(0)["value"]
                                      .get<float>();
                      float e_y = src["nodes"]
                                      .at(0)["actions"]
                                      .at(0)["actionParameters"]
                                      .at(1)["value"]
                                      .get<float>();
                      state["agvPosition"]["x"] = s_x;
                      state["agvPosition"]["y"] = s_y;
                      state["agvPosition"]["theta"] = 0;
                      state["agvPosition"]["mapId"] = "";
                      state["agvPosition"]["positionInitialized"] = false;
                      state["actionStates"] = nlohmann::json::array();
                      nlohmann::json act_st;
                      act_st["actionId"] =
                          src["nodes"].at(0)["actions"].at(0)["actionId"];
                      act_st["actionStatus"] = "RUNNING";
                      state["actionStates"].push_back(act_st);
                      auto vx = 5000 * cos(std::atan2(e_y - s_y, e_x - s_x));
                      auto vy = 5000 * sin(std::atan2(e_y - s_y, e_x - s_x));

                      auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                       (e_y - s_y) * (e_y - s_y)) *
                                  1000 / 5000;  // ms
                      state["driving"] = true;
                      state["nodeStates"].at(0)["released"] = true;
                      for (auto i = 0; i < time; i = i + 50) {
                        state["agvPosition"]["x"] = s_x + vx / 5000 * i;
                        state["agvPosition"]["y"] = s_y + vy / 5000 * i;
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(50));
                      }
                      state["agvPosition"]["x"] = e_x;
                      state["agvPosition"]["y"] = e_y;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(1000));
                      state["agvPosition"]["x"] = s_x;
                      state["agvPosition"]["y"] = s_y;
                      state["driving"] = false;
                      state["actionStates"].at(0)["actionStatus"] = "FINISHED";
                      status = Status::IDLE;
                      LOG(INFO) << "action end";
                    }

                  } else if (state["orderId"] == src["orderId"]) {
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
        while (true) {
          if (mqtt_client->is_connected()) {
            auto prefix = "/" + interface_name + "/" + version + "/" +
                          manufacturer + "/" + serial_number + "/";
            mqtt::message_ptr msg = std::make_shared<mqtt::message>();
            msg->set_qos(0);
            msg->set_topic(prefix + "state");
            int id = state["headerId"].get<int>();
            state["headerId"] = ++id;
            state["timestamp"] = get_time_fmt(std::chrono::system_clock::now());
            msg->set_payload(state.dump());
            mqtt_client->publish(msg);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
        LOG(INFO) << t->get_topic();
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
  nlohmann::json state;
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
