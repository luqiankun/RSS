#include "../../../include/component/util/taskpool.hpp"
#include "../../../include/component/util/tools.hpp"
#include "../../../include/component/vda5050/master.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/component/vda5050/vda5050insact.hpp"
#include "../../../include/component/vda5050/vda5050order.hpp"

class SimRabbit3 {
 public:
  enum class Status { RUNNING, PASUED };
  SimRabbit3(const std::string& interface_name,
             const std::string& serial_number, const std::string& vers,
             const std::string& manufacturer, const std::string& init_point,
             double x, double y)
      : interface_name(interface_name),
        serial_number(serial_number),
        manufacturer(manufacturer),
        version(vers) {
    vda_state.version = this->version;
    vda_state.manufacturer = manufacturer;
    vda_state.serial_number = serial_number;
    vda_state.order_id = "";
    vda_state.header_id = 0;
    vda_state.timestamp = get_time_fmt(std::chrono::system_clock::now());
    vda_state.order_update_id = 0;
    vda_state.last_node_id = init_point;
    vda_state.agv_position = vda5050::state::AgvPosition();
    vda_state.agv_position.value().x = x / 1000;
    vda_state.agv_position.value().y = y / 1000;
    vda_state.agv_position.value().theta = 0;
    vda_state.agv_position.value().map_id = "lyg";
    vda_state.last_node_seq_id = 0;
    vda_state.driving = false;
    vda_state.battery_state.battery_charge = 100;
    vda_state.battery_state.charging = false;
    vda_state.operating_mode = vda5050::state::OperMode::AUTOMATIC;
    vda_state.safetystate.estop = vda5050::state::SafetyStatus::AUTOACK;
    vda_state.safetystate.field_violation = false;
  }
  void process_msgs();
  void set_mqtt_ops(const std::string& name, const std::string& server_ip,
                    int server_port = 1883, bool ssl = false) {
    if (ssl) {
      auto addr = "ssl://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<vda5050::MqttClient>(addr, name);
    } else {
      auto addr = "tcp://" + server_ip + ":" + std::to_string(server_port);
      mqtt_client = std::make_shared<vda5050::MqttClient>(addr, name);
    }
    con_ops.set_automatic_reconnect(true);
    con_ops.set_keep_alive_interval(5000);
    con_ops.set_connect_timeout(2);
    int ver = static_cast<int>(std::stod(version));
    auto vda_version = "v" + std::to_string(ver);
    auto prefix = interface_name + "/" + vda_version + "/" + manufacturer +
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
        long long send_id{0};
        int ver = static_cast<int>(std::stod(version));
        auto vda_version = "v" + std::to_string(ver);
        auto prefix = interface_name + "/" + vda_version + "/" + manufacturer +
                      "/" + serial_number + "/";
        while (true) {
          if (mqtt_client->is_connected()) {
            std::unique_lock<std::mutex> lock(pro_mut);
            con.wait_for(lock, std::chrono::milliseconds(20));
            try {
              std::unique_lock<std::mutex> lock_(mut);
              vda_state.header_id = send_id;
              mqtt::message_ptr msg = mqtt::make_message(
                  prefix + "state", vda_state.to_json().as_string(), 0, false);
              // LOG(WARNING) << serial_number << " " << send_id << " "
              //              << vda_state.header_id;
              send_id += 1;
              vda_state.timestamp =
                  get_time_fmt(std::chrono::system_clock::now());
              mqtt_client->publish(msg)->wait();
            } catch (mqtt::exception& ec) {
              LOG(ERROR) << ec.get_error_str();
            }
            // LOG(INFO) << vda_state.order_id << " " << vda_state.header_id;
          }
        }
      });
      mqtt_client->on();
      mqtt_client->set_connected_handler([&](std::string) {
        LOG(INFO) << serial_number << "  ONLINE";
        int ver = static_cast<int>(std::stod(version));
        auto vda_version = "v" + std::to_string(ver);
        auto prefix = interface_name + "/" + vda_version + "/" + manufacturer +
                      "/" + serial_number + "/";
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
        mqtt_client->subscribe(interface_name + "/" + vda_version + "/" +
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
      int ver = static_cast<int>(std::stod(version));
      auto vda_version = "v" + std::to_string(ver);
      const std::string prefix = interface_name + "/" + vda_version + "/" +
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
            LOG(INFO) << current_msg->get_payload();
            auto src = jsoncons::json::parse(current_msg->get_payload());
            // LOG(INFO) << current_msg->to_string();
            auto err = vda5050::order_validate(src);
            if (err.empty()) {
              // do
              auto ord = vda5050::order::VDA5050Order(src);
              if (vda_state.order_id != ord.order_id) {
                // new order
                auto state = std::shared_ptr<vda5050::state::VDA5050State>(
                    new vda5050::state::VDA5050State(vda_state));
                state->order_id = ord.order_id;
                state->order_update_id = ord.order_update_id;
                state->errors.clear();
                state->nodestates.clear();
                state->edgestates.clear();
                state->actionstates.clear();  //

                //
                if ((ord.nodes.size() - ord.edges.size()) != 1) {
                  auto err = vda5050::state::Error();
                  err.error_level = vda5050::state::ErrorLevel::FATAL;
                  err.error_type = "node size not match edge size";
                  state->errors.push_back(err);
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
                  con.notify_one();
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
                    state->nodestates.push_back(n1);
                    for (auto& x : ord.nodes.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  // edge
                  for (int i = 0; i < ord.edges.size(); i++) {
                    auto e1 = vda5050::state::EdgeState();
                    e1.edge_id = ord.edges.at(i).edge_id;
                    e1.released = ord.edges.at(i).released;
                    e1.sequence_id = ord.edges.at(i).sequence_id;
                    e1.edge_description = ord.edges.at(i).edge_description;
                    state->edgestates.push_back(e1);
                    for (auto& x : ord.edges.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
                  con.notify_one();
                  lock.unlock();
                  while (status == Status::PASUED) {
                    std::chrono::milliseconds(50);
                  }
                  //
                  //  front action
                  //   move
                  // first node
                  bool first{true};
                  {
                    vda5050::order::Node start_node;
                    vda5050::order::Node end_node;
                    for (auto& x : ord.nodes) {
                      if (x.node_id == ord.edges.at(0).start_node_id) {
                        start_node = x;
                        break;
                      }
                    }
                    for (auto& x : ord.nodes) {
                      if (x.node_id == ord.edges.at(0).end_node_id) {
                        end_node = x;
                        break;
                      }
                    }
                    assert(!start_node.node_id.empty());
                    assert(!end_node.node_id.empty());
                    if (first) {
                      state->nodestates.erase(
                          state->nodestates.begin());  // move out
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;  // update once
                      con.notify_one();
                      lock.unlock();
                      for (int j = 0; j < start_node.actions.size(); j++) {
                        LOG(INFO)
                            << "action " << start_node.actions.at(j).action_id;
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(500));
                        for (auto& x : state->actionstates) {
                          if (x.action_id ==
                              start_node.actions.at(j).action_id) {
                            x.action_status =
                                vda5050::state::ActionStatus::FINISHED;
                            std::unique_lock<std::mutex> lock(mut);
                            vda_state = *state;
                            con.notify_one();
                            lock.unlock();
                            break;
                          }
                        }
                      }
                      first = false;
                    }
                    // move to end point
                    LOG(INFO) << "move from " << start_node.node_id << " to "
                              << end_node.node_id;
                    float s_x = start_node.node_position.value().x;
                    float s_y = start_node.node_position.value().y;
                    float e_x = end_node.node_position.value().x;
                    float e_y = end_node.node_position.value().y;
                    // move
                    auto vx = 5 * cos(std::atan2(e_y - s_y, e_x - s_x));
                    auto vy = 5 * sin(std::atan2(e_y - s_y, e_x - s_x));
                    LOG(INFO) << "Vx " << vx << " Vy " << vy;
                    auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                     (e_y - s_y) * (e_y - s_y)) *
                                1000 / 5;  // ms
                    state->driving = true;
                    for (auto i = 0; i < time; i = i + 50) {
                      while (status == Status::PASUED) {
                        std::chrono::milliseconds(50);
                      }
                      state->agv_position.value().x = s_x + vx / 1000 * i;
                      state->agv_position.value().y = s_y + vy / 1000 * i;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(50));
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                    }
                    state->agv_position.value().x = e_x;
                    state->agv_position.value().y = e_y;
                    std::for_each(
                        state->edgestates.begin(), state->edgestates.end(),
                        [&](auto& x) { LOG(INFO) << x.edge_id << "\t"; });
                    LOG(INFO) << state->edgestates.front().edge_id;
                    state->edgestates.erase(state->edgestates.begin());
                    LOG(INFO) << state->edgestates.front().edge_id;
                    state->driving = false;
                    state->last_node_id = end_node.node_id;
                    std::unique_lock<std::mutex> lock(mut);
                    vda_state = *state;
                    con.notify_one();
                    lock.unlock();
                    for (int j = 0; j < end_node.actions.size(); j++) {
                      LOG(INFO)
                          << "action " << end_node.actions.at(j).action_id;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      for (auto& x : state->actionstates) {
                        if (x.action_id == end_node.actions.at(j).action_id) {
                          x.action_status =
                              vda5050::state::ActionStatus::FINISHED;
                          std::unique_lock<std::mutex> lock(mut);
                          vda_state = *state;
                          con.notify_one();
                          break;
                        }
                      }
                    }
                    for (int j = 0; j < ord.edges.at(0).actions.size(); j++) {
                      LOG(INFO) << "action "
                                << ord.edges.at(0).actions.at(j).action_id;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      for (auto& x : state->actionstates) {
                        if (x.action_id ==
                            ord.edges.at(0).actions.at(j).action_id) {
                          x.action_status =
                              vda5050::state::ActionStatus::FINISHED;
                          std::unique_lock<std::mutex> lock(mut);
                          vda_state = *state;
                          con.notify_one();
                          break;
                        }
                      }
                    }
                    state->nodestates.erase(state->nodestates.begin());
                  }
                  lock.lock();
                  vda_state = *state;
                  con.notify_one();
                  std::this_thread::sleep_for(std::chrono::milliseconds(700));
                  std::cout << "--------------------------------------------\n";
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
                    state->nodestates.push_back(n1);
                    for (auto& x : ord.nodes.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
                  con.notify_one();
                  lock.unlock();
                  // action
                  for (int i = 0; i < ord.nodes.at(0).actions.size(); i++) {
                    float s_x = ord.nodes.at(0).node_position.value().x;
                    float s_y = ord.nodes.at(0).node_position.value().y;
                    auto it_x = std::find_if(ord.nodes.at(0)
                                                 .actions.at(i)
                                                 .action_parameters.value()
                                                 .begin(),
                                             ord.nodes.at(0)
                                                 .actions.at(i)
                                                 .action_parameters.value()
                                                 .end(),
                                             [&](auto& x) {
                                               return x.key == "location_x" ||
                                                      x.key == "point_x";
                                             });
                    auto it_y = std::find_if(ord.nodes.at(0)
                                                 .actions.at(i)
                                                 .action_parameters.value()
                                                 .begin(),
                                             ord.nodes.at(0)
                                                 .actions.at(i)
                                                 .action_parameters.value()
                                                 .end(),
                                             [&](auto& x) {
                                               return x.key == "location_y" ||
                                                      x.key == "point_y";
                                             });
                    float e_x = std::get<double>(it_x->value);
                    float e_y = std::get<double>(it_y->value);
                    auto vx = 5 * cos(std::atan2(e_y - s_y, e_x - s_x));
                    auto vy = 5 * sin(std::atan2(e_y - s_y, e_x - s_x));
                    LOG(INFO) << "Vx " << vx << " Vy " << vy;
                    auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                     (e_y - s_y) * (e_y - s_y)) *
                                1000 / 5;  // ms
                    state->driving = true;
                    state->nodestates.erase(state->nodestates.begin());

                    for (auto i = 0; i < time; i = i + 50) {
                      if (!state->agv_position.has_value()) {
                        state->agv_position = vda5050::state::AgvPosition();
                      }
                      state->agv_position.value().x = s_x + vx / 1000 * i;
                      state->agv_position.value().y = s_y + vy / 1000 * i;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(50));
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    }
                    LOG(INFO) << data::model::Actions::get_type(
                        ord.nodes.at(0).actions.at(i).action_type);
                    if (ord.nodes.at(0).actions.at(i).action_type ==
                        vda5050::order::ActionType::CHARGE) {
                      LOG(INFO) << "=====charging=====";
                      state->driving = false;
                      state->last_node_id = "";
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                      util_task.async_run([&]() mutable {
                        while (vda_state.battery_state.battery_charge < 100) {
                          std::unique_lock<std::mutex> lock(mut);
                          if (vda_state.driving) {
                            break;
                          }
                          vda_state.battery_state.battery_charge += 1;
                          vda_state.battery_state.charging = true;
                          lock.unlock();
                          std::this_thread::sleep_for(
                              std::chrono::milliseconds(100));
                        }
                        std::unique_lock<std::mutex> lock(mut);
                        vda_state.battery_state.charging = false;
                      });
                      LOG(INFO) << "=====charging=====";
                    } else if (ord.nodes.at(0).actions.at(i).action_type ==
                               vda5050::order::ActionType::PARK) {
                      LOG(INFO) << "=====park=====";
                      state->driving = false;
                      state->last_node_id.clear();
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                      LOG(INFO) << "=====park=====";
                    } else if (ord.nodes.at(0).actions.at(i).action_type ==
                               vda5050::order::ActionType::MOVE) {
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      state->agv_position.value().x = s_x;
                      state->agv_position.value().y = s_y;
                      state->driving = false;
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    } else {
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      state->agv_position.value().x = s_x;
                      state->agv_position.value().y = s_y;
                      state->driving = false;
                      // state->last_node_id.clear();
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    }
                    LOG(INFO) << "action end";
                  }
                }
              } else if (vda_state.order_id == src["orderId"]) {
                // update order
                // new order
                auto state = std::shared_ptr<vda5050::state::VDA5050State>(
                    new vda5050::state::VDA5050State(vda_state));
                state->order_id = ord.order_id;
                state->order_update_id = ord.order_update_id;
                state->errors.clear();
                state->nodestates.clear();
                state->edgestates.clear();
                state->actionstates.clear();  //

                //
                if ((ord.nodes.size() - ord.edges.size()) != 1) {
                  auto err = vda5050::state::Error();
                  err.error_level = vda5050::state::ErrorLevel::FATAL;
                  err.error_type = "node size not match edge size";
                  state->errors.push_back(err);
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
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
                    state->nodestates.push_back(n1);
                    for (auto& x : ord.nodes.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  // edge
                  for (int i = 0; i < ord.edges.size(); i++) {
                    auto e1 = vda5050::state::EdgeState();
                    e1.edge_id = ord.edges.at(i).edge_id;
                    e1.released = ord.edges.at(i).released;
                    e1.sequence_id = ord.edges.at(i).sequence_id;
                    e1.edge_description = ord.edges.at(i).edge_description;
                    state->edgestates.push_back(e1);
                    for (auto& x : ord.edges.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
                  con.notify_one();
                  lock.unlock();
                  while (status == Status::PASUED) {
                    std::chrono::milliseconds(50);
                  }
                  //
                  //  front action
                  //   move
                  // first node
                  bool first{true};
                  {
                    vda5050::order::Node start_node;
                    vda5050::order::Node end_node;
                    for (auto& x : ord.nodes) {
                      if (x.node_id == ord.edges.at(0).start_node_id) {
                        start_node = x;
                        break;
                      }
                    }
                    for (auto& x : ord.nodes) {
                      if (x.node_id == ord.edges.at(0).end_node_id) {
                        end_node = x;
                        break;
                      }
                    }
                    assert(!start_node.node_id.empty());
                    assert(!end_node.node_id.empty());
                    if (first) {
                      state->nodestates.erase(
                          state->nodestates.begin());  // move out
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;  // update once
                      con.notify_one();
                      lock.unlock();
                      for (int j = 0; j < start_node.actions.size(); j++) {
                        LOG(INFO)
                            << "action " << start_node.actions.at(j).action_id;
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(500));
                        for (auto& x : state->actionstates) {
                          if (x.action_id ==
                              start_node.actions.at(j).action_id) {
                            x.action_status =
                                vda5050::state::ActionStatus::FINISHED;
                            std::unique_lock<std::mutex> lock(mut);
                            vda_state = *state;
                            con.notify_one();
                            lock.unlock();
                            break;
                          }
                        }
                      }
                      first = false;
                    }
                    // move to end point
                    LOG(INFO) << "move from " << start_node.node_id << " to "
                              << end_node.node_id;
                    float s_x = start_node.node_position.value().x;
                    float s_y = start_node.node_position.value().y;
                    float e_x = end_node.node_position.value().x;
                    float e_y = end_node.node_position.value().y;
                    // move
                    auto vx = 5 * cos(std::atan2(e_y - s_y, e_x - s_x));
                    auto vy = 5 * sin(std::atan2(e_y - s_y, e_x - s_x));
                    LOG(INFO) << "Vx " << vx << " Vy " << vy;
                    auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                     (e_y - s_y) * (e_y - s_y)) *
                                1000 / 5;  // ms
                    state->driving = true;
                    for (auto i = 0; i < time; i = i + 50) {
                      while (status == Status::PASUED) {
                        std::chrono::milliseconds(50);
                      }
                      state->agv_position.value().x = s_x + vx / 1000 * i;
                      state->agv_position.value().y = s_y + vy / 1000 * i;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(50));
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                    }
                    state->agv_position.value().x = e_x;
                    state->agv_position.value().y = e_y;
                    std::for_each(
                        state->edgestates.begin(), state->edgestates.end(),
                        [&](auto& x) { LOG(INFO) << x.edge_id << "\t"; });
                    LOG(INFO) << state->edgestates.front().edge_id;
                    state->edgestates.erase(state->edgestates.begin());
                    LOG(INFO) << state->edgestates.front().edge_id;
                    state->driving = false;
                    state->last_node_id = end_node.node_id;
                    std::unique_lock<std::mutex> lock(mut);
                    vda_state = *state;
                    con.notify_one();
                    lock.unlock();
                    for (int j = 0; j < end_node.actions.size(); j++) {
                      LOG(INFO)
                          << "action " << end_node.actions.at(j).action_id;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      for (auto& x : state->actionstates) {
                        if (x.action_id == end_node.actions.at(j).action_id) {
                          x.action_status =
                              vda5050::state::ActionStatus::FINISHED;
                          std::unique_lock<std::mutex> lock(mut);
                          vda_state = *state;
                          con.notify_one();
                          break;
                        }
                      }
                    }
                    for (int j = 0; j < ord.edges.at(0).actions.size(); j++) {
                      LOG(INFO) << "action "
                                << ord.edges.at(0).actions.at(j).action_id;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      for (auto& x : state->actionstates) {
                        if (x.action_id ==
                            ord.edges.at(0).actions.at(j).action_id) {
                          x.action_status =
                              vda5050::state::ActionStatus::FINISHED;
                          std::unique_lock<std::mutex> lock(mut);
                          vda_state = *state;
                          con.notify_one();
                          break;
                        }
                      }
                    }
                    state->nodestates.erase(state->nodestates.begin());
                  }
                  lock.lock();
                  vda_state = *state;
                  con.notify_one();
                  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                  std::cout << "--------------------------------------------\n";
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
                    state->nodestates.push_back(n1);
                    for (auto& x : ord.nodes.at(i).actions) {
                      auto act = vda5050::state::ActionState();
                      act.action_id = x.action_id;
                      act.action_status = vda5050::state::ActionStatus::WAITING;
                      state->actionstates.push_back(act);
                    }
                  }
                  std::unique_lock<std::mutex> lock(mut);
                  vda_state = *state;
                  con.notify_one();
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
                    auto vx = 5 * cos(std::atan2(e_y - s_y, e_x - s_x));
                    auto vy = 5 * sin(std::atan2(e_y - s_y, e_x - s_x));
                    LOG(INFO) << "Vx " << vx << " Vy " << vy;
                    auto time = sqrt((e_x - s_x) * (e_x - s_x) +
                                     (e_y - s_y) * (e_y - s_y)) *
                                1000 / 5;  // ms
                    state->driving = true;
                    state->nodestates.erase(state->nodestates.begin());

                    for (auto i = 0; i < time; i = i + 50) {
                      if (!state->agv_position.has_value()) {
                        state->agv_position = vda5050::state::AgvPosition();
                      }
                      state->agv_position.value().x = s_x + vx / 1000 * i;
                      state->agv_position.value().y = s_y + vy / 1000 * i;
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(50));
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    }
                    LOG(INFO) << data::model::Actions::get_type(
                        ord.nodes.at(0).actions.at(i).action_type);
                    if (ord.nodes.at(0).actions.at(i).action_type ==
                        vda5050::order::ActionType::CHARGE) {
                      LOG(INFO) << "=====charging=====";
                      state->driving = false;
                      state->last_node_id = "";
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                      util_task.async_run([&]() mutable {
                        while (vda_state.battery_state.battery_charge < 100) {
                          std::unique_lock<std::mutex> lock(mut);
                          if (vda_state.driving) {
                            break;
                          }
                          vda_state.battery_state.battery_charge += 1;
                          vda_state.battery_state.charging = true;
                          lock.unlock();
                          std::this_thread::sleep_for(
                              std::chrono::milliseconds(100));
                        }
                        std::unique_lock<std::mutex> lock(mut);
                        vda_state.battery_state.charging = false;
                      });
                      LOG(INFO) << "=====charging=====";
                    } else if (ord.nodes.at(0).actions.at(i).action_type ==
                               vda5050::order::ActionType::PARK) {
                      LOG(INFO) << "=====park=====";
                      state->driving = false;
                      state->last_node_id.clear();
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                      lock.unlock();
                      LOG(INFO) << "=====park=====";
                    } else if (ord.nodes.at(0).actions.at(i).action_type ==
                               vda5050::order::ActionType::MOVE) {
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      state->agv_position.value().x = s_x;
                      state->agv_position.value().y = s_y;
                      state->driving = false;
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    } else {
                      std::this_thread::sleep_for(
                          std::chrono::milliseconds(500));
                      state->agv_position.value().x = s_x;
                      state->agv_position.value().y = s_y;
                      state->driving = false;
                      // state->last_node_id.clear();
                      state->actionstates.at(i).action_status =
                          vda5050::state::ActionStatus::FINISHED;
                      std::unique_lock<std::mutex> lock(mut);
                      vda_state = *state;
                      con.notify_one();
                    }
                    LOG(INFO) << "action end";
                  }
                }
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
                    int ver = static_cast<int>(std::stod(version));
                    auto vda_version = "v" + std::to_string(ver);
                    mqtt::message_ptr msg = mqtt::make_message(
                        interface_name + "/" + vda_version + "/" +
                            manufacturer + "/" + serial_number + "/" + "state",
                        vda_state.to_json().as_string(), 0, false);
                    vda_state.timestamp =
                        get_time_fmt(std::chrono::system_clock::now());
                    // LOG(WARNING) << msg->get_payload_ref();
                    // mqtt_client->publish(msg)->wait();
                    if (x.action_type ==
                        vda5050::instantaction::ActionType::STARTPAUSE) {
                      if (status == Status::RUNNING) {
                        status = Status::PASUED;
                      }
                    } else if (x.action_type ==
                               vda5050::instantaction::ActionType::STOPPAUSE) {
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
    util_task.stop();
  }

 public:
  mqtt::connect_options con_ops;
  std::shared_ptr<vda5050::MqttClient> mqtt_client;
  std::queue<mqtt::const_message_ptr> order_msgs;
  std::queue<mqtt::const_message_ptr> instantaction_msgs;
  vda5050::state::VDA5050State vda_state;
  std::thread th_state;
  std::thread th_pro;
  fa::taskpool_t move_task{1};
  fa::taskpool_t action_task{1};
  fa::taskpool_t util_task{4};
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
  if (argc < 2) {
    std::cerr << "usage: ./rabbit3_test ip only\n"
              << "ip: ip" << "\n"
              << "only: true:one vehilce false:four vehilce\n";
    return -1;
  }
  std::string ip;
  bool only{true};
  if (argc == 2) {
    ip = std::string(argv[1]);
  } else {
    ip = std::string(argv[1]);
    only = std::string(argv[2]) == "true" ? true : false;
  }
  LOG(INFO) << "ip: " << ip << "\t" << "only: " << only << "\n";

  SimRabbit3 agv1("uagv", "tx1", "2.0", "rw", "P1", 45150, -107450);
  agv1.set_mqtt_ops(agv1.serial_number, ip);
  SimRabbit3 agv2("uagv", "tx2", "2.0", "rw", "P4", 74475, -107450);
  agv2.set_mqtt_ops(agv2.serial_number, ip);
  SimRabbit3 agv3("uagv", "tx3", "2.0", "tx", "P1", 7000, 19500);
  agv3.set_mqtt_ops(agv3.serial_number, ip);
  SimRabbit3 agv4("uagv", "tx4", "2.0", "tx", "Point-0001", 500, 19500);
  agv4.set_mqtt_ops(agv4.serial_number, ip);
  if (only) {
    agv3.start();
    agv4.start();
  } else {
    agv1.start();
    agv2.start();
    agv3.start();
    agv4.start();
  }
  std::cin.get();
}
