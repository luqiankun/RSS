
#include "../../../include/kernel/driver/vehicle.hpp"

#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/dispatch/dispatch.hpp"
namespace kernel {
namespace driver {
void Vehicle::execute_action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  pool.async_run([=] {
    auto op_ret = action(dest);
    if (!op_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = std::chrono::system_clock::now();
      CLOG(WARNING, "driver") << current_order->name << " action : "
                              << " failed";
    } else {
      CLOG(INFO, "driver") << current_order->name << " action : "
                           << " ok";
    }
    if (current_command) {
      current_command->vehicle_execute_cb(op_ret);
    }
  });
}
void Vehicle::execute_move(std::shared_ptr<data::order::Step> step) {
  pool.async_run([=] {
    auto move_ret = move(step);
    if (!move_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = std::chrono::system_clock::now();
      CLOG(WARNING, "driver") << current_order->name << " action : "
                              << " move : " << step->name.c_str() << " failed";
    } else {
      CLOG(INFO, "driver") << current_order->name
                           << " move : " << step->name.c_str() << " ok";
    }

    if (current_command) {
      current_command->vehicle_execute_cb(move_ret);
    }
  });
}

void Vehicle::cancel_all_order() {
  for (auto& ord : orders) {
    if (ord->state == data::order::TransportOrder::State::RAW ||
        ord->state == data::order::TransportOrder::State::ACTIVE ||
        ord->state == data::order::TransportOrder::State::DISPATCHABLE ||
        ord->state == data::order::TransportOrder::State::BEING_PROCESSED)
      ord->state = data::order::TransportOrder::State::WITHDRAWL;
  }
  orders.clear();
}

void Vehicle::run() { init(); }
void Vehicle::close() {
  task_run = false;
  pool.stop();
  // if (run_th.joinable()) {
  //   run_th.join();
  // }
  current_command.reset();
}
Vehicle::~Vehicle() {
  close();
  CLOG(INFO, "driver") << name << " close";
}

void Vehicle::plan_route() {
  // TODO  solver
  auto start_planner = current_point;
  std::shared_ptr<data::model::Point> end_planner;
  for (auto& op : current_order->driverorders) {
    auto dest = op->destination->destination.lock();
    auto start_check = resource.lock()->find(dest->name);
    auto destination = orderpool.lock()->res_to_destination(
        start_check.second, op->destination->operation);
    op->destination = destination;
    bool able{false};
    if (start_check.first == allocate::ResourceManager::ResType::Point) {
      end_planner =
          std::dynamic_pointer_cast<data::model::Point>(start_check.second);
    } else if (start_check.first ==
               allocate::ResourceManager::ResType::Location) {
      auto temp =
          std::dynamic_pointer_cast<data::model::Location>(start_check.second);
      if (!temp->locked) {
        end_planner = temp->link.lock();
      }
    }
    if (!end_planner) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
      CLOG(WARNING, "driver") << current_order->name << " can not find obj";
    }
    auto path = planner.lock()->find_paths(start_planner, end_planner);
    if (path.empty()) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
      CLOG(WARNING, "driver") << current_order->name << " can not routable";
    } else {
      auto driverorder = orderpool.lock()->route_to_driverorder(
          resource.lock()->paths_to_route(path.front()), destination);
      driverorder->transport_order = current_order;
      op = driverorder;
      able = true;
    }
    if (!able) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
    }
  }
}

void Vehicle::get_next_ord() {
  // 获取新订单
  for (;;) {
    if (orders.empty()) {
      proc_state = ProcState::AWAITING_ORDER;
      state = State::IDLE;
      break;
    } else {
      CLOG(INFO, "driver") << "_______________";
      current_order = orders.front();
      orders.pop_front();
      // TODO  solver
      if (current_order->state !=
          data::order::TransportOrder::State::BEING_PROCESSED) {
        current_order.reset();
        continue;
      }
      // route
      plan_route();
      if (current_order->state ==
          data::order::TransportOrder::State::UNROUTABLE) {
        current_order.reset();
        continue;
      } else {
        break;
      }
    }
  }
  if (current_order) {
    state = State::EXECUTING;
    proc_state = ProcState::PROCESSING_ORDER;
    next_command();
  } else {
    proc_state = ProcState::AWAITING_ORDER;
    state = State::IDLE;
    idle_time = std::chrono::system_clock::now();
    CLOG(INFO, "driver") << "now is idle " << get_time_fmt(idle_time);
  }
}

void Vehicle::command_done() {
  bool ord_shutdown{false};
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
    // 订单取消
    future_claim_resources.clear();
    CLOG(ERROR, "driver") << current_order->name << " withdrawl.";
    current_order.reset();
    get_next_ord();
    return;
  }
  if (current_order->dead_time < std::chrono::system_clock::now()) {
    CLOG(ERROR, "driver") << current_order->name << " timeout.";
    current_order->state = data::order::TransportOrder::State::FAILED;
  }
  if (current_order->state == data::order::TransportOrder::State::FAILED) {
    // 订单失败
    future_claim_resources.clear();
    current_order.reset();
    get_next_ord();
    return;
  }
  // 完成step or action
  auto dr = (current_order->driverorders[current_order->current_driver_index]);
  if (dr->state == data::order::DriverOrder::State::FINISHED ||
      dr->state == data::order::DriverOrder::State::FAILED) {
    current_order->current_driver_index += 1;
  }
  if (current_order->driverorders.size() <=
      current_order->current_driver_index) {
    current_order->state = data::order::TransportOrder::State::FINISHED;
    CLOG(INFO, "driver") << current_order->name << " finished";
    current_order->end_time = std::chrono::system_clock::now();
    current_order.reset();
    current_command.reset();
    future_claim_resources.clear();
    get_next_ord();
  } else {
    // 继续执行订单
    if (current_order->state ==
        data::order::TransportOrder::State::BEING_PROCESSED) {
      next_command();
      state = State::EXECUTING;
      proc_state = ProcState::PROCESSING_ORDER;
    } else {
      // 订单取消了或失败了
      command_done();
    }
  }
}

std::string Vehicle::get_state() {
  if (state == State::IDLE) {
    return "IDLE";
  } else if (state == State::EXECUTING) {
    return "EXECUTING";

  } else if (state == State::CHARGING) {
    return "CHARGING";

  } else if (state == State::ERROR) {
    return "ERROR";

  } else if (state == State::UNAVAILABLE) {
    return "UNAVAILABLE";

  } else {
    return "UNKNOWN";
  }
}
std::optional<std::string> Vehicle::get_proc_state(ProcState s) {
  if (s == kernel::driver::Vehicle::ProcState::IDLE) {
    return "IDLE";
  } else if (s == kernel::driver::Vehicle::ProcState::AWAITING_ORDER) {
    return "AWAITING_ORDER";
  } else if (s == kernel::driver::Vehicle::ProcState::PROCESSING_ORDER) {
    return "PROCESSING_ORDER";
  } else {
    return std::nullopt;
  }
}
std::string Vehicle::get_proc_state() {
  auto res = Vehicle::get_proc_state(this->proc_state);
  return res.value_or("");
}
void Vehicle::next_command() {
  if (!current_order) {
    if (orders.empty()) {
    } else {
      current_order = orders.front();
      // TODO  solver
      orders.pop_front();
      plan_route();
    }
  }
  if (!current_order) {
    get_next_ord();
    return;
  }
  if (current_order->state !=
      data::order::TransportOrder::State::BEING_PROCESSED) {
    current_order.reset();
    get_next_ord();
    return;
  }
  current_command = scheduler.lock()->new_command(shared_from_this());
  // run
  scheduler.lock()->add_command(current_command);
}
void Vehicle::receive_task(std::shared_ptr<data::order::TransportOrder> order) {
  CLOG(INFO, "driver") << name << " receive new order " << order->name;
  if (state == State::ERROR) {  // TODO
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, "driver") << order->name << " failed : " << name
                          << " state is ERROR";
  } else if (state == State::UNAVAILABLE) {
    // TODO
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, "driver") << order->name << " failed : " << name
                          << " state is UNAVAILABLE";
  } else if (state == State::IDLE) {
    // TODO
    state = State::EXECUTING;
    proc_state = ProcState::PROCESSING_ORDER;
    current_order.reset();
    orders.push_back(order);
    next_command();
  } else if (state == State::EXECUTING) {
    orders.push_back(order);
  } else if (state == State::CHARGING) {
    orders.push_back(order);
    if (engerg_level > energy_level_good) {
      state = State::EXECUTING;
      next_command();
    }
  } else {
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, "driver") << order->name << " failed : " << name
                          << " state is UNKNOWN";
    // TODO
  }
}
bool SimVehicle::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  if (dest->operation == data::order::DriverOrder::Destination::OpType::MOVE) {
    auto t =
        std::dynamic_pointer_cast<data::model::Point>(dest->destination.lock());
    position.x = t->position.x;
    position.y = t->position.y;
    current_point = t;
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::LOAD) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    current_point = t->link.lock();
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position.x = t->link.lock()->position.x;
    position.y = t->link.lock()->position.y;
    current_point = t->link.lock();
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::UNLOAD) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    current_point = t->link.lock();
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position.x = t->link.lock()->position.x;
    position.y = t->link.lock()->position.y;
    current_point = t->link.lock();
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::NOP) {
    return true;
  }
  return true;
}

bool SimVehicle::move(std::shared_ptr<data::order::Step> step) {
  if (!step->path) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    return true;
  }
  if (step->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
    auto end = step->path->destination_point.lock();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    int x_len = end->position.x - position.x;
    int y_len = end->position.y - position.y;
    for (int i = 0; i < 10; i++) {
      position.x += x_len / 10;
      position.y += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x = end->position.x;
    position.y = end->position.y;
    current_point = end;
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    return true;
  } else if (step->vehicle_orientation ==
             data::order::Step::Orientation::BACKWARD) {
    auto end = step->path->source_point.lock();
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    int x_len = end->position.x - position.x;
    int y_len = end->position.y - position.y;
    for (int i = 0; i < 10; i++) {
      position.x += x_len / 10;
      position.y += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x = end->position.x;
    position.y = end->position.y;
    current_point = end;
    CLOG(INFO, "driver") << name << " now at (" << position.x << " , "
                         << position.y << ")";
    return true;
  } else {
    return false;
  }
}

void Rabbit3::init() {
  set_mqtt_ops(name, this->broker_ip, this->broker_port);
  auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                "/" + serial_number + "/";
  mqtt_client->set_func(prefix + "state", [&](mqtt::const_message_ptr msg) {
    this->onstate(msg);
  });
  mqtt_client->set_func(
      prefix + "connection",
      [&](mqtt::const_message_ptr msg) { this->onconnect(msg); });
  start();
}

void Rabbit3::onstate(mqtt::const_message_ptr msg) {
  auto m = msg->to_string();
  nlohmann::json v;
  try {
    v = nlohmann::json::parse(m);
    auto ms = vda5050::state_validate(v);
    if (ms.empty()) {
      auto serial = v["serialNumber"].get<std::string>();
      if (serial != this->serial_number) {
        return;
      }
      auto ver = v["version"].get<std::string>();
      if (ver != this->version) {
        return;
      }
      auto h_id = v["headerId"].get<int>();
      if (h_id <= rece_header_id) {
        LOG(WARNING) << h_id << " " << rece_header_id;
        return;
      }
      rece_header_id = h_id;
      vdastate = vda5050::state::VDA5050State(v);
      //
      for (auto& x : resource.lock()->points) {
        if (x->name == v["lastNodeId"].get<std::string>()) {
          current_point = x;
          if (!init_pos) {
            this->position = current_point->position;
            init_pos = true;
            init_point = x;
            idle_time = std::chrono::system_clock::now();
            std::vector<std::shared_ptr<TCSResource>> res;
            res.push_back(x);
            resource.lock()->claim(res, shared_from_this());
          }
        }
      }
      if (v.contains("agvPosition")) {
        this->position.x = v["agvPosition"]["x"].get<float>();
        this->position.y = v["agvPosition"]["y"].get<float>();
        this->map_id = v["agvPosition"]["mapId"].get<std::string>();
        this->angle = v["agvPosition"]["theta"].get<float>();
        layout = position;
      }
      if (state == Vehicle::State::UNKNOWN) {
        if (veh_state == vda5050::VehicleMqttStatus::ONLINE) {
          state = State::IDLE;
        }
      }
      if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
        state = State::UNKNOWN;
      }
    } else {
      std::for_each(ms.begin(), ms.end(),
                    [](const std::string s) { CLOG(WARNING, "driver") << s; });
    }
  } catch (nlohmann::json::exception& ec) {
    CLOG(WARNING, "driver") << ec.what();
  }
}

void Rabbit3::onconnect(mqtt::const_message_ptr msg) {
  auto m = msg->to_string();
  nlohmann::json v;
  try {
    v = nlohmann::json::parse(m);
    auto ms = vda5050::connection_validate(v);
    if (ms.empty()) {
      if (v["connectionState"] == "ONLINE") {
        veh_state = vda5050::VehicleMqttStatus::ONLINE;
        CLOG(INFO, "driver") << serial_number + " ONLINE";
      } else if (v["connectionState"] == "OFFLINE") {
        veh_state = vda5050::VehicleMqttStatus::OFFLINE;
        state = Vehicle::State::UNKNOWN;
        CLOG(INFO, "driver") << serial_number + " OFFLINE";
      } else if (v["connectionState"] == "CONNECTIONBROKEN") {
        veh_state = vda5050::VehicleMqttStatus::CONNECTIONBROKEN;
        CLOG(WARNING, "driver") << serial_number + " CONNECTIONBROKEN";
        state = Vehicle::State::UNKNOWN;
      }
    } else {
      std::for_each(ms.begin(), ms.end(),
                    [](const std::string s) { CLOG(WARNING, "driver") << s; });
    }
  } catch (nlohmann::json::exception& ec) {
    CLOG(WARNING, "driver") << ec.what();
  }
}

bool Rabbit3::move(std::shared_ptr<data::order::Step> step) {
  CLOG(INFO, "driver") << "move along " << step->path->name;
  task_run = true;
  if (master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, "driver") << "master not online";
    task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, "driver") << "vehlicle not online";
    task_run = false;
    return false;
  }
  auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                "/" + serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(std::chrono::system_clock::now());
  ord.version = version;
  ord.manufacturer = manufacturer;
  ord.serial_number = serial_number;
  order_id++;
  ord.order_id = prefix + std::to_string(order_id);
  ord.order_update_id = 0;
  std::shared_ptr<data::model::Point> start_point;
  std::shared_ptr<data::model::Point> end_point;
  if (step->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
    start_point = step->path->source_point.lock();
    end_point = step->path->destination_point.lock();
  } else {
    start_point = step->path->destination_point.lock();
    end_point = step->path->source_point.lock();
  }
  auto start = vda5050::order::Node();
  start.node_id = start_point->name;
  start.released = false;
  start.sequence_id = 0;
  auto pos = vda5050::order::NodePosition();
  pos.x = start_point->position.x;
  pos.y = start_point->position.y;
  pos.map_id = map_id;
  pos.theta = angle;
  start.node_position = pos;
  ord.nodes.push_back(start);
  auto end = vda5050::order::Node();
  end.node_id = end_point->name;
  end.released = false;
  end.sequence_id = 1;
  auto pos2 = vda5050::order::NodePosition();
  pos2.x = end_point->position.x;
  pos2.y = end_point->position.y;
  pos2.map_id = map_id;
  pos2.theta = angle;
  end.node_position = pos2;
  ord.nodes.push_back(end);

  auto e = vda5050::order::Edge();
  e.edge_id = step->path->name;
  e.sequence_id = 0;
  e.released = false;
  e.start_node_id = start_point->name;
  e.end_node_id = end_point->name;
  e.max_speed = max_vel;
  ord.edges.push_back(e);

  //

  auto msg = std::make_shared<mqtt::message>();
  msg->set_qos(0);
  msg->set_topic(prefix + "order");
  msg->set_payload(ord.to_json().dump());
  mqtt_client->publish(msg)->wait();

  // wait move
  int n{0};
  while (task_run) {
    if (vdastate.order_id == prefix + std::to_string(order_id)) {
      auto p = get_time_from_str(vdastate.timestamp);
      auto dt = std::chrono::system_clock::now() - p.value();
      if (dt > std::chrono::seconds(5)) {
        task_run = false;
        return false;
      }
      // state
      if (veh_state == vda5050::VehicleMqttStatus::OFFLINE) {
        CLOG(ERROR, "driver") << "vehicle offline";
        task_run = false;
        return false;
      }
      // error
      if (!vdastate.errors.empty()) {
        for (auto& x : vdastate.errors) {
          if (x.error_level == vda5050::state::ErrorLevel::WARNING) {
            CLOG(WARNING, "driver") << x.error_type;
          } else {
            CLOG(ERROR, "driver") << x.error_type;
            task_run = false;
            return false;
          }
        }
      }
      //
      bool node_ok{true};
      for (auto& x : vdastate.nodestates) {
        if (x.released == false) {
          node_ok = false;
          break;
        }
      }

      bool edge_ok{true};
      for (auto& x : vdastate.edgestates) {
        if (x.released == false) {
          edge_ok = false;
          break;
        }
      }
      if (node_ok && edge_ok) {
        task_run = false;
        CLOG(INFO, "driver") << "move ok " << step->path->name;
        return true;
      }
    } else {
      CLOG(WARNING, "driver") << "order state has not been updated,want <"
                              << prefix + std::to_string(order_id)
                              << ">, but now is <" << vdastate.order_id << ">";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    n++;
    if (n > 2 * 60 * 10) {
      CLOG(ERROR, "driver") << "timeout";
      task_run = false;
      return false;
    }
  }
  CLOG(WARNING, "driver") << "task cancel";
  task_run = false;
  return false;
}
bool Rabbit3::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  CLOG(INFO, "driver") << "action along " << dest->destination.lock()->name;
  task_run = true;
  if (master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, "driver") << "master not online";
    task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, "driver") << "vehlicle not online";
    task_run = false;
    return false;
  }

  auto prefix = "/" + interface_name + "/" + version + "/" + manufacturer +
                "/" + serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(std::chrono::system_clock::now());
  ord.version = version;
  ord.manufacturer = manufacturer;
  ord.serial_number = serial_number;
  order_id++;
  ord.order_id = prefix + std::to_string(order_id);
  ord.order_update_id = 0;
  {
    if (dest->operation ==
        data::order::DriverOrder::Destination::OpType::MOVE) {
      home_state = Vehicle::HomeState::PARK;
      task_run = false;
      return true;
    } else if (dest->operation ==
               data::order::DriverOrder::Destination::OpType::NOP) {
      task_run = false;
      return true;
    } else {
      home_state = Vehicle::HomeState::HOME;
      auto t = std::dynamic_pointer_cast<data::model::Location>(
          dest->destination.lock());
      auto node = vda5050::order::Node();
      node.node_id = t->link.lock()->name;
      node.released = false;
      node.sequence_id = 0;
      node.node_position = vda5050::order::NodePosition();
      node.node_position.value().x = t->link.lock()->position.x;
      node.node_position.value().y = t->link.lock()->position.y;
      node.node_position.value().map_id = map_id;
      node.node_position.value().theta = angle + M_PI / 2;
      auto act = vda5050::order::Action();
      act.action_id = ord.order_id + "/action";
      if (dest->operation ==
          data::order::DriverOrder::Destination::OpType::LOAD) {
        act.action_type = vda5050::order::ActionType::LOAD;
      } else {
        act.action_type = vda5050::order::ActionType::UNLOAD;
      }
      act.blocking_type = vda5050::order::ActionBlockingType::HARD;
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      auto param_x = vda5050::order::ActionParam();
      param_x.key = "x";
      param_x.value = (float)t->position.x;
      act.action_parameters->push_back(param_x);
      auto param_y = vda5050::order::ActionParam();
      param_y.key = "y";
      param_y.value = (float)t->position.y;
      act.action_parameters->push_back(param_y);
      node.actions.push_back(act);
      ord.nodes.push_back(node);
    }
  }
  auto msg = std::make_shared<mqtt::message>();
  msg->set_qos(0);
  msg->set_topic(prefix + "order");
  msg->set_payload(ord.to_json().dump());
  mqtt_client->publish(msg)->wait();
  // wait
  int n{0};
  while (task_run) {
    if (vdastate.order_id == prefix + std::to_string(order_id)) {
      // state
      if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
        CLOG(ERROR, "driver") << "vehicle not online";
        task_run = false;
        return false;
      }
      // error
      if (!vdastate.errors.empty()) {
        for (auto& x : vdastate.errors) {
          if (x.error_level == vda5050::state::ErrorLevel::WARNING) {
            CLOG(WARNING, "driver") << x.error_type;
          } else {
            CLOG(ERROR, "driver") << x.error_type;
            task_run = false;
            return false;
          }
        }
      }
      //
      bool all_ok{true};
      for (auto& x : vdastate.actionstates) {
        auto sta = x.action_status;
        if (sta == vda5050::state::ActionStatus::FINISHED) {
          CLOG(INFO, "driver") << "action ok" << dest->destination.lock()->name;
        } else if (sta == vda5050::state::ActionStatus::FAILED) {
          CLOG(INFO, "driver")
              << "action failed" << dest->destination.lock()->name;
          task_run = false;
          return false;
        } else if (sta == vda5050::state::ActionStatus::WAITING) {
          all_ok = false;
        } else if (sta == vda5050::state::ActionStatus::RUNNING) {
          all_ok = false;
        } else if (sta == vda5050::state::ActionStatus::INITIALIZING) {
          all_ok = false;
        }
      }
      if (all_ok) {
        task_run = false;
        return true;
      }
    } else {
      CLOG(WARNING, "driver") << "order state has not been updated,want <"
                              << prefix + std::to_string(order_id)
                              << ">, but now is <" << vdastate.order_id << ">";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    n++;
    if (n > 2 * 60 * 10) {
      CLOG(ERROR, "driver") << "timeout";
      task_run = false;
      return false;
    }
  }
  task_run = false;
  CLOG(WARNING, "vehicel") << "task cancel";
  return false;
}
}  // namespace driver
}  // namespace kernel