
#include "../../../include/kernel/driver/vehicle.hpp"

#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/dispatch/dispatch.hpp"
namespace kernel {
namespace driver {

std::string vehicle_state_to_str(Vehicle::State state) {
  std::string res{"UNKNOWN"};
  if (state == Vehicle::State::UNAVAILABLE) {
    res = "UNAVAILABLE";
  } else if (state == Vehicle::State::ERROR) {
    res = "ERROR";

  } else if (state == Vehicle::State::IDEL) {
    res = "IDLE";

  } else if (state == Vehicle::State::EXECUTING) {
    res = "EXECUTING";

  } else if (state == Vehicle::State::CHARGING) {
    res = "CHARGING";
  }
  return res;
}

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
      CLOG(WARNING, driver_log)
          << name << " " << current_order->name << " action : "
          << " failed";
    } else {
      CLOG(INFO, driver_log)
          << name << " " << current_order->name << " action : "
          << " ok\n";
    }
    if (current_command) {
      current_command->vehicle_execute_cb(op_ret);
    }
  });
}
void Vehicle::execute_move(
    std::vector<std::shared_ptr<data::order::Step>> steps) {
  pool.async_run([=] {
    auto move_ret = move(steps);
    if (!move_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = std::chrono::system_clock::now();
      std::stringstream ss;
      ss << current_order->name << " : ";
      for (auto& x : steps) {
        ss << " move : " << x->name.c_str() << " & ";
      }
      CLOG(WARNING, driver_log) << name << " " << ss.str() << " failed";
    } else {
      std::stringstream ss;
      ss << current_order->name << " : ";
      for (auto& x : steps) {
        ss << " {" << x->name.c_str() << "} ";
      }
      CLOG(INFO, driver_log) << name << " " << ss.str() << " ok\n";
    }

    if (current_command) {
      current_command->vehicle_execute_cb(move_ret);
    }
  });
}
void Vehicle::execute_instatn_action(
    std::shared_ptr<vda5050::instantaction::Action> act) {
  current_action = act;
  instant_pool.async_run([&] {
    auto ret = instant_action(current_action);
    if (!ret) {
      CLOG(ERROR, driver_log)
          << name << " " << current_action->action_id << " failed";
    } else {
      CLOG(INFO, driver_log)
          << name << " " << current_action->action_id << " ok";
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
  instanttask_run = false;
  pool.stop();
  instant_pool.stop();
  // if (run_th.joinable()) {
  //   run_th.join();
  // }
  // current_command.reset();
  // current_action.reset();
}
Vehicle::~Vehicle() {
  close();
  CLOG(INFO, driver_log) << name << " close\n";
}
void Vehicle::reroute() { reroute_flag = true; }
void Vehicle::plan_route() {
  auto start_planner = last_point;
  std::shared_ptr<data::model::Point> end_planner;
  for (auto& op : current_order->driverorders) {
    auto dest = op->destination->destination.lock();
    auto start_check = resource.lock()->find(dest->name);
    auto destination = orderpool.lock()->res_to_destination(
        start_check.second, op->destination->operation);
    op->destination = destination;
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
      CLOG(WARNING, driver_log)
          << name << " " << current_order->name << " can not find obj";
    } else {
      auto path = planner.lock()->find_second_paths(start_planner, end_planner);
      if (path.empty()) {
        current_order->state = data::order::TransportOrder::State::UNROUTABLE;
        CLOG(WARNING, driver_log)
            << name << " " << current_order->name << " can not routable";
      } else {
        auto driverorder = orderpool.lock()->route_to_driverorder(
            resource.lock()->paths_to_route(path.front()), destination);
        driverorder->transport_order = current_order;
        driverorder->state = data::order::DriverOrder::State::PRISTINE;
        op = driverorder;
      }
    }
  }
}

void Vehicle::get_next_ord() {
  // 获取新订单
  for (;;) {
    if (orders.empty()) {
      // state = State::IDLE;
      break;
    } else {
      current_order = orders.front();
      orders.pop_front();
      if (current_order->state !=
          data::order::TransportOrder::State::BEING_PROCESSED) {
        if (process_chargeing) process_chargeing = false;
        current_order.reset();
        continue;
      }
      // route
      plan_route();
      if (current_order->state ==
          data::order::TransportOrder::State::UNROUTABLE) {
        if (process_chargeing) process_chargeing = false;
        current_order.reset();
        continue;
      } else {
        break;
      }
    }
  }
  if (current_order) {
    // state = State::EXECUTING;
    next_command();
  } else {
    process_state = proState::IDEL;
    if (process_chargeing) {
      CLOG_IF(state != State::CHARGING, INFO, driver_log)
          << name << " "
          << "state transform to : [" << vehicle_state_to_str(State::CHARGING)
          << "]";
      state = State::CHARGING;
    } else {
      CLOG_IF(state != State::IDEL, INFO, driver_log)
          << name << " "
          << "state transform to : [" << vehicle_state_to_str(State::IDEL)
          << "]\n";
      state = State::IDEL;
      idle_time = std::chrono::system_clock::now();
      CLOG(INFO, driver_log)
          << name << " "
          << "now is idle " << get_time_fmt(idle_time) << "\n";
    }
  }
}

void Vehicle::command_done() {
  process_state = proState::AWAITING_ORDER;
  if (reroute_flag) {
    plan_route();
    reroute_flag = false;
  }
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
    // 订单取消
    future_allocate_resources.clear();
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " withdrawl.";
    if (process_chargeing) {
      process_chargeing = false;
    }
    current_order.reset();
    get_next_ord();
    return;
  }
  if (current_order->state == data::order::TransportOrder::State::UNROUTABLE) {
    // 订单不可达
    future_allocate_resources.clear();
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " unrouteable.";
    if (process_chargeing) {
      process_chargeing = false;
    }
    current_order.reset();
    get_next_ord();
    return;
  }
  if (current_order->dead_time < std::chrono::system_clock::now()) {
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " timeout.";
    current_order->state = data::order::TransportOrder::State::FAILED;
  }
  if (current_order->state == data::order::TransportOrder::State::FAILED) {
    // 订单失败
    future_allocate_resources.clear();
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
    CLOG(INFO, driver_log) << name << " " << current_order->name
                           << " finished\n";
    current_order->end_time = std::chrono::system_clock::now();
    current_order.reset();
    current_command.reset();
    future_allocate_resources.clear();
    get_next_ord();
  } else {
    // 继续执行订单
    if (current_order->state ==
        data::order::TransportOrder::State::BEING_PROCESSED) {
      next_command();
      // state = State::EXECUTING;
    } else {
      // 订单取消了或失败了
      command_done();
    }
  }
}

std::string Vehicle::get_state() {
  if (state == State::IDEL) {
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
std::string Vehicle::get_process_state() {
  if (process_state == proState::AWAITING_ORDER) {
    return "AWAITING_ORDER";
  } else if (process_state == proState::PROCESSING_ORDER) {
    return "PROCESSING_ORDER";
  } else if (process_state == proState::IDEL) {
    return "IDLE";
  } else {
    return "UNKNOWN";
  }
}
void Vehicle::next_command() {
  if (!current_order) {
    if (orders.empty()) {
    } else {
      current_order = orders.front();
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
    if (process_chargeing) {
      process_chargeing = false;
    }
    get_next_ord();
    return;
  }
  current_command = scheduler.lock()->new_command(shared_from_this());
  // run
  scheduler.lock()->add_command(current_command);
  process_state = proState::PROCESSING_ORDER;
}
void Vehicle::receive_task(std::shared_ptr<data::order::TransportOrder> order) {
  CLOG(INFO, driver_log) << name << " receive new order " << order->name
                         << "\n";
  if (state == State::ERROR) {
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is ERROR";
  } else if (state == State::UNAVAILABLE) {
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is UNAVAILABLE";
  } else if (state == State::IDEL) {
    CLOG_IF(state != State::EXECUTING, INFO, driver_log)
        << name << " state transform to : ["
        << vehicle_state_to_str(State::EXECUTING) << "]\n";
    state = State::EXECUTING;
    process_state = proState::AWAITING_ORDER;
    if (current_order) {
      current_order->state = data::order::TransportOrder::State::WITHDRAWL;
    }
    current_order.reset();
    orders.push_back(order);
    next_command();
  } else if (state == State::EXECUTING) {
    if (current_order && current_order->anytime_drop) {
      current_order->state = data::order::TransportOrder::State::WITHDRAWL;
    }
    if (process_chargeing) {
      process_chargeing = false;
    }
    orders.push_back(order);
  } else if (state == State::CHARGING) {
    orders.push_back(order);
    if (engerg_level > energy_level_good) {
      //  stop 充电
      CLOG_IF(state != State::EXECUTING, INFO, driver_log)
          << name << " "
          << "state transform to : [" << vehicle_state_to_str(State::EXECUTING)
          << "]";
      state = State::EXECUTING;
      process_chargeing = false;
      next_command();
    }
  } else {
    order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is UNKNOWN";
  }
}
bool SimVehicle::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  if (dest->operation == data::order::DriverOrder::Destination::OpType::MOVE) {
    auto t =
        std::dynamic_pointer_cast<data::model::Point>(dest->destination.lock());
    position = t->position;
    last_point = t;
    return true;
  } else if (dest->operation ==
                 data::order::DriverOrder::Destination::OpType::LOAD ||
             dest->operation ==
                 data::order::DriverOrder::Destination::OpType::PICK) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    last_point = t->link.lock();
    CLOG(INFO, driver_log) << name << " now at (" << t->name << ")\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->link.lock()->position;
    last_point = t->link.lock();
    CLOG(INFO, driver_log) << name << " now at (" << t->name << ")\n";
    return true;
  } else if (dest->operation ==
                 data::order::DriverOrder::Destination::OpType::UNLOAD ||
             dest->operation == data::order::DriverOrder::Destination::
                                    Destination::OpType::DROP) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    current_point.reset();
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    last_point = t->link.lock();
    CLOG(INFO, driver_log) << name << " now at (" << position.x() << " , "
                           << position.y() << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->link.lock()->position;
    last_point = t->link.lock();
    current_point = last_point;
    CLOG(INFO, driver_log) << name << " now at (" << position.x() << " , "
                           << position.y() << ")";
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::NOP) {
    return true;
  }
  return true;
}
bool SimVehicle::instant_action(
    std::shared_ptr<data::model::Actions::Action> act) {
  CLOG(INFO, driver_log) << act->action_id << " ok";
  return true;
};
void SimVehicle::init() {
  state = State::IDEL;
  if (!last_point) {
    state = State::UNKNOWN;
    return;
  }
  current_point = last_point;
  this->position = current_point->position;
  if (integration_level == integrationLevel::TO_BE_IGNORED ||
      integration_level == integrationLevel::TO_BE_NOTICED) {
  } else {
    idle_time = std::chrono::system_clock::now();
    std::vector<std::shared_ptr<RSSResource>> ress;
    ress.push_back(current_point);
    resource.lock()->claim(ress, shared_from_this());
    resource.lock()->allocate(ress, shared_from_this());
  }
}
bool SimVehicle::move(std::vector<std::shared_ptr<data::order::Step>> steps) {
  if (steps.empty()) {
    return false;
  }
  auto step = steps.front();
  if (!step->path) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    return false;
  }
  if (step->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
    auto end = step->path->destination_point.lock();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    int x_len = end->position.x() - position.x();
    int y_len = end->position.y() - position.y();
    for (int i = 0; i < 10; i++) {
      position.x() += x_len / 10;
      position.y() += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x() = end->position.x();
    position.y() = end->position.y();
    last_point = end;
    current_point = last_point;
    CLOG(INFO, driver_log) << name << " now at (" << current_point->name
                           << ")\n";
    return true;
  } else if (step->vehicle_orientation ==
             data::order::Step::Orientation::BACKWARD) {
    auto end = step->path->source_point.lock();
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    int x_len = end->position.x() - position.x();
    int y_len = end->position.y() - position.y();
    for (int i = 0; i < 10; i++) {
      position.x() += x_len / 10;
      position.y() += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x() = end->position.x();
    position.y() = end->position.y();
    last_point = end;
    current_point = last_point;
    CLOG(INFO, driver_log) << name << " now at (" << current_point->name
                           << ")\n";
    return true;
  } else {
    return false;
  }
}

void Rabbit3::init() {
  mqtt_cli->set_mqtt_ops(name, this->broker_ip, this->broker_port);
  auto prefix = mqtt_cli->interface_name + "/" + mqtt_cli->version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  mqtt_cli->mqtt_client->set_func(
      prefix + "state",
      [&](mqtt::const_message_ptr msg) { this->onstate(msg); });
  mqtt_cli->mqtt_client->set_func(
      prefix + "connection",
      [&](mqtt::const_message_ptr msg) { this->onconnect(msg); });
  mqtt_cli->start();
}

void Rabbit3::onstate(mqtt::const_message_ptr msg) {
  // cpu_timer t;
  auto res = resource.lock();
  if (!res) return;
  auto m = msg->get_payload();
  jsoncons::json v;
  try {
    v = jsoncons::json::parse(m);
    CLOG(DEBUG, driver_log) << "state:" << v;
    // LOG(INFO) << "state:" << v;
    // cpu_timer t;
    auto ms = vda5050::state_validate(v);
    if (ms.empty()) {
      auto serial = v["serialNumber"].as_string();
      if (serial != this->mqtt_cli->serial_number) {
        return;
      }
      auto ver = v["version"].as_string();
      // if (ver != this->mqtt_cli->version) {
      //   return;
      // }
      auto h_id = v["headerId"].as_integer<int>();
      if (h_id <= rece_header_id) {
        // LOG(WARNING) << h_id << " " << rece_header_id;
        return;
      }
      rece_header_id = h_id;
      auto last_vdastate = vdastate;
      vdastate = vda5050::state::VDA5050State(v);
      CLOG_IF(vdastate.order_id != last_vdastate.order_id, INFO, driver_log)
          << name << " "
          << "order id has been updated, {\'" << last_vdastate.order_id
          << "\'} ==> {\'" << vdastate.order_id << "\'}\n";
      // LOG(INFO) << "------" << vdastate.battery_state.battery_charge;

      //

      if (v["lastNodeId"].as_string().empty()) {
        current_point.reset();
      } else {
        for (auto& x : res->points) {
          if (x->name == v["lastNodeId"].as_string()) {
            last_point = x;
            current_point = last_point;
            if (!init_pos) {
              this->position = current_point->position;
              init_pos = true;
              idle_time = std::chrono::system_clock::now();
              std::vector<std::shared_ptr<RSSResource>> ress;
              ress.push_back(x);
              res->claim(ress, shared_from_this());
              res->allocate(ress, shared_from_this());
            }
          }
        }
      }
      if (v.contains("agvPosition")) {
        this->position.x() = v["agvPosition"]["x"].as_double() * 1000;
        this->position.y() = v["agvPosition"]["y"].as_double() * 1000;
        this->map_id = v["agvPosition"]["mapId"].as_string();
        this->angle = v["agvPosition"]["theta"].as_double();
        layout = position;
      }
      auto last_state = state;
      if (state == Vehicle::State::UNKNOWN) {
        if (veh_state == vda5050::VehicleMqttStatus::ONLINE) {
          state = State::IDEL;
          idle_time = std::chrono::system_clock::now();
        }
      }
      if (!vdastate.errors.empty()) {
        state = State::ERROR;
      }

      if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
        state = State::UNKNOWN;
      }
      engerg_level = vdastate.battery_state.battery_charge;
      if (process_chargeing) {
        if (engerg_level > engrgy_level_full &&
            state == Vehicle::State::CHARGING) {
          state = Vehicle::State::IDEL;
          idle_time = std::chrono::system_clock::now();
          process_chargeing = false;
        }
      }
      if (last_state != state) {
        CLOG(INFO, driver_log)
            << name << " "
            << "state transform to : [" << vehicle_state_to_str(state) << "]\n";
      }

    } else {
      std::for_each(ms.begin(), ms.end(), [&](const std::string& s) {
        CLOG(WARNING, driver_log) << name << " " << s;
      });
    }
  } catch (jsoncons::json_exception& ec) {
    CLOG(WARNING, driver_log) << name << " error: " << ec.what();
  }
}

void Rabbit3::onconnect(mqtt::const_message_ptr msg) {
  auto m = msg->to_string();
  jsoncons::json v;
  try {
    v = jsoncons::json::parse(m);
    auto ms = vda5050::connection_validate(v);
    if (ms.empty()) {
      if (v["connectionState"] == "ONLINE") {
        veh_state = vda5050::VehicleMqttStatus::ONLINE;
        CLOG_N_TIMES(1, INFO, driver_log)
            << name << " " << mqtt_cli->serial_number + " ONLINE";
      } else if (v["connectionState"] == "OFFLINE") {
        veh_state = vda5050::VehicleMqttStatus::OFFLINE;
        state = Vehicle::State::UNKNOWN;
        CLOG(INFO, driver_log)
            << name << " " << mqtt_cli->serial_number + " OFFLINE";
      } else if (v["connectionState"] == "CONNECTIONBROKEN") {
        veh_state = vda5050::VehicleMqttStatus::CONNECTIONBROKEN;
        CLOG(WARNING, driver_log)
            << name << " " << mqtt_cli->serial_number + " CONNECTIONBROKEN";
        state = Vehicle::State::UNKNOWN;
      }
    } else {
      std::for_each(ms.begin(), ms.end(), [&](const std::string& s) {
        CLOG(WARNING, driver_log) << name << " " << s;
      });
    }
  } catch (jsoncons::json_exception& ec) {
    CLOG(WARNING, driver_log) << name << " " << ec.what();
  }
}

bool Rabbit3::move(std::vector<std::shared_ptr<data::order::Step>> steps) {
  // 发送多步，走完一步返回
  std::string name_{"["};

  {
    auto x = steps.front()->path;
    name_.append("{");
    name_.append(x->name);
    name_.append("}");
  }
  name_.append("]");
  CLOG(INFO, driver_log) << name << " move step: " << name_ << "\n";

  if (steps.empty()) {
    CLOG(WARNING, driver_log) << name << " move  null step";
    return true;
  }
  task_run = true;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " master not online";
    task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " vehlicle not online";
    task_run = false;
    return false;
  }
  auto prefix = mqtt_cli->interface_name + "/" + mqtt_cli->version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(std::chrono::system_clock::now());
  ord.version = mqtt_cli->version;
  ord.manufacturer = mqtt_cli->manufacturer;
  ord.serial_number = mqtt_cli->serial_number;
  order_id = get_uuid();
  ord.order_id = prefix + uuids::to_string(order_id);
  ord.order_update_id = 0;

  std::shared_ptr<data::model::Point> start_point;
  if (steps.front()->vehicle_orientation ==
      data::order::Step::Orientation::FORWARD) {
    start_point = steps.front()->path->source_point.lock();
  } else {
    start_point = steps.front()->path->destination_point.lock();
  }
  auto start = vda5050::order::Node();
  start.node_id = start_point->name;
  start.released = false;
  start.sequence_id = 0;
  auto pos = vda5050::order::NodePosition();
  pos.x = start_point->position.x() / 1000.0;
  pos.y = start_point->position.y() / 1000.0;
  pos.map_id = map_id;
  pos.theta = angle;
  start.node_position = pos;
  ord.nodes.push_back(start);

  std::set<std::string> wait_act;
  std::stringstream ss;
  ss << "move along [" << start_point->name << " --> ";
  std::string last_id = start_point->name;
  for (auto& x : steps) {
    std::shared_ptr<data::model::Point> end_point;
    if (x->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
      end_point = x->path->destination_point.lock();
    } else {
      end_point = x->path->source_point.lock();
    }
    if (x != *(steps.end() - 1)) {
      ss << end_point->name << " --> ";
    } else {
      ss << end_point->name << "]";
    }
    auto end = vda5050::order::Node();
    end.node_id = end_point->name;
    end.released = false;
    end.sequence_id = 1;
    auto pos2 = vda5050::order::NodePosition();
    pos2.x = end_point->position.x() / 1000.0;
    pos2.y = end_point->position.y() / 1000.0;
    pos2.map_id = map_id;
    pos2.theta = angle;
    end.node_position = pos2;
    ord.nodes.push_back(end);
    auto e = vda5050::order::Edge();
    e.edge_id = x->path->name;
    e.sequence_id = 0;
    e.released = false;
    e.start_node_id = last_id;
    e.end_node_id = end_point->name;
    e.max_speed = max_vel;
    last_id = end_point->name;
    ord.edges.push_back(e);
    {
      // peraction  服务器本地调用
      if (x == steps.front()) {
        // 只执行第一步的外围动作
        if (!x->path->per_acts.acts.empty()) {
          for (auto& op : x->path->per_acts.acts) {
            if (op.execution_trigger == "AFTER_ALLOCATION") {
              // TODO
              CLOG(INFO, driver_log)
                  << "do " << op.location_name << "[" << op.op_name << "]";
              if (op.completion_required) {
              }
            }
          }
        }
      }
      // action vda5050
      if (!x->path->acts.actions.empty()) {
        CLOG(INFO, driver_log) << name << " this node had "
                               << x->path->acts.actions.size() << " actions";
        for (auto& act : x->path->acts.actions) {
          if (!act.vaild) continue;
          CLOG(INFO, driver_log)
              << name << " " << act.action_id << " " << act.name;
          auto action = static_cast<vda5050::order::Action*>(&act);
          if (act.when == data::model::Actions::ActionWhen::ORDER_START) {
            (ord.nodes.end() - 2)->actions.push_back(*action);
          } else {
            (ord.nodes.end() - 1)->actions.push_back(*action);
          }
          if (x == steps.front()) {
            // 只等待第一步的动作
            if (act.blocking_type != vda5050::order::ActionBlockingType::NONE) {
              wait_act.insert(action->action_id);
            }
          }
        }
      }
    }
  }
  CLOG(INFO, driver_log) << name << " " << ss.str() << "\n";
  //

  auto msg =
      mqtt::make_message(prefix + "order", ord.to_json().as_string(), 0, false);
  mqtt_cli->mqtt_client->publish(msg)->wait();

  // wait move and actions
  int n{0};
  while (task_run) {
    if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " master not online";
      task_run = false;
      return false;
    }
    if (vdastate.order_id == prefix + uuids::to_string(order_id)) {
      auto p = get_time_from_str(vdastate.timestamp);
      if (p.has_value()) {
        auto dt = std::chrono::system_clock::now() - p.value();
        if (dt > std::chrono::seconds(10)) {
          CLOG(ERROR, driver_log)
              << name << " The communication interval is too long > 10s";
          task_run = false;
          return false;
        }
      } else {
        CLOG(WARNING, driver_log)
            << name << " dot not has timestamp, the status may be incorrect ";
      }
      // error
      if (!vdastate.errors.empty()) {
        for (auto& x : vdastate.errors) {
          if (x.error_level == vda5050::state::ErrorLevel::WARNING) {
            CLOG(WARNING, driver_log) << name << " " << x.error_type;
          } else {
            CLOG(ERROR, driver_log) << name << " " << x.error_type;
            task_run = false;
            return false;
          }
        }
      }
      //
      bool all_act_ok{false};
      if (wait_act.empty()) {
        all_act_ok = true;
      } else {
        for (auto& x : vdastate.actionstates) {
          if (wait_act.find(x.action_id) != wait_act.end()) {
            if (x.action_status == vda5050::state::ActionStatus::WAITING) {
            } else if (x.action_status ==
                       vda5050::state::ActionStatus::RUNNING) {
            } else if (x.action_status ==
                       vda5050::state::ActionStatus::INITIALIZING) {
            } else if (x.action_status ==
                       vda5050::state::ActionStatus::FAILED) {
              task_run = false;
              return false;
            } else if (x.action_status ==
                       vda5050::state::ActionStatus::FINISHED) {
              CLOG(INFO, driver_log)
                  << name << " "
                  << "wait action [" << x.action_id << "] ok ";
              wait_act.erase(wait_act.find(x.action_id));
            }
          }
        }
        if (wait_act.empty()) {
          CLOG(INFO, driver_log) << name << " all vadaction ok.";
          all_act_ok = true;
        }
      }
      auto it =
          std::find_if(vdastate.edgestates.begin(), vdastate.edgestates.end(),
                       [&](vda5050::state::EdgeState s) {
                         return s.edge_id == ord.edges.front().edge_id;
                       });
      if (it == vdastate.edgestates.end() && all_act_ok) {
        task_run = false;
        CLOG(INFO, driver_log) << name << " "
                               << "move along [" << name_ << "] ok\n";
        // per act 服务器本地调用 只调第一步
        // TODO
        auto x = steps.front();
        for (auto& op : x->path->per_acts.acts) {
          if (op.execution_trigger == "AFTER_MOVEMENT") {
            CLOG(INFO, driver_log)
                << "do " << op.location_name << "[" << op.op_name << "]";
            if (op.completion_required) {
            }
          }
        }
        return true;
      } else {
        // wait
        // LOG_ONCE(ERROR) << vdastate.edgestates.front().edge_id << " "
        //                 << ord.edges.front().edge_id;
      }
    } else {
      CLOG_N_TIMES(1, WARNING, driver_log)
          << name << " "
          << "order state has not been updated,wait_for {\'"
          << prefix + uuids::to_string(order_id) << "\'}, but now is {\'"
          << prefix << vdastate.order_id << "\'}\n";
      CLOG_EVERY_N(10, WARNING, driver_log)
          << name << " "
          << "order state has not been updated,wait_for {\'"
          << prefix + uuids::to_string(order_id) << "\'}, but now is {\'"
          << prefix << vdastate.order_id << "\'}\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    if (n > 2 * 60 * 100) {
      CLOG(ERROR, driver_log) << name << " "
                              << "timeout";
      task_run = false;
      return false;
    }
  }
  CLOG(WARNING, driver_log) << name << " "
                            << "task cancel";
  task_run = false;
  return false;
}
bool Rabbit3::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  CLOG(INFO, driver_log) << name << " "
                         << "action execute [" << dest->get_type() << "] at "
                         << dest->destination.lock()->name << "\n";
  task_run = true;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " "
                            << "master not online";
    task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " "
                            << "vehlicle not online";
    task_run = false;
    return false;
  }

  auto prefix = mqtt_cli->interface_name + "/" + mqtt_cli->version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(std::chrono::system_clock::now());
  ord.version = mqtt_cli->version;
  ord.manufacturer = mqtt_cli->manufacturer;
  ord.serial_number = mqtt_cli->serial_number;
  order_id = get_uuid();
  ord.order_id = prefix + uuids::to_string(order_id);
  ord.order_update_id = 0;
  {
    auto node = vda5050::order::Node();
    auto act = vda5050::order::Action();
    node.node_position = vda5050::order::NodePosition();
    node.node_position.value().map_id = map_id;
    node.node_position.value().theta = angle + M_PI / 2;
    auto t = resource.lock()->find(dest->destination.lock()->name);
    if (t.first == allocate::ResourceManager::ResType::Point) {
      auto t1 = std::dynamic_pointer_cast<data::model::Point>(
          dest->destination.lock());
      node.node_id = t1->name;
      node.node_position.value().x = t1->position.x() / 1000.0;
      node.node_position.value().y = t1->position.y() / 1000.0;
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      auto param_x = vda5050::order::ActionParam();
      param_x.key = "x";
      param_x.value = (float)t1->position.x() / 1000.0;
      act.action_parameters->push_back(param_x);
      auto param_y = vda5050::order::ActionParam();
      param_y.key = "y";
      param_y.value = (float)t1->position.y() / 1000.0;
      act.action_parameters->push_back(param_y);
    } else if (t.first == allocate::ResourceManager::ResType::Location) {
      auto t1 = std::dynamic_pointer_cast<data::model::Location>(
          dest->destination.lock());
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      node.node_id = t1->link.lock()->name;
      node.node_position.value().x = t1->link.lock()->position.x() / 1000.0;
      node.node_position.value().y = t1->link.lock()->position.y() / 1000.0;
      auto param_x = vda5050::order::ActionParam();
      param_x.key = "x";
      param_x.value = (float)t1->position.x() / 1000.0;
      act.action_parameters->push_back(param_x);
      auto param_y = vda5050::order::ActionParam();
      param_y.key = "y";
      param_y.value = (float)t1->position.y() / 1000.0;
      act.action_parameters->push_back(param_y);
      auto it = t1->type.lock()->allowed_ops.find(dest->get_type());
      auto params = it->second;
      for (auto& param : params) {
        auto p_ = vda5050::order::ActionParam();
        p_.key = param.first;
        p_.value = (std::string)param.second;
        act.action_parameters->push_back(p_);
      }
    } else {
      CLOG(ERROR, driver_log) << name << " "
                              << "dest type is err";
      return false;
    }

    node.released = false;
    node.sequence_id = 0;
    act.action_id = ord.order_id + "/action";
    act.action_type = dest->operation;
    act.blocking_type = vda5050::order::ActionBlockingType::HARD;
    node.actions.push_back(act);
    ord.nodes.push_back(node);
  }
  auto msg = mqtt::make_message(prefix + "order", ord.to_json().as_string());
  mqtt_cli->mqtt_client->publish(msg)->wait();
  // wait
  int n{0};
  while (task_run) {
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " "
                              << "vehlicle not online";
      task_run = false;
      return false;
    }
    if (vdastate.order_id == prefix + uuids::to_string(order_id)) {
      // error
      if (!vdastate.errors.empty()) {
        for (auto& x : vdastate.errors) {
          if (x.error_level == vda5050::state::ErrorLevel::WARNING) {
            CLOG(WARNING, driver_log) << name << " " << x.error_type;
          } else {
            CLOG(ERROR, driver_log) << name << " " << x.error_type;
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
          CLOG(INFO, driver_log) << name << " "
                                 << "action [" << dest->get_type() << "] ok\n";
        } else if (sta == vda5050::state::ActionStatus::FAILED) {
          CLOG(INFO, driver_log)
              << name << " "
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
        CLOG(INFO, driver_log) << name << " "
                               << "all actions ok\n";
        task_run = false;
        return true;
      }
    } else {
      CLOG(WARNING, driver_log)
          << name << " "
          << "order state has not been updated,needed {\'"
          << prefix + uuids::to_string(order_id) << "\'}, but now is {\'"
          << vdastate.order_id << "}\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    if (n > 2 * 60 * 100) {
      CLOG(ERROR, driver_log) << name << " "
                              << "timeout";
      task_run = false;
      return false;
    }
  }
  task_run = false;
  CLOG(WARNING, driver_log) << name << " "
                            << "task cancel";
  return false;
}
bool Rabbit3::instant_action(
    std::shared_ptr<data::model::Actions::Action> act) {
  CLOG(INFO, driver_log) << "instantaction " << act->action_id;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " "
                            << "master not online";
    instanttask_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " "
                            << "vehlicle not online";
    instanttask_run = false;
    return false;
  }
  auto prefix = mqtt_cli->interface_name + "/" + mqtt_cli->version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto insact = std::make_shared<vda5050::instantaction::InstantAction>();
  insact->serial_number = mqtt_cli->serial_number;
  insact->header_id = send_header_id++;
  insact->manufacturer = mqtt_cli->manufacturer;
  insact->timestamp = get_time_fmt(std::chrono::system_clock::now());
  insact->version = mqtt_cli->version;
  auto action = static_cast<vda5050::instantaction::Action*>(act.get());
  insact->actions.push_back(*action);
  auto msg = mqtt::make_message(prefix + "instantAction",
                                insact->to_json().as_string());
  mqtt_cli->mqtt_client->publish(msg)->wait();
  auto id = act->action_id;
  int n{0};
  instanttask_run = true;
  bool ok{false};
  while (instanttask_run) {
    if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " "
                              << "master not online";
      instanttask_run = false;
      return false;
    }
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " "
                              << "vehlicle not online";
      instanttask_run = false;
      return false;
    }
    for (auto& x : vdastate.actionstates) {
      if (x.action_id == id) {
        if (x.action_status == vda5050::state::ActionStatus::WAITING) {
        } else if (x.action_status == vda5050::state::ActionStatus::RUNNING) {
        } else if (x.action_status ==
                   vda5050::state::ActionStatus::INITIALIZING) {
        } else if (x.action_status == vda5050::state::ActionStatus::FAILED) {
          instanttask_run = false;
          break;
        } else if (x.action_status == vda5050::state::ActionStatus::FINISHED) {
          ok = true;
          instanttask_run = false;
          break;
        } else {
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    if (n > 2 * 60 * 100) {
      CLOG(ERROR, driver_log) << name << " "
                              << "timeout";
      break;
    }
  }
  instanttask_run = false;
  return ok;
}

Rabbit3::~Rabbit3() {
  if (mqtt_cli->mqtt_client && mqtt_cli->mqtt_client->is_connected()) {
    mqtt_cli->mqtt_client->disable_callbacks();
    mqtt_cli->mqtt_client->unsubscribe("#");
    mqtt_cli->mqtt_client->disconnect()->wait();
  }
}

}  // namespace driver
}  // namespace kernel