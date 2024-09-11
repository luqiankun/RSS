
#include "../../../include/kernel/driver/vehicle.hpp"

#include "../../../include/component/util/tools.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/component/vda5050/vda5050order.hpp"
#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
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
      current_order->end_time = get_now_utc_time();
      CLOG(WARNING, driver_log)
          << name << " " << current_order->name << " action : " << " failed";
    } else {
      CLOG(INFO, driver_log)
          << name << " " << current_order->name << " action : " << " ok\n";
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
      current_order->end_time = get_now_utc_time();
      std::stringstream ss;
      ss << current_order->name << " : move {" << steps.front()->name << "}";
      CLOG(WARNING, driver_log) << name << " " << ss.str() << " failed\n";
    } else {
      // std::stringstream ss;
      // ss << current_order->name << " : move {" << steps.front()->name << "}";
      // CLOG(INFO, driver_log) << name << " " << ss.str() << " ok\n";
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
  auto res = resource.lock();
  auto ordpoll = orderpool.lock();
  auto route_planner = planner.lock();

  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return;
  }
  if (!ordpoll) {
    CLOG(ERROR, driver_log) << name << " orderpoll is null\n";
    return;
  }
  if (!route_planner) {
    CLOG(ERROR, driver_log) << name << " planner is null\n";
    return;
  }
  bool first_driverorder{true};
  std::shared_ptr<data::model::Point> start_planner;
  std::shared_ptr<data::model::Point> end_planner;
  for (auto& op : current_order->driverorders) {
    if (first_driverorder) {
      first_driverorder = false;
      start_planner = last_point;
    } else {
      start_planner = end_planner;
    }
    auto dest = op->destination->destination.lock();
    auto start_check = res->find(dest->name);
    auto destination = ordpoll->res_to_destination(start_check.second,
                                                   op->destination->operation);
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
    if (!end_planner || !start_planner) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
      CLOG(WARNING, driver_log)
          << name << " " << current_order->name << " can not find obj "
          << destination->destination.lock()->name
          << " or can not locate current_pos\n";
    } else {
      auto path = route_planner->find_second_paths(start_planner, end_planner);
      if (path.empty()) {
        current_order->state = data::order::TransportOrder::State::UNROUTABLE;
        CLOG(WARNING, driver_log)
            << name << " " << current_order->name << " can not routable\n";
      } else {
        auto driverorder = ordpoll->route_to_driverorder(
            res->paths_to_route(path.front()), destination);
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
          data::order::TransportOrder::State::DISPATCHABLE) {
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
    // current_order->state =
    // data::order::TransportOrder::State::BEING_PROCESSED;
    next_command();
  } else {
    process_state = proState::IDEL;
    if (process_chargeing) {
      CLOG_IF(state != State::CHARGING, INFO, driver_log)
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::CHARGING) << "]\n";
      state = State::CHARGING;
    } else {
      CLOG_IF(state != State::IDEL, INFO, driver_log)
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::IDEL) << "]\n";
      state = State::IDEL;
      idle_time = get_now_utc_time();
      CLOG(INFO, driver_log)
          << name << " " << "now is idle " << get_time_fmt(idle_time) << "\n";
    }
  }
}

void Vehicle::command_done() {
  process_state = proState::AWAITING_ORDER;
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return;
  }
  if (reroute_flag) {
    plan_route();
    reroute_flag = false;
  }
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
    // 订单取消
    future_allocate_resources.clear();
    init_pos = false;
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " withdrawl.\n";
    if (process_chargeing) {
      process_chargeing = false;
    }
    now_order_state = Vehicle::nowOrder::END;
    current_order.reset();
    get_next_ord();
    return;
  }
  if (current_order->state == data::order::TransportOrder::State::UNROUTABLE) {
    // 订单不可达
    future_allocate_resources.clear();
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " unrouteable.";
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto& a : this->allocated_resources) {
      for (auto& x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    res->free(temp, shared_from_this());
    if (process_chargeing) {
      process_chargeing = false;
    }
    now_order_state = Vehicle::nowOrder::END;
    current_order.reset();
    get_next_ord();
    return;
  }
  if (current_order->dead_time < get_now_utc_time()) {
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " timeout.";
    current_order->state = data::order::TransportOrder::State::FAILED;
    now_order_state = Vehicle::nowOrder::END;
  }
  if (current_order->state == data::order::TransportOrder::State::FAILED) {
    // 订单失败
    future_allocate_resources.clear();
    now_order_state = Vehicle::nowOrder::END;
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
    current_order->end_time = get_now_utc_time();
    now_order_state = Vehicle::nowOrder::END;
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
  auto scheduler_ = scheduler.lock();
  if (!scheduler_) {
    CLOG(ERROR, driver_log) << name << " scheduler is null\n";
    return;
  }
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
  if (current_order->state ==
      data::order::TransportOrder::State::DISPATCHABLE) {
    current_order->state = data::order::TransportOrder::State::BEING_PROCESSED;
    CLOG(INFO, order_log) << current_order->name
                          << " status: {begin_processed}\n";
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
  current_command = scheduler_->new_command(shared_from_this());
  // run
  scheduler_->add_command(current_command);
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
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::EXECUTING) << "]";
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
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return;
  }
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
    idle_time = get_now_utc_time();
    std::vector<std::shared_ptr<RSSResource>> ress;
    ress.push_back(current_point);
    res->claim(ress, shared_from_this());
    res->allocate(ress, shared_from_this());
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
  int ver = static_cast<int>(std::stod(mqtt_cli->version));
  auto vda_version = "v" + std::to_string(ver);
  mqtt_cli->set_mqtt_ops(name, this->broker_ip, this->broker_port);
  auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
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
    auto opt = jsoncons::json_options{}.precision(15).float_format(
        jsoncons::float_chars_format::general);
    v = jsoncons::json::parse(m, opt);
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
        // 重启了？
        LOG(WARNING) << h_id << " " << rece_header_id;
        if (current_order) {
          current_order->state = data::order::TransportOrder::State::FAILED;
        }
        state = Vehicle::State::UNKNOWN;
        std::vector<std::shared_ptr<RSSResource>> temp;
        for (auto& a : allocated_resources) {
          for (auto& x : a) {
            temp.push_back(x);
          }
        }
        res->free(temp, shared_from_this());
        init_pos = false;
        CLOG(WARNING, driver_log) << name << " 重启了???\n";
      }
      rece_header_id = h_id;
      auto last_vdastate = vdastate;
      vdastate = vda5050::state::VDA5050State(v);
      CLOG_IF(vdastate.order_id != last_vdastate.order_id ||
                  vdastate.order_update_id != last_vdastate.order_update_id,
              INFO, driver_log)
          << name << " " << "order_id has been updated, {\'"
          << last_vdastate.order_id << " -> update_order_id_"
          << last_vdastate.order_update_id << "\'} ==> {\'" << vdastate.order_id
          << " -> update_order_id_" << vdastate.order_update_id << "\'}\n";
      // LOG(INFO) << "------" << vdastate.battery_state.battery_charge;

      //

      if (vdastate.last_node_id.empty()) {
        current_point.reset();
      } else {
        for (auto& x : res->points) {
          // LOG(INFO) << x->name;
          if (x->name == vdastate.last_node_id) {
            last_point = x;
            current_point = last_point;
            if (!init_pos) {
              this->position = current_point->position;
              idle_time = get_now_utc_time();
              std::vector<std::shared_ptr<RSSResource>> ress;
              ress.push_back(x);
              res->claim(ress, shared_from_this());
              if (!res->allocate(ress, shared_from_this())) {
                CLOG(ERROR, driver_log)
                    << name << " init failed:{ allocate init_point failed }\n";
                init_pos = false;
              } else {
                init_pos = true;
              }
            } else {
              if (!vdastate.agv_position.has_value()) {
                this->position = current_point->position;
              }
              if (state == State::IDEL &&
                  last_vdastate.last_node_id != vdastate.last_node_id) {
                LOG(INFO) << last_vdastate.last_node_id << " "
                          << vdastate.last_node_id;
                std::vector<std::shared_ptr<RSSResource>> temp;
                for (auto& a : allocated_resources) {
                  for (auto& x : a) {
                    temp.push_back(x);
                  }
                }
                res->free(temp, shared_from_this());
                std::vector<std::shared_ptr<RSSResource>> ress;
                ress.push_back(x);
                res->claim(ress, shared_from_this());
                if (!res->allocate(ress, shared_from_this())) {
                  CLOG(ERROR, driver_log)
                      << name
                      << " reinit failed:{ allocate init_point failed }\n";
                  init_pos = false;
                }
              }
            }
          }
        }
      }
      this->position.x() = vdastate.agv_position->x * 1000;
      this->position.y() = vdastate.agv_position->y * 1000;
      this->map_id = vdastate.agv_position->map_id;
      this->angle = vdastate.agv_position->theta;
      layout = position;
      auto last_state = state;
      if (state == Vehicle::State::UNKNOWN) {
        if (veh_state == vda5050::VehicleMqttStatus::ONLINE) {
          state = State::IDEL;
          idle_time = get_now_utc_time();
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
        if (engerg_level > energy_level_good &&
            state == Vehicle::State::CHARGING) {
          state = Vehicle::State::IDEL;
          idle_time = get_now_utc_time();
          process_chargeing = false;
        }
      }
      if (last_state != state) {
        CLOG(INFO, driver_log) << name << " " << "state transform to : ["
                               << vehicle_state_to_str(state) << "]\n";
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
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log)
        << name << " " << mqtt_cli->serial_number << " resource is not exist\n";
    return;
  }
  auto m = msg->to_string();
  jsoncons::json v;
  try {
    v = jsoncons::json::parse(m);
    auto ms = vda5050::connection_validate(v);
    if (ms.empty()) {
      if (v["connectionState"] == "ONLINE") {
        CLOG_IF(veh_state != vda5050::VehicleMqttStatus::ONLINE, INFO,
                driver_log)
            << name << " " << mqtt_cli->serial_number + " ONLINE\n";
        if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
          veh_state = vda5050::VehicleMqttStatus::ONLINE;
          rece_header_id = -1;
          init_pos = false;
        }
      } else if (v["connectionState"] == "OFFLINE") {
        CLOG_IF(veh_state != vda5050::VehicleMqttStatus::OFFLINE, INFO,
                driver_log)
            << name << " " << mqtt_cli->serial_number + " OFFLINE\n";
        if (veh_state != vda5050::VehicleMqttStatus::OFFLINE) {
          veh_state = vda5050::VehicleMqttStatus::OFFLINE;
          state = Vehicle::State::UNKNOWN;
          std::vector<std::shared_ptr<RSSResource>> temp;
          for (auto& a : allocated_resources) {
            for (auto& x : a) {
              temp.push_back(x);
            }
          }
          res->free(temp, shared_from_this());
        }
        rece_header_id = -1;
        init_pos = false;
      } else if (v["connectionState"] == "CONNECTIONBROKEN") {
        if (veh_state != vda5050::VehicleMqttStatus::CONNECTIONBROKEN) {
          veh_state = vda5050::VehicleMqttStatus::CONNECTIONBROKEN;
          state = Vehicle::State::UNKNOWN;
          std::vector<std::shared_ptr<RSSResource>> temp;
          for (auto& a : allocated_resources) {
            for (auto& x : a) {
              temp.push_back(x);
            }
          }
          res->free(temp, shared_from_this());
        }
        rece_header_id = -1;
        init_pos = false;
        CLOG(WARNING, driver_log)
            << name << " " << mqtt_cli->serial_number + " CONNECTIONBROKEN\n";
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
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is not exist\n";
    return false;
  }
  bool last_steps{false};
  auto& driver_order =
      current_order->driverorders[current_order->current_driver_index];

  if (driver_order->route->steps.empty()) {
    last_steps = true;
  }
  // 发送多步，走完一步返回
  std::string name_{""};

  {
    auto x = steps.front()->path;
    name_.append("{");
    name_.append(x->name);
    name_.append("}");
  }
  // CLOG(INFO, driver_log) << name << " will move step: " << name_ << "\n";

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
  auto Pre = current_order->driverorders[current_order->current_driver_index]
                 ->destination->get_type();
  int ver = static_cast<int>(std::stod(mqtt_cli->version));
  auto vda_version = "v" + std::to_string(ver);
  auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(get_now_utc_time());
  ord.version = mqtt_cli->version;
  ord.manufacturer = mqtt_cli->manufacturer;
  ord.serial_number = mqtt_cli->serial_number;
  if (now_order_state == nowOrder::BEGIN) {
    ord.order_id = Pre + "-" + uuids::to_string(order_id);
    update_vda_order_id++;
    ord.order_update_id = update_vda_order_id;
    if (last_step_count > 0) {
      seq_id = (seq_id - 1) - (last_step_count - 1) * 2;
    } else {
      seq_id = (seq_id - 1);
    }
  } else {
    now_order_state = nowOrder::BEGIN;
    seq_id = 0;
    order_id = get_uuid();
    ord.order_id = Pre + "-" + uuids::to_string(order_id);
    update_vda_order_id = 0;
    ord.order_update_id = update_vda_order_id;
  }
  last_step_count = 0;
  std::set<std::string> wait_act_ord_start;
  std::set<std::string> wait_act_ord_end;
  std::shared_ptr<data::model::Point> start_point;
  if (steps.front()->vehicle_orientation ==
      data::order::Step::Orientation::FORWARD) {
    start_point = steps.front()->path->source_point.lock();
  } else {
    start_point = steps.front()->path->destination_point.lock();
  }
  auto start = vda5050::order::Node();
  start.node_id = start_point->name;
  start.released = true;
  start.sequence_id = seq_id++;
  auto pos = vda5050::order::NodePosition();
  pos.x = start_point->position.x() / 1000.0;
  pos.y = start_point->position.y() / 1000.0;
  pos.map_id = map_id;
  pos.theta = start_point->vehicle_orientation;
  pos.allowed_deviation_xy = deviation_xy;
  pos.allowed_deviation_theta = deviation_theta;
  start.node_position = pos;
  {
    if (steps.front()->route_index == 0) {
      // 起点
      // vda 遍历当前动作
      for (auto& x : steps.front()->path->acts.actions) {
        auto action = static_cast<vda5050::order::Action*>(&x);
        if (action->when == vda5050::order::ActionWhen::ORDER_START) {
          if (action->blocking_type !=
              vda5050::order::ActionBlockingType::NONE) {
            wait_act_ord_start.insert(action->action_id);
          }
          start.actions.push_back(*action);
        }
      }
      for (auto& x : driver_order->route->steps) {
        for (auto& act : x->path->acts.actions) {
          auto action = static_cast<vda5050::order::Action*>(&act);
          if (action->when == vda5050::order::ActionWhen::ORDER_START) {
            if (action->blocking_type !=
                vda5050::order::ActionBlockingType::NONE) {
              wait_act_ord_start.insert(action->action_id);
            }
            start.actions.push_back(*action);
          }
        }
      }
    }
  }
  ord.nodes.push_back(start);

  std::stringstream ss;
  ss << "begin move along [" << start_point->name << " --> ";
  std::string last_id = start_point->name;
  int forward_two{0};  // 只有前两步是Base,后面都是Horizon
  for (auto& x : steps) {
    last_step_count++;
    std::shared_ptr<data::model::Point> end_point;
    if (x->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
      end_point = x->path->destination_point.lock();
    } else {
      end_point = x->path->source_point.lock();
    }
    // LOG(INFO) << "□→□" << end_point->name;
    if (x != *(steps.end() - 1)) {
      ss << end_point->name << " --> ";
    } else {
      ss << end_point->name << "]";
    }
    auto end = vda5050::order::Node();
    end.node_id = end_point->name;
    if (forward_two < 2) {
      end.released = true;
    } else {
      end.released = false;
    }
    auto pos2 = vda5050::order::NodePosition();
    pos2.x = end_point->position.x() / 1000.0;
    pos2.y = end_point->position.y() / 1000.0;
    if (last_steps && x == steps.back()) {
      // 订单的动作，不是路径的动作
      pos2.allowed_deviation_xy = (dest_deviation_xy);
      pos2.allowed_deviation_theta = (dest_deviation_theta);
      auto act = vda5050::order::Action();
      auto dest = driver_order->destination;
      auto t = res->find(dest->destination.lock()->name);
      act.action_id = ord.order_id + "-action";
      act.action_type = dest->operation;
      act.blocking_type = vda5050::order::ActionBlockingType::HARD;
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      if (t.first == allocate::ResourceManager::ResType::Location) {
        auto t1 = std::dynamic_pointer_cast<data::model::Location>(
            dest->destination.lock());
        std::string op_type = dest->get_type();
        std::transform(op_type.begin(), op_type.end(), op_type.begin(),
                       ::tolower);
        auto it = t1->type.lock()->allowed_ops.find(op_type);
        auto params = it->second;
        for (auto& param : params) {
          auto p_ = vda5050::order::ActionParam();
          p_.key = param.first;
          p_.value = (std::string)param.second;
          act.action_parameters->push_back(p_);
        }
        auto param_x = vda5050::order::ActionParam();
        param_x.key = "location_x";
        param_x.value = (double)t1->position.x() / 1000.0;
        act.action_parameters->push_back(param_x);
        auto param_y = vda5050::order::ActionParam();
        param_y.key = "location_y";
        param_y.value = (double)t1->position.y() / 1000.0;
        act.action_parameters->push_back(param_y);
      } else if (t.first == allocate::ResourceManager::ResType::Point) {
        auto t1 = std::dynamic_pointer_cast<data::model::Point>(
            dest->destination.lock());
        auto param_x = vda5050::order::ActionParam();
        param_x.key = "point_x";
        param_x.value = (double)t1->position.x() / 1000.0;
        act.action_parameters->push_back(param_x);
        auto param_y = vda5050::order::ActionParam();
        param_y.key = "point_y";
        param_y.value = (double)t1->position.y() / 1000.0;
        act.action_parameters->push_back(param_y);
      }
      // TODO TEST
      end.actions.push_back(act);
    } else {
      pos2.allowed_deviation_xy = (deviation_xy);
      pos2.allowed_deviation_theta = (deviation_theta);
    }
    pos2.map_id = map_id;
    pos2.theta = end_point->vehicle_orientation;
    end.node_position = pos2;
    auto e = vda5050::order::Edge();
    e.edge_id = x->path->name;
    if (forward_two < 2) {
      e.released = true;
    } else {
      e.released = false;
    }
    e.start_node_id = last_id;
    e.end_node_id = end_point->name;
    Eigen::Vector3i st_to_ed = x->path->destination_point.lock()->position -
                               x->path->source_point.lock()->position;
    double angle_radians = std::atan2(st_to_ed.y(), st_to_ed.x());
    if (x->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
      e.max_speed = std::make_optional(max_vel * 1.0 / 1000);
      e.direction = std::make_optional("forward");
      if (x->path->orientation_forward.has_value()) {
        e.orientation =
            std::make_optional(x->path->orientation_forward.value());
      } else {
        e.orientation = std::make_optional(angle_radians);
      }
    } else {
      // angle_radians += M_PI;
      // if (angle_radians > M_PI) {
      //   angle_radians -= 2 * M_PI;
      // }
      e.max_speed = std::make_optional(max_reverse_vel * 1.0 / 1000);
      e.direction = std::make_optional("backward");
      if (x->path->orientation_forward.has_value()) {
        e.orientation =
            std::make_optional(x->path->orientation_reverse.value());
      } else {
        e.orientation = std::make_optional(angle_radians);
      }
    }
    if (x->path->layout.connect_type ==
        data::model::Path::ConnectType::BEZIER) {
      auto trajectory = vda5050::order::Trajectory();
      trajectory.degree = 3;
      trajectory.knot_vector.assign({0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0});
      auto st_ctrl = vda5050::order::ControlPoint();
      st_ctrl.x = x->path->source_point.lock()->position.x() / 1000.0;
      st_ctrl.y = x->path->source_point.lock()->position.y() / 1000.0;
      st_ctrl.weight = 1;
      trajectory.control_points.push_back(st_ctrl);
      assert(x->path->layout.control_points.size() == 2);
      for (auto& cp : x->path->layout.control_points) {
        auto p = vda5050::order::ControlPoint();
        p.x = cp.x() * 50 / 1000.0;
        p.y = -cp.y() * 50 / 1000.0;
        p.weight = 1;
        trajectory.control_points.push_back(p);
      }
      auto ed_ctrl = vda5050::order::ControlPoint();
      ed_ctrl.x = x->path->destination_point.lock()->position.x() / 1000.0;
      ed_ctrl.y = x->path->destination_point.lock()->position.y() / 1000.0;
      ed_ctrl.weight = 1;
      trajectory.control_points.push_back(ed_ctrl);
      if (x->path->orientation_forward.has_value() ||
          x->path->orientation_reverse.has_value()) {
        for (auto& traj : trajectory.control_points) {
          traj.orientation = 3.14159;
        }
      }
      if (x->vehicle_orientation == data::order::Step::Orientation::BACKWARD) {
        std::reverse(trajectory.control_points.begin(),
                     trajectory.control_points.end());
      }
      e.trajectory = std::make_optional(trajectory);
    } else if (x->path->layout.connect_type ==
               data::model::Path::ConnectType::BEZIER_3) {
      auto trajectory = vda5050::order::Trajectory();
      trajectory.degree = 6;
      trajectory.knot_vector.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      auto st_ctrl = vda5050::order::ControlPoint();
      st_ctrl.x = x->path->source_point.lock()->position.x() / 1000.0;
      st_ctrl.y = x->path->source_point.lock()->position.y() / 1000.0;
      st_ctrl.weight = 1;
      trajectory.control_points.push_back(st_ctrl);
      assert(x->path->layout.control_points.size() == 5);
      for (auto& cp : x->path->layout.control_points) {
        auto p = vda5050::order::ControlPoint();
        p.x = cp.x() * 50 / 1000.0;
        p.y = -cp.y() * 50 / 1000.0;
        p.weight = 1;
        trajectory.control_points.push_back(p);
      }
      auto ed_ctrl = vda5050::order::ControlPoint();
      ed_ctrl.x = x->path->destination_point.lock()->position.x() / 1000.0;
      ed_ctrl.y = x->path->destination_point.lock()->position.y() / 1000.0;
      ed_ctrl.weight = 1;
      trajectory.control_points.push_back(ed_ctrl);
      if (x->path->orientation_forward.has_value() ||
          x->path->orientation_reverse.has_value()) {
        for (auto& traj : trajectory.control_points) {
          traj.orientation = 3.14159;
        }
      }
      if (x->vehicle_orientation == data::order::Step::Orientation::BACKWARD) {
        std::reverse(trajectory.control_points.begin(),
                     trajectory.control_points.end());
      }
      e.trajectory = std::make_optional(trajectory);
    }

    last_id = end_point->name;
    e.sequence_id = seq_id++;
    end.sequence_id = seq_id++;
    {
      // peraction  服务器本地调用

      if (!x->path->per_acts.acts.empty()) {
        for (auto& op : x->path->per_acts.acts) {
          if (op.execution_trigger == "AFTER_ALLOCATION") {
            // TODO
            CLOG(INFO, driver_log)
                << "do " << op.location_name << "[" << op.op_name << "]";
            if (op.completion_required) {
              // wait_act_ord_start.insert(op.op_name);
            }
          } else {
            CLOG(INFO, driver_log)
                << "do " << op.location_name << "[" << op.op_name << "]";
            if (op.completion_required) {
              // wait_act_ord_end.insert(op.op_name);
            }
          }
        }
      }
    }
    ord.nodes.push_back(end);
    ord.edges.push_back(e);
    forward_two++;
  }
  {
    if (steps.front()->route_index == driver_order->route->step_number - 1) {
      // 终点
      // 遍历当前动作
      for (auto& x : steps.front()->path->acts.actions) {
        auto action = static_cast<vda5050::order::Action*>(&x);
        if (action->when == vda5050::order::ActionWhen::ORDER_END) {
          if (action->blocking_type !=
              vda5050::order::ActionBlockingType::NONE) {
            wait_act_ord_end.insert(action->action_id);
          }
          ord.nodes.back().actions.push_back(*action);
        }
      }
      for (auto& x : driver_order->route->steps) {
        for (auto& act : x->path->acts.actions) {
          auto action = static_cast<vda5050::order::Action*>(&act);
          if (action->when == vda5050::order::ActionWhen::ORDER_END) {
            if (action->blocking_type !=
                vda5050::order::ActionBlockingType::NONE) {
              wait_act_ord_end.insert(action->action_id);
            }
            ord.nodes.back().actions.push_back(*action);
          }
        }
      }
    }
  }
  CLOG(INFO, driver_log) << name << " " << ss.str() << "\n";
  //
  auto ord_js = ord.to_json();
  // LOG(INFO) << ord_js.to_string();
  auto opt = jsoncons::json_options{}.precision(15).float_format(
      jsoncons::float_chars_format::general);
  std::string msg_str;
  ord_js.dump(msg_str, opt);
  jsoncons::json vaild = jsoncons::json::parse(msg_str, opt);
  assert(vda5050::order_validate(vaild).empty());
  auto msg = mqtt::make_message(prefix + "order", msg_str, 0, false);
  mqtt_cli->mqtt_client->publish(msg)->wait();

  // wait move and actions
  int n{0};
  bool run_ok{false};
  bool act_start_ok{false};
  bool act_end_ok{false};
  while (task_run) {
    if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " master not online\n";
      task_run = false;
      return false;
    }
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " vehlicle not online\n";
      task_run = false;
      return false;
    }
    if (current_order->state == data::order::TransportOrder::State::FAILED) {
      CLOG(ERROR, driver_log) << name << " order failed\n";
      task_run = false;
      return false;
    }
    if (vdastate.order_id == Pre + "-" + uuids::to_string(order_id) &&
        vdastate.order_update_id == (update_vda_order_id)) {
      auto p = get_time_from_str(vdastate.timestamp);
      if (p.has_value()) {
        auto dt = get_now_utc_time() - p.value();
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
      // wait ord_start
      if (!act_start_ok) {
        if (wait_act_ord_start.empty()) {
          act_start_ok = true;
        } else {
          for (auto& x : vdastate.actionstates) {
            if (wait_act_ord_start.find(x.action_id) !=
                wait_act_ord_start.end()) {
              if (x.action_status == vda5050::state::ActionStatus::WAITING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::RUNNING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::INITIALIZING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::FAILED) {
                CLOG(ERROR, driver_log)
                    << name << " action:[" << x.action_id << "] failed\n";
                // act_start_ok = true;
                // TODO TEST
                task_run = false;
                return false;
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::FINISHED) {
                CLOG(INFO, driver_log) << name << " " << "wait action ["
                                       << x.action_id << "] ok. \n";
                wait_act_ord_start.erase(wait_act_ord_start.find(x.action_id));
              }
            }
          }
          if (wait_act_ord_start.empty()) {
            CLOG(INFO, driver_log) << name << " all vad_start action ok.\n";
            act_start_ok = true;
          }
        }
      }

      if (!run_ok) {
        // 路径移动完成
        auto it =
            std::find_if(vdastate.edgestates.begin(), vdastate.edgestates.end(),
                         [&](vda5050::state::EdgeState s) {
                           return s.edge_id == ord.edges.front().edge_id;
                         });
        // 到达点
        auto it_end_point =
            std::find_if(vdastate.nodestates.begin(), vdastate.nodestates.end(),
                         [&](vda5050::state::NodeState s) {
                           return s.node_id == (ord.nodes.begin() + 1)->node_id;
                         });
        if (it == vdastate.edgestates.end() &&
            it_end_point == vdastate.nodestates.end()) {
          CLOG(INFO, driver_log)
              << name << " " << "move along " << name_ << " ok\n";
          run_ok = true;
          // per act 服务器本地调用 只等待第一步的
          // TODO
          for (auto& x : steps) {
            for (auto& op : x->path->per_acts.acts) {
              if (op.execution_trigger == "AFTER_MOVEMENT") {
                CLOG(INFO, driver_log)
                    << "do " << op.location_name << "[" << op.op_name << "]";
                if (op.completion_required && x == steps.front()) {
                  wait_act_ord_end.insert(op.op_name);
                }
              }
            }
          }
        }
      }
      if (!act_end_ok) {
        if (wait_act_ord_end.empty()) {
          act_end_ok = true;
        } else {
          for (auto& x : vdastate.actionstates) {
            if (wait_act_ord_end.find(x.action_id) != wait_act_ord_end.end()) {
              if (x.action_status == vda5050::state::ActionStatus::WAITING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::RUNNING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::INITIALIZING) {
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::FAILED) {
                CLOG(ERROR, driver_log)
                    << name << " action:[" << x.action_id << "] failed\n";
                // act_end_ok = true;
                task_run = false;
                // TODO TEST
                return false;
              } else if (x.action_status ==
                         vda5050::state::ActionStatus::FINISHED) {
                CLOG(INFO, driver_log) << name << " " << "wait action ["
                                       << x.action_id << "] ok.\n ";
                wait_act_ord_end.erase(wait_act_ord_end.find(x.action_id));
              }
            }
          }
          if (wait_act_ord_end.empty()) {
            CLOG(INFO, driver_log) << name << " all vad_end action ok.\n";
            act_end_ok = true;
          }
        }
      }
      // TODO TEST
      // if (run_ok) {
      //   task_run = false;
      //   return true;
      // }
      if (act_start_ok && act_end_ok && run_ok) {
        task_run = false;
        return true;
      }

    } else {
      CLOG_EVERY_N(20, WARNING, driver_log)
          << name << " "
          << "order state has not been updated,wait_for {\'" + Pre + "-"
          << uuids::to_string(order_id) << " -> update_order_id_"
          << update_vda_order_id << "\'}, but now is {\'" << vdastate.order_id
          << " -> update_order_id_" << vdastate.order_update_id << "\'}\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    CLOG_EVERY_N(20, INFO, driver_log)
        << name << " waiting for move {" << steps.front()->name << "}\n";
    if (n > 120000) {  // 100min
      CLOG(ERROR, driver_log) << name << " " << "timeout";
      task_run = false;
      return false;
    }
  }
  CLOG(WARNING, driver_log) << name << " " << "task cancel";
  task_run = false;
  return false;
}
bool Rabbit3::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  CLOG(INFO, driver_log) << name << " " << "action execute ["
                         << dest->get_type() << "] at "
                         << dest->destination.lock()->name << "\n";
  return true;
  task_run = true;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "master not online";
    task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "vehlicle not online";
    task_run = false;
    return false;
  }

  int ver = static_cast<int>(std::stod(mqtt_cli->version));
  auto vda_version = "v" + std::to_string(ver);
  auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt(get_now_utc_time());
  ord.version = mqtt_cli->version;
  ord.manufacturer = mqtt_cli->manufacturer;
  ord.serial_number = mqtt_cli->serial_number;
  last_step_count = 0;
  if (now_order_state == nowOrder::BEGIN) {
    ord.order_id = dest->get_type() + "-" + uuids::to_string(order_id);
    update_vda_order_id++;
    ord.order_update_id = update_vda_order_id;
    seq_id = (seq_id - 1);
  } else {
    now_order_state = nowOrder::BEGIN;
    seq_id = 0;
    order_id = get_uuid();
    ord.order_id = dest->get_type() + "-" + uuids::to_string(order_id);
    update_vda_order_id = 0;
    ord.order_update_id = update_vda_order_id;
  }
  {
    auto node = vda5050::order::Node();
    auto act = vda5050::order::Action();
    node.node_position = vda5050::order::NodePosition();
    node.node_position.value().map_id = map_id;
    node.sequence_id = seq_id++;
    auto t = resource.lock()->find(dest->destination.lock()->name);
    for (auto& pro_ : dest->properties) {
      auto param = vda5050::order::ActionParam();
      param.key = pro_.first;
      param.value = pro_.second;
      act.action_parameters.value().push_back(param);
    }
    if (t.first == allocate::ResourceManager::ResType::Point) {
      auto t1 = std::dynamic_pointer_cast<data::model::Point>(
          dest->destination.lock());
      node.node_id = t1->name;
      node.node_position.value().x = t1->position.x() / 1000.0;
      node.node_position.value().y = t1->position.y() / 1000.0;
      node.node_position.value().theta = t1->vehicle_orientation;
      node.node_position->allowed_deviation_xy = dest_deviation_xy;
      node.node_position->allowed_deviation_theta = dest_deviation_theta;
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      auto param_x = vda5050::order::ActionParam();
      param_x.key = "point_x";
      param_x.value = std::to_string((double)t1->position.x() / 1000.0);
      act.action_parameters->push_back(param_x);
      auto param_y = vda5050::order::ActionParam();
      param_y.key = "point_y";
      param_y.value = std::to_string((double)t1->position.y() / 1000.0);
      act.action_parameters->push_back(param_y);
    } else if (t.first == allocate::ResourceManager::ResType::Location) {
      auto t1 = std::dynamic_pointer_cast<data::model::Location>(
          dest->destination.lock());
      act.action_parameters = std::vector<vda5050::order::ActionParam>();
      node.node_id = t1->link.lock()->name;
      node.node_position.value().x = t1->link.lock()->position.x() / 1000.0;
      node.node_position.value().y = t1->link.lock()->position.y() / 1000.0;
      node.node_position.value().theta = t1->link.lock()->vehicle_orientation;
      node.node_position->allowed_deviation_xy = dest_deviation_xy;
      node.node_position->allowed_deviation_theta = dest_deviation_theta;
      auto param_x = vda5050::order::ActionParam();
      param_x.key = "location_x";
      param_x.value = (double)t1->position.x() / 1000.0;
      act.action_parameters->push_back(param_x);
      auto param_y = vda5050::order::ActionParam();
      param_y.key = "location_y";
      param_y.value = (double)t1->position.y() / 1000.0;
      act.action_parameters->push_back(param_y);
      std::string op_type = dest->get_type();
      std::transform(op_type.begin(), op_type.end(), op_type.begin(),
                     ::tolower);
      auto it = t1->type.lock()->allowed_ops.find(op_type);
      auto params = it->second;
      for (auto& param : params) {
        auto p_ = vda5050::order::ActionParam();
        p_.key = param.first;
        p_.value = (std::string)param.second;
        act.action_parameters->push_back(p_);
      }
    } else {
      CLOG(ERROR, driver_log) << name << " " << "dest type is err";
      return false;
    }

    node.released = true;
    act.action_id = ord.order_id + "-action";
    act.action_type = dest->operation;
    act.blocking_type = vda5050::order::ActionBlockingType::HARD;
    // TODO TEST
    //  node.actions.push_back(act);
    ord.nodes.push_back(node);
  }
  auto ord_js = ord.to_json();
  auto opt = jsoncons::json_options{}.precision(15).float_format(
      jsoncons::float_chars_format::general);
  std::string msg_str;
  ord_js.dump(msg_str, opt);
  jsoncons::json vaild = jsoncons::json::parse(msg_str, opt);
  assert(vda5050::order_validate(vaild).empty());
  auto msg = mqtt::make_message(prefix + "order", msg_str);
  mqtt_cli->mqtt_client->publish(msg)->wait();
  // // wait
  int n{0};
  while (task_run) {
    if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " " << "master not online\n";
      task_run = false;
      return false;
    }
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " " << "vehlicle not online\n";
      task_run = false;
      return false;
    }
    if (current_order->state == data::order::TransportOrder::State::FAILED) {
      CLOG(ERROR, driver_log) << name << " order failed\n";
      task_run = false;
      return false;
    }
    if (vdastate.order_id ==
            dest->get_type() + "-" + uuids::to_string(order_id) &&
        vdastate.order_update_id == (update_vda_order_id)) {
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
        if (x.action_id != vdastate.order_id + "-action") {
          continue;
        }
        auto sta = x.action_status;
        if (sta == vda5050::state::ActionStatus::FINISHED) {
          CLOG(INFO, driver_log)
              << name << " " << "action [" << dest->get_type() << "]ok\n ";
        } else if (sta == vda5050::state::ActionStatus::FAILED) {
          CLOG(INFO, driver_log) << name << " " << "action failed"
                                 << dest->destination.lock()->name;
          task_run = false;
          // TODO TEST
          return true;
          // return false;
        } else if (sta == vda5050::state::ActionStatus::WAITING) {
          all_ok = false;
        } else if (sta == vda5050::state::ActionStatus::RUNNING) {
          all_ok = false;
        } else if (sta == vda5050::state::ActionStatus::INITIALIZING) {
          all_ok = false;
        }
      }
      if (all_ok) {
        CLOG(INFO, driver_log) << name << " " << "all actions ok\n";
        task_run = false;
        return true;
      }
    } else {
      CLOG_EVERY_N(20, WARNING, driver_log)
          << name << " " << "order state has not been updated,needed {\'"
          << dest->get_type() + "-" << uuids::to_string(order_id)
          << " -> update_ord_id_" << update_vda_order_id
          << "\'}, but now is {\'" << vdastate.order_id << " ->update_ord_id_ "
          << vdastate.order_update_id << "}\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    if (n > 2 * 60 * 100) {
      CLOG(ERROR, driver_log) << name << " " << "timeout";
      task_run = false;
      return false;
    }
  }
  task_run = false;
  CLOG(WARNING, driver_log) << name << " " << "task cancel";
  return false;
}
bool Rabbit3::instant_action(
    std::shared_ptr<data::model::Actions::Action> act) {
  CLOG(INFO, driver_log) << "instantaction " << act->action_id;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "master not online";
    instanttask_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "vehlicle not online";
    instanttask_run = false;
    return false;
  }
  int ver = static_cast<int>(std::stod(mqtt_cli->version));
  auto vda_version = "v" + std::to_string(ver);
  auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto insact = std::make_shared<vda5050::instantaction::InstantAction>();
  insact->serial_number = mqtt_cli->serial_number;
  insact->header_id = send_header_id++;
  insact->manufacturer = mqtt_cli->manufacturer;
  insact->timestamp = get_time_fmt(get_now_utc_time());
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
      CLOG(ERROR, driver_log) << name << " " << "master not online";
      instanttask_run = false;
      return false;
    }
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " " << "vehlicle not online";
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
      CLOG(ERROR, driver_log) << name << " " << "timeout";
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