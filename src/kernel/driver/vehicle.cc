
#include "../../../include/kernel/driver/vehicle.hpp"

#include <Python.h>

#include "../../../include/3rdparty/fs/filesystem.hpp"
#include "../../../include/component/util/tools.hpp"
#include "../../../include/component/vda5050/valitator.hpp"
#include "../../../include/component/vda5050/vda5050order.hpp"
#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/allocate/resource.hpp"
#include "../../../include/kernel/dispatch/dispatch.hpp"

namespace kernel::driver {

std::string vehicle_state_to_str(Vehicle::State state) {
  std::string res{"UNKNOWN"};
  if (state == Vehicle::State::UNAVAILABLE) {
    res = "UNAVAILABLE";
  } else if (state == Vehicle::State::ERROR) {
    res = "ERROR";

  } else if (state == Vehicle::State::IDLE) {
    res = "IDLE";

  } else if (state == Vehicle::State::EXECUTING) {
    res = "EXECUTING";

  } else if (state == Vehicle::State::CHARGING) {
    res = "CHARGING";
  }
  return res;
}
allocate::TransOrderPtr Vehicle::redistribute_cur_order() {
  std::unique_lock<std::shared_mutex> lock(current_order->mutex);
  if (current_order) {
    LOG(DEBUG) << name << " " << current_order->name
               << " redistribute current order\n";
    auto ord = current_order;
    auto order_pool = orderpool.lock();
    auto res = resource.lock();
    if (!res) {
      CLOG(ERROR, driver_log) << name << " resource is null\n";
      return nullptr;
    }
    if (!order_pool) {
      CLOG(ERROR, driver_log) << name << " order_pool is null\n";
      return nullptr;
    }
    process_state = proState::IDEL;
    avoid_state = Avoid::Normal;
    now_order_state = Vehicle::nowOrder::END;
    std::unique_lock<std::shared_mutex> lock(res_mut);
    future_allocate_resources.clear();
    claim_resources.clear();
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto &a : this->allocated_resources) {
      for (auto &x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    lock.unlock();
    res->free(temp, shared_from_this());

    ord->conflict_pool.lock()->reset_state(ord);
    LOG(DEBUG) << name << " " << ord->name << " redistribute current order\n";
    // order_pool->redistribute(ord);
    // avoid_state = Avoid::Normal;
    CLOG_IF(state != State::IDLE, INFO, driver_log)
        << name << " " << "state transform to : ["
        << vehicle_state_to_str(State::IDLE) << "]\n";
    current_order.reset();
    state = State::IDLE;
    last_step.clear();
    future_step.clear();
    idle_time = get_now_utc_time();
    CLOG(INFO, driver_log) << name << " " << "now is idle "
                           << get_time_fmt(idle_time) << " at "
                           << last_point->name << "\n";
    if (current_command) {
      current_command->state = Command::State::REDISBUTE;
    }
    return ord;
  } else {
    return nullptr;
  }
}
void Vehicle::execute_action(
    const std::shared_ptr<data::order::DriverOrder::Destination> &dest) {
  io_context.post([=] {
    LOG(DEBUG) << name << " " << "action ---- {"
               << dest->destination.lock()->name << "}\n";
    std::unique_lock<std::shared_mutex> lock(current_order->mutex);
    auto ord = current_order;
    LOG(DEBUG) << name << " " << "action  {" << dest->destination.lock()->name
               << "}\n";

    if (!current_order) {
      CLOG(ERROR, driver_log) << name << " current_order is null\n";
      state = State::ERROR;
      CLOG(DEBUG, driver_log) << name << " " << "state transform to : ["
                              << vehicle_state_to_str(state) << "]\n";
      return;
    }
    lock.unlock();
    auto op_ret = action(dest);
    lock.lock();
    if (current_order != ord) {
      CLOG(DEBUG, driver_log) << name << " " << "action {"
                              << dest->destination.lock()->name << "} failed\n";
      return;
    }
    if (!op_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = get_now_utc_time();
      CLOG(WARNING, driver_log)
          << name << " " << current_order->name << " action : " << " failed\n";
      state = State::ERROR;
      current_order.reset();
      CLOG(DEBUG, driver_log) << name << " " << "state transform to : ["
                              << vehicle_state_to_str(state) << "]\n";

    } else {
      CLOG(DEBUG, driver_log)
          << name << " " << current_order->name << " action : " << " ok\n";
    }
    if (current_command) {
      current_command->vehicle_execute_cb(op_ret);
    }
  });
}
void Vehicle::execute_move(
    const std::vector<std::shared_ptr<data::order::Step>> &steps) {
  io_context.post([=] {
    CLOG(DEBUG, driver_log)
        << name << " " << (int)current_command->state << "}\n";
    if (current_command->state != Command::State::EXECUTING) {
      LOG(FATAL) << "ddd";
    }
    std::unique_lock<std::shared_mutex> lock(current_order->mutex);
    auto ord = current_order;
    CLOG(DEBUG, driver_log)
        << name << " " << "move {" << steps.front()->name << "}\n";
    if (!current_order) {
      // state = State::ERROR;
      CLOG(DEBUG, driver_log) << name << " " << "state transform to : ["
                              << vehicle_state_to_str(state) << "]\n";
      return;
    }
    lock.unlock();
    auto move_ret = move(steps);
    lock.lock();
    if (current_order != ord) {
      CLOG(DEBUG, driver_log)
          << name << " " << "move {" << steps.front()->name << "} failed\n";
      return;
    }
    if (!move_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = get_now_utc_time();
      std::stringstream ss;
      ss << current_order->name << " : move {" << steps.front()->name << "}";
      CLOG(WARNING, driver_log) << name << " " << ss.str() << " failed\n";
      state = State::ERROR;
      current_order.reset();
      CLOG(INFO, driver_log) << name << " " << "state transform to : ["
                             << vehicle_state_to_str(state) << "]\n";
    } else {
      std::stringstream ss;
      ss << current_order->name << " : move {" << steps.front()->name << "}";
      CLOG(DEBUG, driver_log) << name << " " << ss.str() << " ok\n";
    }

    if (current_command &&
        current_command->state == Command::State::EXECUTING) {
      current_command->vehicle_execute_cb(move_ret);
    }
  });
}
void Vehicle::execute_instatn_action(
    const std::shared_ptr<vda5050::instantaction::Action> &act) {
  current_action = act;
  io_context.post([=] {
    auto ret = instant_action(current_action);
    if (!ret) {
      CLOG(ERROR, driver_log)
          << name << " " << current_action->action_id << " failed";
    } else {
      CLOG(DEBUG, driver_log)
          << name << " " << current_action->action_id << " ok";
    }
  });
}
void Vehicle::cancel_all_order() {
  for (auto &ord : orders) {
    std::unique_lock<std::shared_mutex> lock(ord->mutex);
    ord->state = data::order::TransportOrder::State::FAILED;
  }
  orders.clear();
}

void Vehicle::run() {
  init();
  for (int i = 0; i < 4; i++) {
    auto th = new std::thread([this] { io_context.run(); });
    run_th[i] = th;
  }
}
void Vehicle::close() {
  task_run = false;
  instant_task_run = false;
  io_context.stop();
  for (int i = 0; i < 4; i++) {
    run_th[i]->join();
  }
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
bool Vehicle::plan_route(allocate::TransOrderPtr cur) {
  auto res = resource.lock();
  auto ordpoll = orderpool.lock();
  auto route_planner = planner.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return false;
  }
  if (!ordpoll) {
    CLOG(ERROR, driver_log) << name << " orderpoll is null\n";
    return false;
  }
  if (!route_planner) {
    CLOG(ERROR, driver_log) << name << " planner is null\n";
    return false;
  }
  bool first_driverorder{true};
  std::shared_ptr<data::model::Point> start_planner;
  std::shared_ptr<data::model::Point> end_planner;
  for (int i = cur->current_driver_index; i < cur->driverorders.size(); i++) {
    auto op = cur->driverorders[i];
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
      CLOG(WARNING, driver_log)
          << name << " " << cur->name << " can not find obj "
          << destination->destination.lock()->name
          << " or can not locate current_pos\n";
      return false;
    } else {
      // LOG(INFO) << "start_planner : " << start_planner->name
      //           << " end_planner : " << end_planner->name;
      auto path = route_planner->find_paths(start_planner, end_planner);
      if (path.empty()) {
        CLOG(WARNING, driver_log)
            << name << " " << cur->name << " can not routable\n";
        return false;
      } else {
        auto driverorder = ordpoll->route_to_driverorder(
            res->paths_to_route(path.front()), destination);
        driverorder->transport_order = cur;
        driverorder->state = data::order::DriverOrder::State::PRISTINE;
        cur->driverorders[i] = driverorder;
      }
    }
  }
  std::unique_lock<std::shared_mutex> lock2(res_mut);
  claim_resources.clear();
  for (int i = cur->current_driver_index; i < cur->driverorders.size(); i++) {
    auto x = cur->driverorders[i];
    for (auto &st : x->route->steps) {
      std::unordered_set<std::shared_ptr<RSSResource>> set;
      set.insert(st->path);
      set.insert(st->path->source_point.lock());
      set.insert(st->path->destination_point.lock());
      claim_resources.push_back(set);
    }
    std::unordered_set<std::shared_ptr<RSSResource>> set;
    set.insert(x->destination->destination.lock());
    claim_resources.push_back(set);
  }
  return true;
}

void Vehicle::get_next_ord() {
  // 获取新订单
  if (orders.empty()) {
    // state = State::IDLE;
  } else {
    current_order = orders.front();
    orders.pop_front();
  }
  if (current_order) {
    // state = State::EXECUTING;
    // current_order->state =
    // data::order::TransportOrder::State::BEING_PROCESSED;
    next_command();
  } else {
    process_state = proState::IDEL;
    avoid_state = Avoid::Normal;
    if (process_charging) {
      CLOG_IF(state != State::CHARGING, INFO, driver_log)
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::CHARGING) << "]\n";
      state = State::CHARGING;
    } else {
      CLOG_IF(state != State::IDLE, INFO, driver_log)
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::IDLE) << "]\n";
      state = State::IDLE;
      last_step.clear();
      future_step.clear();

      avoid_state = Avoid::Normal;
      idle_time = get_now_utc_time();
      CLOG(INFO, driver_log)
          << name << " " << "now is idle " << get_time_fmt(idle_time) << "\n";
    }
  }
}

void Vehicle::command_done() {
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return;
  }
  if (!current_order) {
    get_next_ord();
    return;
  }
  std::unique_lock<std::shared_mutex> lock(current_order->mutex);
  if (reroute_flag) {
    plan_route(current_order);
    reroute_flag = false;
  }
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
    process_state = proState::AWAITING_ORDER;
    // 订单取消
    std::unique_lock<std::shared_mutex> lock(res_mut);
    future_allocate_resources.clear();
    claim_resources.clear();
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto &a : this->allocated_resources) {
      for (auto &x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    lock.unlock();
    res->free(temp, shared_from_this());
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " withdrawl.\n";
    if (process_charging) {
      process_charging = false;
    }
    now_order_state = Vehicle::nowOrder::END;
    lock.lock();
    current_order->state = data::order::TransportOrder::State::FAILED;
    current_order.reset();
    lock.unlock();
    get_next_ord();
    return;
  } else if (current_order->state ==
             data::order::TransportOrder::State::UNROUTABLE) {
    // 订单不可达
    std::unique_lock<std::shared_mutex> lock(res_mut);
    future_allocate_resources.clear();
    claim_resources.clear();
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " unrouteable.";
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto &a : this->allocated_resources) {
      for (auto &x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    lock.unlock();
    res->free(temp, shared_from_this());
    if (process_charging) {
      process_charging = false;
    }
    now_order_state = Vehicle::nowOrder::END;
    current_order->state = data::order::TransportOrder::State::FAILED;
    // current_order.reset();
    // get_next_ord();
    return;
  }
  if (current_order->dead_time < get_now_utc_time()) {
    CLOG(ERROR, driver_log)
        << name << " " << current_order->name << " timeout.";
    std::unique_lock<std::shared_mutex> lock(res_mut);
    current_order->state = data::order::TransportOrder::State::FAILED;
    future_allocate_resources.clear();
    claim_resources.clear();
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto &a : this->allocated_resources) {
      for (auto &x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    lock.unlock();
    res->free(temp, shared_from_this());
    now_order_state = Vehicle::nowOrder::END;
    return;
  }
  // if (current_order->state == data::order::TransportOrder::State::FAILED) {
  //   // 订单失败
  //   future_allocate_resources.clear();
  //   std::vector<std::shared_ptr<RSSResource>> temp;
  //   for (auto &a : this->allocated_resources) {
  //     for (auto &x : a) {
  //       if (x != this->current_point) {
  //         temp.push_back(x);
  //       }
  //     }
  //   }
  //   res->free(temp, shared_from_this());
  //   now_order_state = Vehicle::nowOrder::END;
  //   CLOG(ERROR, driver_log)
  //       << current_order->name << " failed, requires manual intervention.\n";
  //   // current_order.reset();
  //   // get_next_ord();
  //   return;
  // }
  // 完成step or action
  auto dr = (current_order->driverorders[current_order->current_driver_index]);
  if (dr->state == data::order::DriverOrder::State::FINISHED) {
    current_order->current_driver_index += 1;
    // LOG(INFO) << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
    //           << current_order->current_driver_index;
  } else if (dr->state == data::order::DriverOrder::State::FAILED) {
    current_order->state = data::order::TransportOrder::State::FAILED;
    now_order_state = Vehicle::nowOrder::END;
    CLOG(ERROR, driver_log) << name << " " << current_order->name
                            << " failed, requires manual "
                               "intervention.\n";
    std::unique_lock<std::shared_mutex> lock2(res_mut);

    future_allocate_resources.clear();
    claim_resources.clear();
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto &a : this->allocated_resources) {
      for (auto &x : a) {
        if (x != this->current_point) {
          temp.push_back(x);
        }
      }
    }
    lock2.unlock();
    res->free(temp, shared_from_this());
    // current_order.reset();
    // get_next_ord();
    return;
  }
  if (current_order->driverorders.size() <=
      current_order->current_driver_index) {
    process_state = proState::AWAITING_ORDER;
    current_order->state = data::order::TransportOrder::State::FINISHED;
    CLOG(INFO, driver_log) << name << " order " << current_order->name
                           << " status: [finished]\n";
    current_order->end_time = get_now_utc_time();
    now_order_state = Vehicle::nowOrder::END;
    current_order.reset();
    current_command.reset();
    std::unique_lock<std::shared_mutex> lock2(res_mut);
    future_allocate_resources.clear();
    claim_resources.clear();
    lock2.unlock();
    lock.unlock();
    get_next_ord();
  } else {
    // 继续执行订单
    if (current_order->state ==
        data::order::TransportOrder::State::BEING_PROCESSED) {
      lock.unlock();
      next_command();
      // state = State::EXECUTING;
    } else {
      // 订单取消了或失败了
      LOG(DEBUG) << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^ "
                 << (int)current_order->state;
      if (current_order->state !=
          data::order::TransportOrder::State::BEING_PROCESSED) {
        CLOG(FATAL, driver_log)
            << name << " " << current_order->name << " status: [failed]\n";
      }
      command_done();
    }
  }
}

std::string Vehicle::get_state() const {
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
std::string Vehicle::get_process_state() const {
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
  auto dispatcher_ = dispatcher.lock();
  if (!current_order) {
    get_next_ord();
    return;
  }
  std::unique_lock<std::shared_mutex> lock(current_order->mutex);
  if (!scheduler_ || !dispatcher_) {
    current_order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " scheduler  or dispatcher is null\n";
    return;
  }

  if (current_order->state ==
      data::order::TransportOrder::State::DISPATCHABLE) {
    current_order->state = data::order::TransportOrder::State::BEING_PROCESSED;
    dispatcher_->pop_order(current_order);
    CLOG(INFO, order_log) << current_order->name
                          << " status: {begin_processed}\n";
    lock.unlock();
    plan_route(current_order);  // 首次执行，规划路径
  }
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
    current_order->state = data::order::TransportOrder::State::FAILED;
    current_order.reset();
    if (process_charging) {
      process_charging = false;
    }
    lock.unlock();
    get_next_ord();
    return;
  }
  if (current_order->state == data::order::TransportOrder::State::FAILED) {
    // order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << current_order->name
                            << " failed : " << name << " state is FAILED";
    return;
  }
  if (current_order->state == data::order::TransportOrder::State::FINISHED) {
    lock.unlock();
    get_next_ord();
    return;
  }
  current_command = scheduler_->new_command(shared_from_this());
  // run
  scheduler_->add_command(current_command);
  process_state = proState::PROCESSING_ORDER;
}
void Vehicle::receive_task(
    const std::shared_ptr<data::order::TransportOrder> &order) {
  CLOG(INFO, driver_log) << name << " at " << last_point->name
                         << " receive new order " << order->name << "\n";
  if (state == State::ERROR) {
    // order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is ERROR";
  } else if (state == State::UNAVAILABLE) {
    // order->state = data::order::TransportOrder::State::FAILED;
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is UNAVAILABLE";
  } else if (state == State::IDLE) {
    CLOG_IF(state != State::EXECUTING, INFO, driver_log)
        << name << " state transform to : ["
        << vehicle_state_to_str(State::EXECUTING) << "]\n";
    state = State::EXECUTING;
    avoid_state = Avoid::Normal;
    process_state = proState::AWAITING_ORDER;
    current_order.reset();
    orders.push_back(order);
    if (order->state != data::order::TransportOrder::State::DISPATCHABLE) {
      LOG(FATAL) << "ssss";
    }
    next_command();
  } else if (state == State::EXECUTING) {
    if (current_order && current_order->anytime_drop) {
      current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      LOG(DEBUG) << "__________________|||||||||||||||||||||||||";
    }
    if (process_charging) {
      process_charging = false;
    }
    orders.push_back(order);
    if (current_order &&
        current_order->state == data::order::TransportOrder::State::WITHDRAWL) {
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order.reset();
      next_command();
    }
  } else if (state == State::CHARGING) {
    orders.push_back(order);
    if (energy_level > energy_level_good) {
      //  stop 充电
      CLOG_IF(state != State::EXECUTING, INFO, driver_log)
          << name << " " << "state transform to : ["
          << vehicle_state_to_str(State::EXECUTING) << "]";
      state = State::EXECUTING;
      process_charging = false;
      next_command();
    }
  } else {
    // order->state = data::order::TransportOrder::State::FAILED;
    CLOG(WARNING, dispatch_log) << order->name << " status: [failed]\n";
    CLOG(ERROR, driver_log) << name << " " << order->name
                            << " failed : " << name << " state is UNKNOWN\n";
  }
}
bool SimVehicle::action(
    const std::shared_ptr<data::order::DriverOrder::Destination> &dest) {
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
    // return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    last_point = t->link.lock();
    CLOG(DEBUG, driver_log) << name << " now at (" << t->name << ")\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->link.lock()->position;
    last_point = t->link.lock();
    CLOG(DEBUG, driver_log) << name << " now at (" << t->name << ")\n";
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
    CLOG(DEBUG, driver_log)
        << name << " now at (" << position.x() << " , " << position.y() << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->link.lock()->position;
    last_point = t->link.lock();
    current_point = last_point;
    CLOG(DEBUG, driver_log)
        << name << " now at (" << position.x() << " , " << position.y() << ")";
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::NOP) {
    return true;
  }
  return true;
}
bool SimVehicle::instant_action(
    const std::shared_ptr<data::model::Actions::Action> &act) {
  CLOG(DEBUG, driver_log) << act->action_id << " ok";
  return true;
};
void SimVehicle::init() {
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is null\n";
    return;
  }
  state = State::IDLE;
  last_step.clear();
  future_step.clear();

  avoid_state = Avoid::Normal;
  if (!last_point) {
    state = State::UNKNOWN;
    return;
  }
  orderpool.lock()->veh_ps[name] = last_point;
  current_point = last_point;
  this->position = current_point->position;
  if (integration_level == integrationLevel::TO_BE_IGNORED ||
      integration_level == integrationLevel::TO_BE_NOTICED) {
  } else {
    idle_time = get_now_utc_time();
    std::vector<std::shared_ptr<RSSResource>> ress;
    ress.push_back(current_point);
    // res->claim(ress, shared_from_this());
    res->allocate(ress, shared_from_this());
  }
}
bool SimVehicle::move(
    const std::vector<std::shared_ptr<data::order::Step>> &steps) {
  if (steps.empty()) {
    return false;
  }
  const auto &step = steps.front();
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
    angle = std::atan2(y_len, x_len);
    for (int i = 0; i < 100; i++) {
      position.x() += x_len / 100;
      position.y() += y_len / 100;
      angle = std::atan2(y_len, x_len);
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 100));
    }
    position.x() = end->position.x();
    position.y() = end->position.y();
    last_point = end;
    current_point = last_point;
    CLOG(DEBUG, driver_log)
        << name << " now at (" << current_point->name << ")\n";
    return true;
  } else if (step->vehicle_orientation ==
             data::order::Step::Orientation::BACKWARD) {
    auto end = step->path->source_point.lock();
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    int x_len = end->position.x() - position.x();
    int y_len = end->position.y() - position.y();
    angle = std::atan2(y_len, x_len);
    for (int i = 0; i < 100; i++) {
      position.x() += x_len / 100;
      position.y() += y_len / 100;
      angle = std::atan2(y_len, x_len);
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 100));
    }
    position.x() = end->position.x();
    position.y() = end->position.y();
    last_point = end;
    current_point = last_point;
    CLOG(DEBUG, driver_log)
        << name << " now at (" << current_point->name << ")\n";
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
      [&](const mqtt::const_message_ptr &msg) { this->onstate(msg); });
  mqtt_cli->mqtt_client->set_func(
      prefix + "connection",
      [&](const mqtt::const_message_ptr &msg) { this->onconnect(msg); });
  mqtt_cli->start();
}

void Rabbit3::onstate(const mqtt::const_message_ptr &msg) {
  // cpu_timer t;
  auto res = resource.lock();
  if (!res) return;
  auto m = msg->get_payload();
  try {
    jsoncons::json v;
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
          std::unique_lock<std::shared_mutex> lock(current_order->mutex);
          current_order->state = data::order::TransportOrder::State::FAILED;
          CLOG(WARNING, dispatch_log)
              << current_order->name << " status: [failed]\n";
        }
        state = Vehicle::State::UNKNOWN;
        std::vector<std::shared_ptr<RSSResource>> temp;
        std::shared_lock<std::shared_mutex> lock(res_mut);
        for (auto &a : allocated_resources) {
          for (auto &x : a) {
            temp.push_back(x);
          }
        }
        lock.unlock();
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
        for (auto &x : res->points) {
          // LOG(INFO) << x->name;
          if (x->name == vdastate.last_node_id) {
            last_point = x;
            current_point = last_point;
            if (!init_pos) {
              this->position = current_point->position;
              idle_time = get_now_utc_time();
              std::vector<std::shared_ptr<RSSResource>> temp;
              std::shared_lock<std::shared_mutex> lock(res_mut);
              for (auto &a : allocated_resources) {
                for (auto &x_ : a) {
                  if (x_ != last_point) temp.push_back(x_);
                }
              }
              lock.unlock();
              res->free(temp, shared_from_this());
              std::vector<std::shared_ptr<RSSResource>> ress;
              ress.push_back(x);
              // res->claim(ress, shared_from_this());
              if (!res->allocate(ress, shared_from_this())) {
                CLOG(ERROR, driver_log)
                    << name << " init failed:{ allocate init_point failed }\n";
                init_pos = false;
              } else {
                init_pos = true;
                orderpool.lock()->veh_ps[name] = last_point;
              }
            } else {
              if (!vdastate.agv_position.has_value()) {
                this->position = current_point->position;
              }
              if (state == State::IDLE &&
                  last_vdastate.last_node_id != vdastate.last_node_id) {
                LOG(INFO) << last_vdastate.last_node_id << " "
                          << vdastate.last_node_id;
                std::vector<std::shared_ptr<RSSResource>> temp;
                std::shared_lock<std::shared_mutex> lock(res_mut);
                for (auto &a : allocated_resources) {
                  for (auto &x_ : a) {
                    if (x_ != last_point) temp.push_back(x_);
                  }
                }
                lock.unlock();
                res->free(temp, shared_from_this());
                std::vector<std::shared_ptr<RSSResource>> ress;
                ress.push_back(x);
                // res->claim(ress, shared_from_this());
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
      this->position.x() = static_cast<int>(vdastate.agv_position->x * 1000);
      this->position.y() = static_cast<int>(vdastate.agv_position->y * 1000);
      this->map_id = vdastate.agv_position->map_id;
      this->angle = vdastate.agv_position->theta;
      layout = position;
      auto last_state = state;
      if (state == Vehicle::State::UNKNOWN) {
        if (veh_state == vda5050::VehicleMqttStatus::ONLINE) {
          state = State::IDLE;
          last_step.clear();
          future_step.clear();
          avoid_state = Avoid::Normal;
          idle_time = get_now_utc_time();
        }
      }
      if (!vdastate.errors.empty()) {
        state = State::ERROR;
      }

      if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
        state = State::UNKNOWN;
      }
      energy_level = static_cast<int>(vdastate.battery_state.battery_charge);
      if (process_charging) {
        if (energy_level > energy_level_good &&
            state == Vehicle::State::CHARGING) {
          state = Vehicle::State::IDLE;
          avoid_state = Avoid::Normal;
          idle_time = get_now_utc_time();
          process_charging = false;
        }
      }
      if (last_state != state) {
        CLOG(INFO, driver_log) << name << " " << "state transform to : ["
                               << vehicle_state_to_str(state) << "]\n";
      }
    } else {
      std::for_each(ms.begin(), ms.end(), [&](const std::string &s) {
        CLOG(WARNING, driver_log) << name << " " << s;
      });
    }
  } catch (jsoncons::json_exception &ec) {
    CLOG(WARNING, driver_log) << name << " error: " << ec.what();
  }
}

void Rabbit3::onconnect(const mqtt::const_message_ptr &msg) {
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log)
        << name << " " << mqtt_cli->serial_number << " resource is not exist\n";
    return;
  }
  auto m = msg->to_string();
  try {
    jsoncons::json v;
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
          std::shared_lock<std::shared_mutex> lock(res_mut);
          for (auto &a : allocated_resources) {
            for (auto &x : a) {
              temp.push_back(x);
            }
          }
          lock.unlock();
          res->free(temp, shared_from_this());
        }
        rece_header_id = -1;
        init_pos = false;
      } else if (v["connectionState"] == "CONNECTIONBROKEN") {
        if (veh_state != vda5050::VehicleMqttStatus::CONNECTIONBROKEN) {
          veh_state = vda5050::VehicleMqttStatus::CONNECTIONBROKEN;
          state = Vehicle::State::UNKNOWN;
          std::vector<std::shared_ptr<RSSResource>> temp;
          std::shared_lock<std::shared_mutex> lock(res_mut);
          for (auto &a : allocated_resources) {
            for (auto &x : a) {
              temp.push_back(x);
            }
          }
          lock.unlock();
          res->free(temp, shared_from_this());
        }
        rece_header_id = -1;
        init_pos = false;
        CLOG(WARNING, driver_log)
            << name << " " << mqtt_cli->serial_number + " CONNECTIONBROKEN\n";
      }
    } else {
      std::for_each(ms.begin(), ms.end(), [&](const std::string &s) {
        CLOG(WARNING, driver_log) << name << " " << s;
      });
    }
  } catch (jsoncons::json_exception &ec) {
    CLOG(WARNING, driver_log) << name << " " << ec.what();
  }
}

bool Rabbit3::move(
    const std::vector<std::shared_ptr<data::order::Step>> &steps) {
  auto res = resource.lock();
  if (!res) {
    CLOG(ERROR, driver_log) << name << " resource is not exist\n";
    return false;
  }
  auto &driver_order =
      current_order->driverorders[current_order->current_driver_index];

  // 发送多步，走完一步返回
  std::string name_;
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
  int ver = static_cast<int>(std::stod(mqtt_cli->version));
  auto vda_version = "v" + std::to_string(ver);
  auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
                mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number + "/";
  auto ord = vda5050::order::VDA5050Order();
  ord.header_id = send_header_id++;
  ord.timestamp = get_time_fmt_utc(get_now_utc_time());
  ord.version = mqtt_cli->version;
  ord.manufacturer = mqtt_cli->manufacturer;
  ord.serial_number = mqtt_cli->serial_number;
  if (now_order_state == nowOrder::BEGIN) {
    ord.order_id = current_order->name;
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
    // order_id = get_uuid();
    ord.order_id = current_order->name;
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
  if (!std::isnan(start_point->vehicle_orientation)) {
    pos.theta = start_point->vehicle_orientation;
  }
  pos.allowed_deviation_xy = deviation_xy;
  pos.allowed_deviation_theta = deviation_theta;
  start.node_position = pos;
  ord.nodes.push_back(start);
  //////////////////////////////////
  std::vector<data::model::PeripheralActions::PeripheralAction> ready_action;
  std::vector<data::model::PeripheralActions::PeripheralAction>
      after_allocated_action;
  std::vector<data::model::PeripheralActions::PeripheralAction>
      after_moved_action;
  if (steps.front()->type == data::order::Step::Type::FRONT ||
      steps.size() == 1) {
    ready_action.insert(ready_action.end(), start_point->per_acts.acts.begin(),
                        start_point->per_acts.acts.end());
    for (int i = 0; i < steps.size(); ++i) {
      ready_action.insert(ready_action.end(),
                          steps[i]->path->per_acts.acts.begin(),
                          steps[i]->path->per_acts.acts.end());
      if (steps[i]->vehicle_orientation ==
          data::order::Step::Orientation::FORWARD) {
        ready_action.insert(
            ready_action.end(),
            steps[i]->path->destination_point.lock()->per_acts.acts.begin(),
            steps[i]->path->destination_point.lock()->per_acts.acts.end());
      } else {
        ready_action.insert(
            ready_action.end(),
            steps[i]->path->source_point.lock()->per_acts.acts.begin(),
            steps[i]->path->source_point.lock()->per_acts.acts.end());
      }
    }
  } else {
    if (steps.size() > 1) {
      ready_action.insert(ready_action.end(),
                          steps.back()->path->per_acts.acts.begin(),
                          steps.back()->path->per_acts.acts.end());
    }
  }

  for (auto &op : ready_action) {
    if (op.completion_required) continue;
    if (op.execution_trigger == "AFTER_ALLOCATION") {
      after_allocated_action.push_back(op);
    } else {
      after_moved_action.push_back(op);
    }
  }
  ready_action.clear();
  for (int i = 0; i < steps.size(); ++i) {
    ready_action.insert(ready_action.end(),
                        steps[i]->path->per_acts.acts.begin(),
                        steps[i]->path->per_acts.acts.end());
  }
  for (auto &op : ready_action) {
    if (!op.completion_required) continue;
    if (op.execution_trigger == "AFTER_ALLOCATION") {
      after_allocated_action.push_back(op);
    } else {
      after_moved_action.push_back(op);
    }
  }
  // do
  for (auto &op : after_allocated_action) {
    CLOG(INFO, driver_log) << "do " << op.location_name << "[" << op.op_name
                           << "]\n";
    std::map<std::string, std::string> act_param;
    for (auto &loc : res->locations) {
      if (loc->name == op.location_name) {
        for (auto &x : loc->properties) {
          act_param[x.first] = x.second;
        }
      }
    }
    act_param["op"] = op.op_name;
    std::string script = op.script;
    CLOG_IF(script.empty(), WARNING, driver_log)
        << "the script's name is empty\n";
    if (op.completion_required) {
      // wait_act_ord_start.insert(op.op_name);

      auto ret = run_script(script, act_param);
      if (!ret) {
        CLOG(ERROR, driver_log)
            << "do " << op.location_name << "[" << op.op_name << "] failed\n";
        return false;
      } else {
        CLOG(INFO, driver_log)
            << "do " << op.location_name << "[" << op.op_name << "] ok\n";
      }
    } else {
      io_context.post([=] {
        auto ret = run_script(script, act_param);
        CLOG(INFO, driver_log)
            << "do " << op.location_name << "[" << op.op_name << "] "
            << (ret == true ? "ok" : "failed") << "\n";
      });
    }
  }
  /////////////////////////////////
  std::stringstream ss;
  ss << "begin move along [" << start_point->name << " --> ";
  std::string last_id = start_point->name;
  int forward_two{0};  // 只有前两步是Base,后面都是Horizon
  if (steps.front()->type == data::order::Step::Type::FRONT) {
    // 起点
    // vda 遍历当前动作
    for (auto &x : steps.front()->path->acts.actions) {
      auto action = static_cast<vda5050::order::Action *>(&x);
      if (action->when == vda5050::order::ActionWhen::ORDER_START) {
        if (action->blocking_type != vda5050::order::ActionBlockingType::NONE) {
          wait_act_ord_start.insert(action->action_id);
        }
        ord.nodes.front().actions.push_back(*action);
      }
    }
    for (auto &x : driver_order->route->steps) {
      for (auto &act : x->path->acts.actions) {
        auto action = static_cast<vda5050::order::Action *>(&act);
        if (action->when == vda5050::order::ActionWhen::ORDER_START) {
          if (action->blocking_type !=
              vda5050::order::ActionBlockingType::NONE) {
            wait_act_ord_start.insert(action->action_id);
          }
          ord.nodes.front().actions.push_back(*action);
        }
      }
    }
  }
  for (auto &x : steps) {
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
    if (x->type == data::order::Step::Type::BACK) {
      // 订单的动作，不是路径的动作
      pos2.allowed_deviation_xy = (dest_deviation_xy);
      pos2.allowed_deviation_theta = (dest_deviation_theta);
      auto act = vda5050::order::Action();
      auto dest = driver_order->destination;
      auto t = res->find(dest->destination.lock()->name);
      if (order_action_uuid.is_nil()) {
        order_action_uuid = get_uuid();
      }
      act.action_id = "Action-" + uuids::to_string(order_action_uuid);
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
        for (auto &param : params) {
          auto p_ = vda5050::order::ActionParam();
          p_.key = param.first;
          p_.value = (std::string)param.second;
          act.action_parameters->push_back(p_);
        }
        auto param_x = vda5050::order::ActionParam();
        param_x.key = "dstx";
        param_x.value = std::to_string(t1->position.x() / 1000.0);
        act.action_parameters->push_back(param_x);
        auto param_y = vda5050::order::ActionParam();
        param_y.key = "dsty";
        param_y.value = std::to_string(t1->position.y() / 1000.0);
        act.action_parameters->push_back(param_y);
        auto param_sx = vda5050::order::ActionParam();
        param_sx.key = "srcx";
        param_sx.value = std::to_string(t1->link.lock()->position.x() / 1000.0);
        act.action_parameters->push_back(param_sx);
        auto param_sy = vda5050::order::ActionParam();
        param_sy.key = "srcy";
        param_sy.value = std::to_string(t1->link.lock()->position.y() / 1000.0);
        act.action_parameters->push_back(param_sy);
        end.actions.push_back(act);
        if (x == steps.front()) {
          // 只等待第一步的
          wait_act_ord_end.insert(act.action_id);
        }
      } else if (t.first == allocate::ResourceManager::ResType::Point) {
        // TODO
        //  auto t1 = std::dynamic_pointer_cast<data::model::Point>(
        //      dest->destination.lock());
        //  auto param_x = vda5050::order::ActionParam();
        //  param_x.key = "point_x";
        //  param_x.value = std::to_string(t1->position.x() / 1000.0);
        //  act.action_parameters->push_back(param_x);
        //  auto param_y = vda5050::order::ActionParam();
        //  param_y.key = "point_y";
        //  param_y.value = std::to_string(t1->position.y() / 1000.0);
        //  act.action_parameters->push_back(param_y);
      }
    } else {
      pos2.allowed_deviation_xy = (deviation_xy);
      pos2.allowed_deviation_theta = (deviation_theta);
    }
    pos2.map_id = map_id;
    if (!std::isnan(end_point->vehicle_orientation)) {
      pos2.theta = end_point->vehicle_orientation;
    }
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
      for (auto &cp : x->path->layout.control_points) {
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
        for (auto &traj : trajectory.control_points) {
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
      for (auto &cp : x->path->layout.control_points) {
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
        for (auto &traj : trajectory.control_points) {
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

    ord.nodes.push_back(end);
    ord.edges.push_back(e);
    forward_two++;
  }
  if (steps.front()->type == data::order::Step::Type::BACK) {
    // 终点vda动作
    // 遍历当前动作
    for (auto &x : steps.front()->path->acts.actions) {
      auto action = static_cast<vda5050::order::Action *>(&x);
      if (action->when == vda5050::order::ActionWhen::ORDER_END) {
        if (action->blocking_type != vda5050::order::ActionBlockingType::NONE) {
          wait_act_ord_end.insert(action->action_id);
        }
        ord.nodes.back().actions.push_back(*action);
      }
    }
    for (auto &x : driver_order->route->steps) {
      for (auto &act : x->path->acts.actions) {
        auto action = static_cast<vda5050::order::Action *>(&act);
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
  CLOG(DEBUG, driver_log) << name << " " << ss.str() << "\n";
  //
  auto ord_js = ord.to_json();
  // LOG(INFO) << ord_js.to_string();
  auto opt = jsoncons::json_options{}.precision(15).float_format(
      jsoncons::float_chars_format::general);
  std::string msg_str;
  ord_js.dump(msg_str, opt);
  jsoncons::json vaild = jsoncons::json::parse(msg_str, opt);
  auto err = vda5050::order_validate(vaild);
  if (!err.empty()) {
    for (auto &x : err) {
      CLOG(ERROR, driver_log) << name << "err: " << x << "\n";
    }
    return false;
  }
  assert(err.empty());
  auto msg = mqtt::make_message(prefix + "order", msg_str, 0, false);
  mqtt_cli->mqtt_client->publish(msg)->wait();
  // CLOG(INFO, driver_log) << "send order mqtt msg: \n"
  //                        << jsoncons::pretty_print(vaild) << "\n";

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
    if (vdastate.order_id == current_order->name &&
        vdastate.order_update_id == (update_vda_order_id)) {
      auto p = get_time_from_str(vdastate.timestamp);
      if (p.has_value()) {
        auto dt = get_now_utc_time() - p.value();
        if (dt > std::chrono::seconds(10)) {
          CLOG(ERROR, driver_log)
              << name << " The communication interval is too long > 10s  "
              << get_time_fmt_utc(get_now_utc_time()) << " "
              << get_time_fmt_utc(p.value()) << "\n";
          task_run = false;
          return false;
        }
      } else {
        CLOG(WARNING, driver_log)
            << name << " dot not has timestamp, the status may be incorrect ";
      }
      // error
      if (!vdastate.errors.empty()) {
        for (auto &x : vdastate.errors) {
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
          for (auto &x : vdastate.actionstates) {
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
                         [&](const vda5050::state::EdgeState &s) {
                           return s.edge_id == ord.edges.front().edge_id;
                         });
        // 到达点
        auto it_end_point =
            std::find_if(vdastate.nodestates.begin(), vdastate.nodestates.end(),
                         [&](const vda5050::state::NodeState &s) {
                           return s.node_id == (ord.nodes.begin() + 1)->node_id;
                         });
        if (it == vdastate.edgestates.end() &&
            it_end_point == vdastate.nodestates.end()) {
          CLOG(DEBUG, driver_log)
              << name << " " << "move along " << name_ << " ok\n";
          run_ok = true;
        }
      }
      if (!act_end_ok) {
        if (wait_act_ord_end.empty()) {
          act_end_ok = true;
        } else {
          for (auto &x : vdastate.actionstates) {
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
      //  TEST
      // if (run_ok) {
      //   task_run = false;
      //   return true;
      // }

      if (act_start_ok && act_end_ok && run_ok) {
        for (auto &op : after_moved_action) {
          CLOG(INFO, driver_log)
              << "do " << op.location_name << "[" << op.op_name << "]\n";
          std::map<std::string, std::string> act_param;
          for (auto &loc : res->locations) {
            if (loc->name == op.location_name) {
              for (auto &x : loc->properties) {
                act_param[x.first] = x.second;
              }
            }
          }
          act_param["op"] = op.op_name;
          std::string script = op.script;
          if (op.completion_required) {
            // wait_act_ord_start.insert(op.op_name);

            auto ret = run_script(script, act_param);
            if (!ret) {
              CLOG(ERROR, driver_log) << "do " << op.location_name << "["
                                      << op.op_name << "] failed\n";
              return false;
            } else {
              CLOG(INFO, driver_log)
                  << "do " << op.location_name << "[" << op.op_name << "] ok\n";
            }
          } else {
            io_context.post([=] {
              auto ret = run_script(script, act_param);
              CLOG(INFO, driver_log)
                  << "do " << op.location_name << "[" << op.op_name << "] "
                  << (ret == true ? "ok" : "failed") << "\n";
            });
          }
        }
        task_run = false;
        return true;
      }

    } else {
      CLOG_EVERY_N(20, WARNING, driver_log)
          << name << " "
          << "order state has not been updated,wait_for {\'" +
                 current_order->name
          << " -> update_order_id_" << update_vda_order_id
          << "\'}, but now is {\'" << vdastate.order_id
          << " -> update_order_id_" << vdastate.order_update_id << "\'}\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n++;
    CLOG_EVERY_N(50, INFO, driver_log)
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
    const std::shared_ptr<data::order::DriverOrder::Destination> &dest) {
  CLOG(INFO, driver_log) << name << " " << "action execute ["
                         << dest->get_type() << "] at "
                         << dest->destination.lock()->name << "\n";
  order_action_uuid = uuids::uuid();
  return true;
  // task_run = true;
  // if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
  //   CLOG(ERROR, driver_log) << name << " " << "master not online";
  //   task_run = false;
  //   return false;
  // }
  // if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
  //   CLOG(ERROR, driver_log) << name << " " << "vehlicle not online";
  //   task_run = false;
  //   return false;
  // }

  // int ver = static_cast<int>(std::stod(mqtt_cli->version));
  // auto vda_version = "v" + std::to_string(ver);
  // auto prefix = mqtt_cli->interface_name + "/" + vda_version + "/" +
  //               mqtt_cli->manufacturer + "/" + mqtt_cli->serial_number +
  //               "/";
  // auto ord = vda5050::order::VDA5050Order();
  // ord.header_id = send_header_id++;
  // ord.timestamp = get_time_fmt(get_now_utc_time());
  // ord.version = mqtt_cli->version;
  // ord.manufacturer = mqtt_cli->manufacturer;
  // ord.serial_number = mqtt_cli->serial_number;
  // last_step_count = 0;
  // if (now_order_state == nowOrder::BEGIN) {
  //   ord.order_id = dest->get_type() + "-" + uuids::to_string(order_id);
  //   update_vda_order_id++;
  //   ord.order_update_id = update_vda_order_id;
  //   seq_id = (seq_id - 1);
  // } else {
  //   now_order_state = nowOrder::BEGIN;
  //   seq_id = 0;
  //   order_id = get_uuid();
  //   ord.order_id = dest->get_type() + "-" + uuids::to_string(order_id);
  //   update_vda_order_id = 0;
  //   ord.order_update_id = update_vda_order_id;
  // }
  // {
  //   auto node = vda5050::order::Node();
  //   auto act = vda5050::order::Action();
  //   node.node_position = vda5050::order::NodePosition();
  //   node.node_position.value().map_id = map_id;
  //   node.sequence_id = seq_id++;
  //   auto t = resource.lock()->find(dest->destination.lock()->name);
  //   for (auto& pro_ : dest->properties) {
  //     auto param = vda5050::order::ActionParam();
  //     param.key = pro_.first;
  //     param.value = pro_.second;
  //     act.action_parameters.value().push_back(param);
  //   }
  //   if (t.first == allocate::ResourceManager::ResType::Point) {
  //     auto t1 = std::dynamic_pointer_cast<data::model::Point>(
  //         dest->destination.lock());
  //     node.node_id = t1->name;
  //     node.node_position.value().x = t1->position.x() / 1000.0;
  //     node.node_position.value().y = t1->position.y() / 1000.0;
  //     node.node_position.value().theta = t1->vehicle_orientation;
  //     node.node_position->allowed_deviation_xy = dest_deviation_xy;
  //     node.node_position->allowed_deviation_theta = dest_deviation_theta;
  //     act.action_parameters = std::vector<vda5050::order::ActionParam>();
  //     auto param_x = vda5050::order::ActionParam();
  //     param_x.key = "point_x";
  //     param_x.value = std::to_string((double)t1->position.x() / 1000.0);
  //     act.action_parameters->push_back(param_x);
  //     auto param_y = vda5050::order::ActionParam();
  //     param_y.key = "point_y";
  //     param_y.value = std::to_string((double)t1->position.y() / 1000.0);
  //     act.action_parameters->push_back(param_y);
  //   } else if (t.first == allocate::ResourceManager::ResType::Location) {
  //     auto t1 = std::dynamic_pointer_cast<data::model::Location>(
  //         dest->destination.lock());
  //     act.action_parameters = std::vector<vda5050::order::ActionParam>();
  //     node.node_id = t1->link.lock()->name;
  //     node.node_position.value().x = t1->link.lock()->position.x() /
  //     1000.0; node.node_position.value().y = t1->link.lock()->position.y()
  //     / 1000.0; node.node_position.value().theta =
  //     t1->link.lock()->vehicle_orientation;
  //     node.node_position->allowed_deviation_xy = dest_deviation_xy;
  //     node.node_position->allowed_deviation_theta = dest_deviation_theta;
  //     auto param_x = vda5050::order::ActionParam();
  //     param_x.key = "location_x";
  //     param_x.value = (double)t1->position.x() / 1000.0;
  //     act.action_parameters->push_back(param_x);
  //     auto param_y = vda5050::order::ActionParam();
  //     param_y.key = "location_y";
  //     param_y.value = (double)t1->position.y() / 1000.0;
  //     act.action_parameters->push_back(param_y);
  //     std::string op_type = dest->get_type();
  //     std::transform(op_type.begin(), op_type.end(), op_type.begin(),
  //                    ::tolower);
  //     auto it = t1->type.lock()->allowed_ops.find(op_type);
  //     auto params = it->second;
  //     for (auto& param : params) {
  //       auto p_ = vda5050::order::ActionParam();
  //       p_.key = param.first;
  //       p_.value = (std::string)param.second;
  //       act.action_parameters->push_back(p_);
  //     }
  //   } else {
  //     CLOG(ERROR, driver_log) << name << " " << "dest type is err";
  //     return false;
  //   }

  //   node.released = true;
  //   act.action_id = ord.order_id + "-action";
  //   act.action_type = dest->operation;
  //   act.blocking_type = vda5050::order::ActionBlockingType::HARD;
  //   //  TEST
  //   //  node.actions.push_back(act);
  //   ord.nodes.push_back(node);
  // }
  // auto ord_js = ord.to_json();
  // auto opt = jsoncons::json_options{}.precision(15).float_format(
  //     jsoncons::float_chars_format::general);
  // std::string msg_str;
  // ord_js.dump(msg_str, opt);
  // jsoncons::json vaild = jsoncons::json::parse(msg_str, opt);
  // assert(vda5050::order_validate(vaild).empty());
  // auto msg = mqtt::make_message(prefix + "order", msg_str);
  // CLOG(INFO, driver_log) << "send order mqtt msg: "
  //                        << jsoncons::pretty_print(vaild) << "\n";
  // mqtt_cli->mqtt_client->publish(msg)->wait();
  // // // wait
  // int n{0};
  // while (task_run) {
  //   if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
  //     CLOG(ERROR, driver_log) << name << " " << "master not online\n";
  //     task_run = false;
  //     return false;
  //   }
  //   if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
  //     CLOG(ERROR, driver_log) << name << " " << "vehlicle not online\n";
  //     task_run = false;
  //     return false;
  //   }
  //   if (current_order->state == data::order::TransportOrder::State::FAILED)
  //   {
  //     CLOG(ERROR, driver_log) << name << " order failed\n";
  //     task_run = false;
  //     return false;
  //   }
  //   if (vdastate.order_id ==
  //           dest->get_type() + "-" + uuids::to_string(order_id) &&
  //       vdastate.order_update_id == (update_vda_order_id)) {
  //     // error
  //     if (!vdastate.errors.empty()) {
  //       for (auto& x : vdastate.errors) {
  //         if (x.error_level == vda5050::state::ErrorLevel::WARNING) {
  //           CLOG(WARNING, driver_log) << name << " " << x.error_type;
  //         } else {
  //           CLOG(ERROR, driver_log) << name << " " << x.error_type;
  //           task_run = false;
  //           return false;
  //         }
  //       }
  //     }
  //     //
  //     bool all_ok{true};
  //     for (auto& x : vdastate.actionstates) {
  //       if (x.action_id != vdastate.order_id + "-action") {
  //         continue;
  //       }
  //       auto sta = x.action_status;
  //       if (sta == vda5050::state::ActionStatus::FINISHED) {
  //         CLOG(INFO, driver_log)
  //             << name << " " << "action [" << dest->get_type() << "]ok\n ";
  //       } else if (sta == vda5050::state::ActionStatus::FAILED) {
  //         CLOG(INFO, driver_log) << name << " " << "action failed"
  //                                << dest->destination.lock()->name;
  //         task_run = false;
  //         // return true;
  //         return false;
  //       } else if (sta == vda5050::state::ActionStatus::WAITING) {
  //         all_ok = false;
  //       } else if (sta == vda5050::state::ActionStatus::RUNNING) {
  //         all_ok = false;
  //       } else if (sta == vda5050::state::ActionStatus::INITIALIZING) {
  //         all_ok = false;
  //       }
  //     }
  //     if (all_ok) {
  //       CLOG(INFO, driver_log) << name << " " << "all actions ok\n";
  //       task_run = false;
  //       return true;
  //     }
  //   } else {
  //     CLOG_EVERY_N(20, WARNING, driver_log)
  //         << name << " " << "order state has not been updated,needed {\'"
  //         << dest->get_type() + "-" << uuids::to_string(order_id)
  //         << " -> update_ord_id_" << update_vda_order_id
  //         << "\'}, but now is {\'" << vdastate.order_id << "
  //         ->update_ord_id_
  //         "
  //         << vdastate.order_update_id << "}\n";
  //   }
  //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //   n++;
  //   if (n > 2 * 60 * 100) {
  //     CLOG(ERROR, driver_log) << name << " " << "timeout";
  //     task_run = false;
  //     return false;
  //   }
  // }
  // task_run = false;
  // CLOG(WARNING, driver_log) << name << " " << "task cancel";
  // return false;
}
void log_python_err() {
  if (PyErr_Occurred()) {
    CLOG(ERROR, driver_log)
        << "\n-----------------python error-----------------";
    PyObject *ptype, *pvalue, *ptraceback;
    PyErr_Fetch(&ptype, &pvalue, &ptraceback);
    PyErr_NormalizeException(&ptype, &pvalue, &ptraceback);
    if (ptype != nullptr) {
      PyObject *str_exc_type = PyObject_Repr(ptype);
      const char *err_type = PyUnicode_AsUTF8(str_exc_type);
      std::cerr << err_type << std::endl;
      Py_XDECREF(str_exc_type);
    }
    if (pvalue != nullptr) {
      PyObject *str_exc_value = PyObject_Repr(pvalue);
      const char *err_value = PyUnicode_AsUTF8(str_exc_value);
      std::cerr << err_value << std::endl;
      Py_XDECREF(str_exc_value);
    }
    if (ptraceback != nullptr) {
      PyObject *traceback_module = PyImport_ImportModule("traceback");
      PyObject *format_tb_func =
          PyObject_GetAttrString(traceback_module, "format_exception");
      PyObject *traceback_list = PyObject_CallFunctionObjArgs(
          format_tb_func, ptype, pvalue, ptraceback, nullptr);
      PyObject *traceback_str =
          PyUnicode_Join(PyUnicode_FromString(""), traceback_list);
      const char *tb = PyUnicode_AsUTF8(traceback_str);
      std::cerr << std::endl << tb << std::endl;
      Py_XDECREF(traceback_module);
      Py_XDECREF(format_tb_func);
      Py_XDECREF(traceback_list);
      Py_XDECREF(traceback_str);
    }
    std::cerr << "-------------------------------------------\n";
  }
}
bool Rabbit3::run_script(const std::string &path,
                         std::map<std::string, std::string> param) {
  std::unique_lock<std::mutex> lock(python_mutex);
#if WIN32
  const std::string scirpt_save_path{R"(c:\\script)"};
#else
  const std::string scirpt_save_path{R"(/opt/robot/script)"};
#endif
  using namespace ghc;
  filesystem::path p(path);
  if (p.is_absolute()) {
    if (!filesystem::exists(p)) {
      CLOG(ERROR, driver_log) << "file not exist " << path;
      return false;
    }
  }
  std::string script_name = p.stem();
  CLOG(INFO, driver_log) << "ready run script " << script_name << ".py at "
                         << scirpt_save_path << "\n";
  Py_Initialize();
  try {
    PyRun_SimpleString("import sys");
    std::string env = "sys.path.append('" + scirpt_save_path + "')";
    PyRun_SimpleString(env.c_str());
    PyObject *pModule = PyImport_ImportModule(script_name.c_str());
    if (pModule == NULL) {
      CLOG(ERROR, driver_log) << "The import failed, the module was not found "
                                 "or there was an error inside the module"
                              << std::endl;
      log_python_err();
      //    Py_DECREF(pModule);
      Py_Finalize();
      return false;
    }
    pModule = PyImport_ReloadModule(pModule);
    PyObject *pFunc = PyObject_GetAttrString(pModule, "run");
    if (pFunc == nullptr) {
      CLOG(ERROR, driver_log) << "function <run> not found" << std::endl;
      log_python_err();
      Py_DECREF(pModule);
      Py_Finalize();
      return false;
    }
    std::string param_msg{'{'};
    for (auto &item : param) {
      param_msg += "\"" + item.first + "\":\"" + item.second + "\",";
    }
    param_msg.pop_back();
    param_msg += "}";

    CLOG(INFO, driver_log)
        << script_name << " running....\n"
        << "--------------python stdout--------------------\n";
    auto py_state = PyGILState_Ensure();
    PyObject *pReturn = PyObject_CallFunction(pFunc, "s", param_msg.c_str());
    std::cerr << "-----------------------------------------------\n";
    PyGILState_Release(py_state);
    CLOG(INFO, driver_log) << script_name << " run end\n";
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    int ret_code{-10};
    std::string reason{"no return reason"};
    if (pReturn == nullptr) {
      CLOG(ERROR, driver_log)
          << "run result err, return value is null" << std::endl;
      log_python_err();
      Py_DECREF(pFunc);
      Py_DECREF(pModule);
      Py_Finalize();
      return false;
    } else if (!PyDict_Check(pReturn)) {
      CLOG(ERROR, driver_log)
          << "result err: type of return value is not a dict " << std::endl;
      Py_DECREF(pFunc);
      Py_DECREF(pReturn);
      Py_DECREF(pModule);
      Py_Finalize();
      return false;
    }
    while (PyDict_Next(pReturn, &pos, &key, &value)) {
      auto msg = PyUnicode_AsUTF8(key);
      // LOG(INFO) << std::string(msg);
      if (msg == std::string("ret_code")) {
        int ret = -10;
        PyArg_Parse(value, "i", &ret);
        // std::cout << ":" << ret << std::endl;
        ret_code = ret;
      } else if (msg == std::string("reason")) {
        char *msg_ = NULL;
        PyArg_Parse(value, "s", &msg_);
        // std::cout << ":" << msg_ << std::endl;
        reason = std::string(msg_);
      }
    }
    if (ret_code != 0) {
      CLOG(ERROR, driver_log)
          << "run result err,reason: " << reason << std::endl;
      Py_DECREF(pModule);
      Py_Finalize();
      return false;
    } else {
      Py_DECREF(pReturn);
      Py_DECREF(pFunc);
      Py_DECREF(pModule);
      Py_Finalize();
      CLOG(INFO, driver_log) << "run " << script_name << " ok" << std::endl;
      return true;
    }
  } catch (...) {
    CLOG(ERROR, driver_log) << "run is  exception" << std::endl;
    log_python_err();
    Py_Finalize();
    return false;
  }
}

bool Rabbit3::instant_action(
    const std::shared_ptr<data::model::Actions::Action> &act) {
  CLOG(INFO, driver_log) << "instantaction " << act->action_id;
  if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "master not online";
    instant_task_run = false;
    return false;
  }
  if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
    CLOG(ERROR, driver_log) << name << " " << "vehlicle not online";
    instant_task_run = false;
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
  insact->timestamp = get_time_fmt_utc(get_now_utc_time());
  insact->version = mqtt_cli->version;
  auto action = static_cast<vda5050::instantaction::Action *>(act.get());
  insact->actions.push_back(*action);
  auto msg = mqtt::make_message(prefix + "instantAction",
                                insact->to_json().as_string());
  mqtt_cli->mqtt_client->publish(msg)->wait();
  auto id = act->action_id;
  int n{0};
  instant_task_run = true;
  bool ok{false};
  while (instant_task_run) {
    if (mqtt_cli->master_state != vda5050::MasterMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " " << "master not online\n";
      instant_task_run = false;
      return false;
    }
    if (veh_state != vda5050::VehicleMqttStatus::ONLINE) {
      CLOG(ERROR, driver_log) << name << " " << "vehlicle not online\n";
      instant_task_run = false;
      return false;
    }
    for (auto &x : vdastate.actionstates) {
      if (x.action_id == id) {
        if (x.action_status == vda5050::state::ActionStatus::WAITING) {
        } else if (x.action_status == vda5050::state::ActionStatus::RUNNING) {
        } else if (x.action_status ==
                   vda5050::state::ActionStatus::INITIALIZING) {
        } else if (x.action_status == vda5050::state::ActionStatus::FAILED) {
          instant_task_run = false;
          break;
        } else if (x.action_status == vda5050::state::ActionStatus::FINISHED) {
          ok = true;
          instant_task_run = false;
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
  instant_task_run = false;
  return ok;
}

Rabbit3::~Rabbit3() {
  if (mqtt_cli->mqtt_client && mqtt_cli->mqtt_client->is_connected()) {
    mqtt_cli->mqtt_client->disable_callbacks();
    mqtt_cli->mqtt_client->unsubscribe("#");
    mqtt_cli->mqtt_client->disconnect()->wait();
  }
}

}  // namespace kernel::driver