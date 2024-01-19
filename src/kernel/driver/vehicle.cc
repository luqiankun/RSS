
#include "../../../include/kernel/driver/vehicle.hpp"

#include "../../../include/kernel/dispatch/dispatch.hpp"

namespace kernel {
namespace driver {
void Vehicle::execute_action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  strand->post([=] {
    auto op_ret = action(dest);
    if (!op_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = std::chrono::system_clock::now();
    }
    LOG(INFO) << current_order->name << " action : "
              << " ok";
    current_command->vehicle_execute_cb(op_ret);
  });
}
void Vehicle::execute_move(std::shared_ptr<data::order::Step> step) {
  strand->post([=] {
    auto move_ret = move(step);
    if (!move_ret) {
      // 订单失败
      current_order->driverorders[current_order->current_driver_index]->state =
          data::order::DriverOrder::State::FAILED;
      current_order->state = data::order::TransportOrder::State::FAILED;
      current_order->end_time = std::chrono::system_clock::now();
    }
    LOG(INFO) << current_order->name << " move : " << step->name.c_str()
              << " ok";
    current_command->vehicle_execute_cb(move_ret);
  });
}
void Vehicle::run() {
  run_th = std::thread([&] {
#if BOOST_VERSION > 107001
    boost::asio::executor_work_guard<io_service_type::executor_type> work =
        boost::asio::make_work_guard(*ctx);
#else
    boost::asio::io_service::work w(*ctx));
#endif
    try {
      ctx->run();
    } catch (boost::system::error_code& ec) {
      LOG(ERROR) << "client err " << ec.message() << "\n";
    }
    // LOG(INFO) << name << " stop";
  });
}
void Vehicle::close() {
  if (!ctx->stopped()) {
    ctx->stop();
  }
  if (run_th.joinable()) {
    run_th.join();
  }
  current_command.reset();
}
Vehicle::~Vehicle() {
  close();
  LOG(INFO) << name << " close";
}

void Vehicle::plan_route() {
  // TODO  solver
  auto start_planner = current_point;
  std::shared_ptr<data::model::Point> end_planner;
  for (auto& op : current_order->driverorders) {
    auto dest = op->destination->destination.lock();
    auto start_check = dispatcher.lock()->find(dest->name);
    auto destination = dispatcher.lock()->res_to_destination(
        start_check.second, op->destination->operation);
    op->destination = destination;
    bool able{false};
    if (start_check.first == kernel::dispatch::Dispatcher::ResType::Point) {
      end_planner =
          std::dynamic_pointer_cast<data::model::Point>(start_check.second);
    } else if (start_check.first ==
               kernel::dispatch::Dispatcher::ResType::Location) {
      end_planner =
          std::dynamic_pointer_cast<data::model::Location>(start_check.second)
              ->link.lock();
    }
    if (!end_planner) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
      LOG(WARNING) << current_order->name << " can not find obj";
    }
    auto path =
        dispatcher.lock()->planner->find_paths(start_planner, end_planner);
    if (path.empty()) {
      current_order->state = data::order::TransportOrder::State::UNROUTABLE;
      LOG(WARNING) << current_order->name << " can not routable";
    } else {
      auto driverorder = dispatcher.lock()->route_to_driverorder(
          dispatcher.lock()->paths_to_route(path.front()), destination);
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
  // 订单结束了
  current_order->state = data::order::TransportOrder::State::FINISHED;
  LOG(INFO) << current_order->name << " finished";
  current_order->end_time = std::chrono::system_clock::now();
  current_order.reset();
  current_command.reset();
  // 获取新订单
  for (;;) {
    if (orders.empty()) {
      proc_state = ProcState::AWAITING_ORDER;
      state = State::IDLE;
      break;
    } else {
      current_order = orders.front();
      // TODO  solver
      plan_route();
      orders.pop_front();
      if (current_order->state !=
          data::order::TransportOrder::State::BEING_PROCESSED) {
        current_order.reset();
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
  }
}

void Vehicle::command_done() {
  bool ord_shutdown{false};
  if (current_order->state == data::order::TransportOrder::State::WITHDRAWN) {
    // 订单取消
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

void Vehicle::next_command() {
  if (!current_order) {
    if (orders.empty()) {
      return;
    } else {
      current_order = orders.front();
      // TODO  solver
      plan_route();
      orders.pop_front();
    }
  }
  if (!current_order) {
    return;
  }
  current_command = scheduler.lock()->new_command(shared_from_this());
  // run
  scheduler.lock()->add_command(current_command);
}
void Vehicle::receive_task(std::shared_ptr<data::order::TransportOrder> order) {
  LOG(INFO) << name << " receive new order " << order->name;
  if (state == State::ERROR) {  // TODO
  } else if (state == State::UNAVAILABLE) {
    // TODO
  } else if (state == State::IDLE) {
    // TODO
    current_order.reset();
    orders.push_back(order);
    next_command();
    state = State::EXECUTING;
  } else if (state == State::EXECUTING) {
    orders.push_back(order);
  } else if (state == State::CHARGING) {
    orders.push_back(order);
  } else {
    // TODO
  }
}
bool SimVehicle::action(
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  if (dest->operation == data::order::DriverOrder::Destination::OpType::NOP) {
    auto t =
        std::dynamic_pointer_cast<data::model::Point>(dest->destination.lock());
    position.x() = t->pose.x();
    position.y() = t->pose.y();
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
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position.x() = t->link.lock()->pose.x();
    position.y() = t->link.lock()->pose.y();
    current_point = t->link.lock();
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
    return true;
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::UNLOAD) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    current_point = t->link.lock();
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position.x() = t->link.lock()->pose.x();
    position.y() = t->link.lock()->pose.y();
    current_point = t->link.lock();
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
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
    int x_len = end->pose.x() - position.x();
    int y_len = end->pose.y() - position.y();
    for (int i = 0; i < 10; i++) {
      position.x() += x_len / 10;
      position.y() += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x() = end->pose.x();
    position.y() = end->pose.y();
    current_point = end;
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
    return true;
  } else if (step->vehicle_orientation ==
             data::order::Step::Orientation::BACKWARD) {
    auto end = step->path->source_point.lock();
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    int x_len = end->pose.x() - position.x();
    int y_len = end->pose.y() - position.y();
    for (int i = 0; i < 10; i++) {
      position.x() += x_len / 10;
      position.y() += y_len / 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(t / 10));
    }
    position.x() = end->pose.x();
    position.y() = end->pose.y();
    current_point = end;
    LOG(INFO) << name << " now at (" << position.x() << " , " << position.y()
              << ")";
    return true;
  } else {
    return false;
  }
}

void SimVehicle::update() {}

}  // namespace driver
}  // namespace kernel