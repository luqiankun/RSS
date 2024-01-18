
#include "../../../include/kernel/driver/vehicle.hpp"

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
    if (notify_result) {
      notify_result(op_ret);
    }
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
    if (notify_result) {
      notify_result(move_ret);
    }
  });
}
void Vehicle::run() {
  run_th = std::thread([&] {
#if BOOST_VERSION > 107001
    boost::asio::executor_work_guard<io_service_type::executor_type> work =
        boost::asio::make_work_guard(ctx);
#else
    boost::asio::io_service::work w(*(io_context.get()));
#endif
    io_service_type::strand st(ctx);
    strand = std::shared_ptr<io_service_type::strand>(&st);
    try {
      ctx.run();
    } catch (boost::system::error_code& ec) {
      LOG(ERROR) << "client err " << ec.message() << "\n";
    }
  });
}

Vehicle::~Vehicle() { ctx.stop(); }

void Vehicle::command_done() {
  // 完成step or action
  auto dr = (current_order->driverorders[current_order->current_driver_index]);
  if (dr->state == data::order::DriverOrder::State::FINISHED ||
      dr->state == data::order::DriverOrder::State::FAILED) {
    current_order->current_driver_index += 1;
  }
  if (current_order->driverorders.size() <=
      current_order->current_driver_index) {
    // 订单结束了
    current_order->state = data::order::TransportOrder::State::FINISHED;
    LOG(INFO) << current_order->name << " finished";
    current_order->finished_time = std::chrono::system_clock::now();
    current_order = nullptr;
    current_command = nullptr;
    // 获取新订单
    for (;;) {
      if (orders.empty()) {
        proc_state = ProcState::AWAITING_ORDER;
        state = State::IDLE;
        break;
      } else {
        current_order = orders.front();
        orders.pop_front();
        if (current_order->state !=
            data::order::TransportOrder::State::DISPATCHABLE) {
          current_order = nullptr;
        } else {
          break;
        }
      }
    }
    if (current_order) {
      current_order->state =
          data::order::TransportOrder::State::BEING_PROCESSED;
      state = State::EXECUTING;
      proc_state = ProcState::PROCESSING_ORDER;
      next_command();
    } else {
      proc_state = ProcState::AWAITING_ORDER;
      state = State::IDLE;
    }
    //
  } else {
    // 继续执行订单
    next_command();
    state = State::EXECUTING;
    proc_state = ProcState::PROCESSING_ORDER;
  }
}

void Vehicle::next_command() {
  current_command = scheduler->new_command(shared_from_this());
  // run
  scheduler->add_command(current_command);
}
void Vehicle::receive_task(std::shared_ptr<data::order::TransportOrder> order) {
  LOG(INFO) << name << " receive new order " << order->name;
  if (state == State::ERROR) {  // TODO
  } else if (state == State::UNAVAILABLE) {
    // TODO
  } else if (state == State::IDLE) {
    // TODO
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
    LOG(INFO) << "action |";
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    position.x() = t->link.lock()->pose.x();
    position.y() = t->link.lock()->pose.y();
    current_point = t->link.lock();
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
  } else if (dest->operation ==
             data::order::DriverOrder::Destination::OpType::PARK) {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t = std::dynamic_pointer_cast<data::model::Location>(
        dest->destination.lock());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    position = t->position;
    return true;
  }
  return true;
}

bool SimVehicle::move(std::shared_ptr<data::order::Step> step) {
  LOG(INFO) << "move |";
  if (!step->path) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
    return true;
  }
  if (step->vehicle_orientation == data::order::Step::Orientation::FORWARD) {
    auto end = step->path->destination_point.lock();
    size_t t = step->path->length * 1000 / max_vel / rate;  // ms
    std::this_thread::sleep_for(std::chrono::milliseconds(t));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(t));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(step->wait_time / rate));
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