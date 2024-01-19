#include "../../../include/kernel/driver/command.hpp"

#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel {
namespace driver {
void Command::run_once() {
  if (!vehicle.lock()) {
    return;
  }
  for (auto& x : order->dependencies) {
    auto dep = x.lock();
    if (dep) {
      if (dep->state != data::order::TransportOrder::State::FINISHED) {
        return;
      }
    }
  }
  if (order->state != data::order::TransportOrder::State::BEING_PROCESSED) {
    // 无需处理
    state = State::END;
    vehicle.lock()->command_done();
    return;
  }
  if (state == State::INIT) {
    state = State::CLAIMING;
  } else if (state == State::CLAIMING) {
    auto driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::PRISTINE) {
      driver_order->state = data::order::DriverOrder::State::TRAVELLING;
    }
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto step = get_step_nopop(driver_order);
      if (!step) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
      } else {
        std::vector<std::shared_ptr<TCSResource>> temp;
        temp.push_back(step->path->source_point.lock());
        temp.push_back(step->path->destination_point.lock());
        if (scheduler.lock()->res.lock()->claim(temp, vehicle.lock())) {
          // TODO 添加future_claim
          for (auto& x : get_future(driver_order)) {
            bool has{false};
            for (auto& c : vehicle.lock()->claim_resources) {
              if (c == x) {
                has = true;
              }
            }
            if (!has) {
              vehicle.lock()->future_claim_resources.insert(x);
            }
          }
          state = State::CLAIMED;
        }
      }
    }
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      std::vector<std::shared_ptr<TCSResource>> temp;
      temp.push_back(dest->destination.lock());
      if (scheduler.lock()->res.lock()->claim(temp, vehicle.lock())) {
        state = State::CLAIMED;
      }
    }
  } else if (state == State::CLAIMED) {
    // execute
    auto& driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto step = get_step(driver_order);
      if (!step) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
      } else {
        vehicle.lock()->execute_move(step);
        state = State::EXECUTING;
      }

    } else if (driver_order->state ==
               data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      vehicle.lock()->execute_action(dest);
      state = State::EXECUTING;

    } else {
      state = State::END;
      return;
    }
  } else if (state == State::EXECUTING) {
    // wait  do nothing
  } else if (state == State::EXECUTED) {
    // unclaim
    std::stringstream ss;
    ss << vehicle.lock()->name << " unclaim ";
    std::vector<std::shared_ptr<TCSResource>> temp;
    for (auto& x : vehicle.lock()->claim_resources) {
      if (x != vehicle.lock()->current_point) {
        temp.push_back(x);
        ss << x->name << " ";
      }
    }
    if (scheduler.lock()->res.lock()->unclaim(temp, vehicle.lock())) {
      state = State::END;
      LOG(INFO) << ss.str();
    }
  } else if (state == State::END) {
    vehicle.lock()->command_done();
    state = State::DISPOSABLE;
  }
}
void Command::vehicle_execute_cb(bool ret) {
  if (state == State::EXECUTING) {
    auto& driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      if (ret) {
        driver_order->state = data::order::DriverOrder::State::FINISHED;
      } else {
        driver_order->state = data::order::DriverOrder::State::FAILED;
      }
    }

    state = State::EXECUTED;
  }
}
std::shared_ptr<data::order::DriverOrder::Destination> Command::get_dest(
    std::shared_ptr<data::order::DriverOrder> order) {
  return order->destination;
}
std::shared_ptr<data::order::Step> Command::get_step(
    std::shared_ptr<data::order::DriverOrder> order) {
  if (order->route->steps.empty()) {
    order->route->current_step.reset();
    return nullptr;
  }
  auto res = order->route->steps.front();
  order->route->current_step = res;
  order->route->steps.pop_front();
  return res;
}
std::shared_ptr<data::order::Step> Command::get_step_nopop(
    std::shared_ptr<data::order::DriverOrder> order) {
  if (order->route->steps.empty()) {
    order->route->current_step.reset();
    return nullptr;
  }
  auto res = order->route->steps.front();
  order->route->current_step = res;
  return res;
}
std::vector<std::shared_ptr<TCSResource>> Command::get_future(
    std::shared_ptr<data::order::DriverOrder> order) {
  std::vector<std::shared_ptr<TCSResource>> res;
  if (order->route->steps.size() > 1) {
    auto next = *(order->route->steps.begin() + 1);
    auto beg = next->path->destination_point.lock();
    auto end = next->path->source_point.lock();
    for (auto& x : vehicle.lock()->claim_resources) {
      if (x == beg) {
        beg = nullptr;
      }
      if (x == end) {
        end = nullptr;
      }
    }
    if (beg) {
      res.push_back(beg);
    }
    if (end) {
      res.push_back(end);
    }
  }
  return res;
}

}  // namespace driver
}  // namespace kernel