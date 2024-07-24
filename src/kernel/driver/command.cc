#include "../../../include/kernel/driver/command.hpp"

#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel {
namespace driver {
void Command::run_once() {
  auto veh = vehicle.lock();
  auto scheduler = veh->scheduler.lock();
  auto res = scheduler->resource.lock();
  if (!veh || !scheduler || !res) {
    return;
  }

  if (veh->paused) {
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
  if (state == State::INIT) {
    if (order->state == data::order::TransportOrder::State::WITHDRAWL) {
      state = State::EXECUTED;
    } else {
      state = State::ALLOCATING;
    }
  } else if (state == State::ALLOCATING) {
    if (order->state == data::order::TransportOrder::State::WITHDRAWL) {
      state = State::EXECUTED;
      return;
    }
    auto driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::PRISTINE) {
      driver_order->state = data::order::DriverOrder::State::TRAVELLING;
    }
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto steps = get_step_nopop(driver_order, veh->send_queue_size);
      if (steps.empty()) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
      } else {
        std::vector<std::shared_ptr<TCSResource>> temp;
        veh->claim_resources.clear();
        for (auto& x : steps) {
          std::vector<std::shared_ptr<TCSResource>> step_res;
          bool has_1{false}, has_2{false};
          auto s_p = x->path->source_point.lock();
          auto e_p = x->path->destination_point.lock();
          for (auto& exist : temp) {
            if (exist->name == s_p->name) {
              has_1 = true;
            }
            if (exist->name == e_p->name) {
              has_2 = true;
            }
          }
          if (!has_1) {
            temp.push_back(x->path->source_point.lock());
            step_res.push_back(x->path->source_point.lock());
          }
          if (!has_2) {
            temp.push_back(x->path->destination_point.lock());
            step_res.push_back(x->path->destination_point.lock());
          }
          temp.push_back(x->path);
          step_res.push_back(x->path);
          for (auto x = step_res.begin(); x != step_res.end();) {
            bool has{false};
            for (auto& allocate_list : veh->allocated_resources) {
              for (auto& temo_res : allocate_list) {
                if (temo_res == *x) {
                  has = true;
                  break;
                }
              }
            }
            if (has) {
              x = step_res.erase(x);
            } else {
              x++;
            }
          }
          // allocate
          veh->claim_resources.push_back(
              std::unordered_set<std::shared_ptr<TCSResource>>{step_res.begin(),
                                                               step_res.end()});
          if (!res->allocate(step_res, veh)) {
            // future_claim
            for (auto& x : step_res) {
              bool has{false};
              for (auto& t : veh->allocated_resources) {
                for (auto& c : t) {
                  if (c == x) {
                    has = true;
                  }
                }
              }
              if (!has) {
                veh->future_allocate_resources.insert(x);
              }
            }
            //
            return;
          }
        }
        // future_claim
        for (auto& x : get_future(driver_order)) {
          bool has{false};
          for (auto& t : veh->allocated_resources) {
            for (auto& c : t) {
              if (c == x) {
                has = true;
              }
            }
          }
          if (!has) {
            veh->future_allocate_resources.insert(x);
          }
        }
        state = State::ALLOCATED;
      }
    }
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      std::vector<std::shared_ptr<TCSResource>> temp;
      temp.push_back(dest->destination.lock());
      veh->claim_resources.clear();
      veh->claim_resources.push_back(
          std::unordered_set<std::shared_ptr<TCSResource>>{temp.begin(),
                                                           temp.end()});
      for (auto& r : veh->allocated_resources) {
        for (auto& x : r) {
          if (x == dest->destination.lock()) {
            state = State::ALLOCATED;
            return;
          }
        }
      }
      if (res->allocate(temp, veh)) {
        state = State::ALLOCATED;
      }
    }
  } else if (state == State::ALLOCATED) {
    if (order->state == data::order::TransportOrder::State::WITHDRAWL) {
      state = State::EXECUTED;
      return;
    }
    // execute
    auto& driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto steps = get_step(driver_order, veh->send_queue_size);
      if (steps.empty()) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
      } else {
        move(steps);
        state = State::EXECUTING;
      }

    } else if (driver_order->state ==
               data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      action(dest);
      state = State::EXECUTING;

    } else {
      state = State::END;
      return;
    }
  } else if (state == State::EXECUTING) {
    // wait  do nothing
  } else if (state == State::EXECUTED) {
    auto& driver_order = order->driverorders[order->current_driver_index];
    // free
    std::stringstream ss;
    ss << veh->name << " free ";
    std::vector<std::shared_ptr<TCSResource>> temp;
    LOG(INFO) << veh->allocated_resources.size() << "\n";
    for (auto& a : veh->allocated_resources) {
      for (auto& x : a) {
        if (x != veh->current_point &&
            x != get_dest(driver_order)->destination.lock()) {
          temp.push_back(x);
          ss << x->name << " ";
        }
      }
    }
    if (!res->free(temp, veh)) {
      return;
    }
    state = State::END;
    CLOG_IF(!ss.str().empty(), INFO, driver_log) << ss.str() << "\n";
  } else if (state == State::END) {
    veh->command_done();
    state = State::DISPOSABLE;
  }
}
void Command::vehicle_execute_cb(bool ret) {
  auto veh = vehicle.lock();
  auto scheduler = veh->scheduler.lock();
  auto res = scheduler->resource.lock();
  if (!veh || !scheduler || !res) {
    state = State::DISPOSABLE;
  }
  if (state == State::EXECUTING) {
    auto& driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      if (ret) {
        driver_order->state = data::order::DriverOrder::State::FINISHED;
      } else {
        driver_order->state = data::order::DriverOrder::State::FAILED;
      }
      veh->claim_resources.clear();
    }
    state = State::EXECUTED;
  }
}
std::shared_ptr<data::order::DriverOrder::Destination> Command::get_dest(
    std::shared_ptr<data::order::DriverOrder> order) {
  return order->destination;
}
std::vector<std::shared_ptr<data::order::Step>> Command::get_step(
    std::shared_ptr<data::order::DriverOrder> order, uint32_t size) {
  if (order->route->steps.empty()) {
    order->route->current_steps.clear();
    return std::vector<std::shared_ptr<data::order::Step>>();
  }
  order->route->current_steps.clear();
  auto step = order->route->steps.front();
  order->route->current_steps.push_back(step);
  auto res = std::vector<std::shared_ptr<data::order::Step>>();
  auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.front());
    order->route->current_steps.push_back(order->route->steps.front());
    order->route->steps.pop_front();
  }
  return res;
}
std::vector<std::shared_ptr<data::order::Step>> Command::get_step_nopop(
    std::shared_ptr<data::order::DriverOrder> order, uint32_t size) {
  if (order->route->steps.empty()) {
    order->route->current_steps.clear();
    return std::vector<std::shared_ptr<data::order::Step>>();
  }
  order->route->current_steps.clear();
  auto step = order->route->steps.front();
  order->route->current_steps.push_back(step);
  auto res = std::vector<std::shared_ptr<data::order::Step>>();
  auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.at(i));
    order->route->current_steps.push_back(order->route->steps.front());
  }
  return res;
}
std::vector<std::shared_ptr<TCSResource>> Command::get_future(
    std::shared_ptr<data::order::DriverOrder> order) {
  auto veh = vehicle.lock();
  std::vector<std::shared_ptr<TCSResource>> res;
  if (order->route->steps.size() > veh->send_queue_size) {
    auto next = *(order->route->steps.begin() + veh->send_queue_size);
    auto path = next->path;
    auto beg = next->path->destination_point.lock();
    auto end = next->path->source_point.lock();
    for (auto& r : veh->allocated_resources) {
      for (auto& x : r) {
        if (x == beg) {
          beg = nullptr;
        }
        if (x == end) {
          end = nullptr;
        }
        if (x == path) {
          path = nullptr;
        }
      }
    }
    if (beg) {
      res.push_back(beg);
    }
    if (end) {
      res.push_back(end);
    }
    if (path) {
      res.push_back(path);
    }
  }
  return res;
}

}  // namespace driver
}  // namespace kernel