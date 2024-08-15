#include "../../../include/kernel/driver/command.hpp"

#include "../../../include/kernel/driver/vehicle.hpp"
#define assert_valid                                                   \
  auto veh = vehicle.lock();                                           \
  auto scheduler = veh->scheduler.lock();                              \
  if (!veh || !scheduler) {                                            \
    state = State::DISPOSABLE;                                         \
    return;                                                            \
  }                                                                    \
  auto res = scheduler->resource.lock();                               \
  if (!res) {                                                          \
    state = State::DISPOSABLE;                                         \
    return;                                                            \
  }                                                                    \
  if (veh->paused) {                                                   \
    return;                                                            \
  }                                                                    \
  if (order->state == data::order::TransportOrder::State::WITHDRAWL) { \
    state = State::EXECUTED;                                           \
    return;                                                            \
  }
namespace kernel {
namespace driver {
// namespace driver
std::vector<allocate::TCSResourcePtr> Command::get_next_allocate_res(
    allocate::DriverOrderPtr driver_order, std::shared_ptr<Vehicle> veh) {
  auto steps = get_step_nopop(driver_order, veh->send_queue_size);
  std::vector<std::shared_ptr<RSSResource>> temp;
  for (auto& x : steps) {
    std::vector<std::shared_ptr<RSSResource>> step_res;
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
    for (auto x_res = step_res.begin(); x_res != step_res.end();) {
      bool has{false};
      for (auto& allocate_list : veh->allocated_resources) {
        if (std::any_of(allocate_list.begin(), allocate_list.end(),
                        [&](auto& x) { return x == *x_res; })) {
          has = true;
        }
      }
      if (has) {
        x_res = step_res.erase(x_res);
      } else {
        x_res++;
      }
    }
  }
  return temp;
}
Command::Command(const std::string& n) : RSSObject(n) {
  cbs[State::INIT] = [&] {
    assert_valid;
    state = State::ALLOCATING;
  };
  cbs[State::ALLOCATING] = [&] {
    assert_valid;
    auto driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::PRISTINE) {
      driver_order->state = data::order::DriverOrder::State::TRAVELLING;
    }
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto steps = get_step_nopop(driver_order, veh->send_queue_size);
      if (steps.empty()) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
        // LOG(WARNING) << "-------------------------";
      } else {
        std::vector<std::shared_ptr<RSSResource>> temp;
        veh->claim_resources.clear();
        for (auto& x : steps) {
          std::vector<std::shared_ptr<RSSResource>> step_res;
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
          for (auto x_res = step_res.begin(); x_res != step_res.end();) {
            bool has{false};
            for (auto& allocate_list : veh->allocated_resources) {
              if (std::any_of(allocate_list.begin(), allocate_list.end(),
                              [&](auto& v) { return v == *x_res; })) {
                has = true;
              }
            }
            if (has) {
              x_res = step_res.erase(x_res);
            } else {
              x_res++;
            }
          }
          // allocate
          veh->claim_resources.push_back(
              std::unordered_set<std::shared_ptr<RSSResource>>{step_res.begin(),
                                                               step_res.end()});
          if (!res->allocate(step_res, veh)) {
            // future_claim
            for (auto& x_res : step_res) {
              bool has{false};
              for (auto& t : veh->allocated_resources) {
                if (std::any_of(t.begin(), t.end(),
                                [&](auto& v) { return v == x_res; })) {
                  has = true;
                  break;
                }
              }
              if (!has) {
                veh->future_allocate_resources.insert(x_res);
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
            if (std::any_of(t.begin(), t.end(),
                            [&](auto& v) { return v == x; })) {
              has = true;
              break;
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
      std::vector<std::shared_ptr<RSSResource>> temp;
      temp.push_back(dest->destination.lock());
      veh->claim_resources.clear();
      veh->claim_resources.push_back(
          std::unordered_set<std::shared_ptr<RSSResource>>{temp.begin(),
                                                           temp.end()});
      for (auto& r : veh->allocated_resources) {
        if (std::any_of(r.begin(), r.end(), [&](auto& v) {
              return v == dest->destination.lock();
            })) {
          state = State::ALLOCATED;
          return;
        }
      }
      if (res->allocate(temp, veh)) {
        state = State::ALLOCATED;
      }
    }
  };
  cbs[State::ALLOCATED] = [&] {
    assert_valid;
    // execute
    auto& driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto steps = get_step(driver_order, veh->send_queue_size);
      if (steps.empty()) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
        // LOG(WARNING) << "-------------------------";
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
  };
  cbs[State::EXECUTING] = [&] { assert_valid; };
  cbs[State::EXECUTED] = [&] {
    auto veh = vehicle.lock();
    auto scheduler = veh->scheduler.lock();
    auto res = scheduler->resource.lock();
    if (!veh || !scheduler || !res) {
      state = State::DISPOSABLE;
      return;
    }
    if (veh->paused) {
      return;
    }
    auto& driver_order = order->driverorders[order->current_driver_index];
    // free
    // 获取下次要分配的资源
    std::stringstream ss;
    auto next_res = get_next_allocate_res(driver_order, veh);
    std::vector<std::shared_ptr<RSSResource>> temp;
    for (auto& a : veh->allocated_resources) {
      for (auto& x : a) {
        if (x == veh->current_point) {
          continue;
        }
        auto dest_ = get_dest(driver_order)->operation;
        if (dest_ == data::model::Actions::OpType::CHARGE ||
            dest_ == data::model::Actions::OpType::PARK) {
          if (x == get_dest(driver_order)->destination.lock()) {
            continue;
          }
        }
        auto it = std::find(next_res.begin(), next_res.end(), x);
        if (it == next_res.end()) {
          temp.push_back(x);
          ss << x->name << " ";
        }
      }
    }
    if (!res->free(temp, veh)) {
      return;
    }
    state = State::END;
    CLOG_IF(!ss.str().empty(), INFO, driver_log)
        << veh->name << " free { " << ss.str() << "}\n";
  };
  cbs[State::END] = [&] {
    assert_valid;
    veh->command_done();
    state = State::DISPOSABLE;
  };
}
void Command::run_once() { cbs[state](); }
void Command::vehicle_execute_cb(bool ret) {
  assert_valid;
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
DestPtr Command::get_dest(DriverOrderPtr order) { return order->destination; }
std::vector<StepPtr> Command::get_step(DriverOrderPtr order, uint32_t size) {
  if (order->route->steps.empty()) {
    order->route->current_step.reset();
    return std::vector<StepPtr>();
  }
  order->route->current_step = order->route->steps.front();
  auto res = std::vector<StepPtr>();
  auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.at(i));
  }
  order->route->steps.pop_front();
  return res;
}
std::vector<StepPtr> Command::get_step_nopop(DriverOrderPtr order,
                                             uint32_t size) {
  if (order->route->steps.empty() || !order) {
    order->route->current_step.reset();
    return std::vector<StepPtr>();
  }
  // auto step = order->route->steps.front();
  // order->route->current_steps.push_back(step);
  auto res = std::vector<StepPtr>();
  auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.at(i));
    // order->route->current_steps.push_back(order->route->steps.at(i));
  }
  return res;
}
std::vector<std::shared_ptr<RSSResource>> Command::get_future(
    DriverOrderPtr order) {
  auto veh = vehicle.lock();
  std::vector<std::shared_ptr<RSSResource>> res;
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