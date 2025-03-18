#include "../../../include/kernel/driver/command.hpp"

#include "../../../include/kernel/dispatch/dispatch.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"

#define assert_valid                                                   \
  auto veh = vehicle.lock();                                           \
  auto scheduler = veh->scheduler.lock();                              \
  auto cur_ord = order;                                                \
  if (!veh || !scheduler || !cur_ord) {                                \
    state = State::DISPOSABLE;                                         \
    return;                                                            \
  }                                                                    \
  std::unique_lock<std::shared_mutex> lock(order->mutex);              \
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
namespace kernel::driver {
std::shared_ptr<Vehicle> veh_swap_conflict(std::shared_ptr<Vehicle> v) {
  try {
    if (!v->current_order) {
      return nullptr;
    }
    auto disptcher = v->dispatcher.lock();
    auto vehs = disptcher->vehicles;
    for (auto &x : vehs) {
      if (x != v && x->state == Vehicle::State::EXECUTING) {
        std::deque<std::shared_ptr<data::order::Step>> left_step(
            v->current_order
                ->driverorders[v->current_order->current_driver_index]
                ->route->steps);
        if (v->current_order
                ->driverorders[v->current_order->current_driver_index]
                ->route->current_step) {
          left_step.push_front(
              v->current_order
                  ->driverorders[v->current_order->current_driver_index]
                  ->route->current_step);
        }
        if (!x->current_order) {
          return nullptr;
        }
        std::unique_lock<std::shared_mutex> lock(x->current_order->mutex);
        std::deque<std::shared_ptr<data::order::Step>> right_step(
            x->current_order
                ->driverorders[x->current_order->current_driver_index]
                ->route->steps);
        if (x->current_order
                ->driverorders[x->current_order->current_driver_index]
                ->route->current_step) {
          right_step.push_front(
              x->current_order
                  ->driverorders[x->current_order->current_driver_index]
                  ->route->current_step);
        }
        lock.unlock();
        float cur_cost = 0;  // mm单位
        const float kLen = 2;
        for (auto &cur_s : left_step) {
          float v_cost = 0;
          for (auto &v_s : right_step) {
            if (cur_s->path == v_s->path &&
                cur_s->vehicle_orientation != v_s->vehicle_orientation) {
              if (fabs(cur_cost - v_cost) < kLen * (v_s->path->length)) {
                LOG(DEBUG) << "swap conflict " << v->name << " " << x->name;
                return x;
              }
            }
            v_cost += v_s->path->length;
          }
          cur_cost += cur_s->path->length;
        }
      }
    }
    return nullptr;
  } catch (...) {
    return nullptr;
  }
}
bool veh_conflict(std::shared_ptr<Vehicle> v) {
  auto driver_order =
      v->current_order->driverorders[v->current_order->current_driver_index];
  if (driver_order->route->steps.empty()) {
    return false;
  }
  std::deque<std::shared_ptr<data::order::Step>> steps =
      driver_order->route->steps;
  for (auto &x : steps) {
    auto st_owner = x->path->source_point.lock()->owner.lock();
    auto ed_owner = x->path->destination_point.lock()->owner.lock();
    if (st_owner && st_owner != v) {
      auto st_veh = std::dynamic_pointer_cast<Vehicle>(st_owner);
      if (st_veh->state == Vehicle::State::IDLE) {
        LOG(DEBUG) << "swap conflict " << v->name << " " << st_veh->name;
        return true;
      }
    }
    if (ed_owner && ed_owner != v) {
      auto ed_veh = std::dynamic_pointer_cast<Vehicle>(ed_owner);
      if (ed_veh->state == Vehicle::State::IDLE) {
        LOG(DEBUG) << "swap conflict " << v->name << " " << ed_veh->name;
        return true;
      }
    }
  }
  return false;
}
// namespace driver
std::vector<allocate::TCSResourcePtr> Command::get_next_allocate_res(
    const allocate::DriverOrderPtr &driver_order,
    const std::shared_ptr<Vehicle> &veh) {
  const auto steps = get_step_nopop(driver_order, veh->send_queue_size);
  std::vector<std::shared_ptr<RSSResource>> temp;
  for (auto &x : steps) {
    std::vector<std::shared_ptr<RSSResource>> step_res;
    bool has_1{false}, has_2{false};
    const auto s_p = x->path->source_point.lock();
    const auto e_p = x->path->destination_point.lock();
    for (const auto &exist : temp) {
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
      for (auto &allocate_list : veh->allocated_resources) {
        if (std::any_of(allocate_list.begin(), allocate_list.end(),
                        [&](auto &x_) { return x_ == *x_res; })) {
          has = true;
        }
      }
      if (has) {
        x_res = step_res.erase(x_res);
      } else {
        ++x_res;
      }
    }
  }
  return temp;
}
Command::Command(const std::string &n) : RSSObject(n) {
  cbs[(int)State::INIT] = [&] {
    assert_valid;
    state = State::ALLOCATING;
    // auto driver_order = order->driverorders[order->current_driver_index];
  };
  cbs[(int)State::REDISBUTE] = [&] {
    assert_valid;
    lock.unlock();
    veh->orderpool.lock()->redistribute(order);
    state = Command::State::DISPOSABLE;
  };
  cbs[(int)State::ALLOCATING] = [&] {
    assert_valid;
    // 冲突判断
    if (veh_conflict(veh)) {
      LOG(DEBUG) << veh->name << "redistribute at " << veh->last_point->name
                 << " " << veh->current_point->name;
      lock.unlock();
      veh->redistribute_cur_order();
      return;
    }
    auto con_veh = veh_swap_conflict(veh);
    if (con_veh) {
      // auto ord1 = con_veh->redistribute_cur_order();
      lock.unlock();
      auto ord2 = veh->redistribute_cur_order();
      // veh->orderpool.lock()->redistribute(ord1);
      // veh->orderpool.lock()->redistribute(ord2);
      LOG(DEBUG) << "redistribute " << veh->name << " " << con_veh->name;
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
        veh->last_step.clear();
        veh->future_step.clear();
        // LOG(WARNING) << "-------------------------";
      } else {
        veh->future_step = steps;
        std::vector<std::shared_ptr<RSSResource>> temp;
        for (auto &x : steps) {
          std::vector<std::shared_ptr<RSSResource>> step_res;
          bool has_1{false}, has_2{false};
          auto s_p = x->path->source_point.lock();
          auto e_p = x->path->destination_point.lock();
          for (auto &exist : temp) {
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
            std::shared_lock<std::shared_mutex> lock(veh->res_mut);
            for (auto &allocate_list : veh->allocated_resources) {
              if (std::any_of(allocate_list.begin(), allocate_list.end(),
                              [&](auto &v) { return v == *x_res; })) {
                has = true;
              }
            }
            lock.unlock();
            if (has) {
              x_res = step_res.erase(x_res);
            } else {
              ++x_res;
            }
          }
          // allocate
          if (!res->allocate(step_res, veh)) {
            // future_claim
            for (auto &x_res : step_res) {
              bool has{false};
              std::shared_lock<std::shared_mutex> lock(veh->res_mut);
              for (auto &t : veh->allocated_resources) {
                if (std::any_of(t.begin(), t.end(),
                                [&](auto &v) { return v == x_res; })) {
                  has = true;
                  break;
                }
              }
              if (!has) {
                veh->future_allocate_resources.insert(x_res);
              }
              lock.unlock();
            }
            //
            return;
          }
        }
        // future_claim
        for (auto &x : get_future(driver_order)) {
          bool has{false};
          std::shared_lock<std::shared_mutex> lock(veh->res_mut);
          for (auto &t : veh->allocated_resources) {
            if (std::any_of(t.begin(), t.end(),
                            [&](auto &v) { return v == x; })) {
              has = true;
              break;
            }
          }
          if (!has) {
            veh->future_allocate_resources.insert(x);
          }
          lock.unlock();
        }
        veh->last_step = steps;
        state = State::ALLOCATED;
      }
    }
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      std::vector<std::shared_ptr<RSSResource>> temp;
      temp.push_back(dest->destination.lock());
      std::shared_lock<std::shared_mutex> lock(veh->res_mut);
      for (auto &r : veh->allocated_resources) {
        if (std::any_of(r.begin(), r.end(), [&](auto &v) {
              return v == dest->destination.lock();
            })) {
          state = State::ALLOCATED;
          return;
        }
      }
      lock.unlock();
      if (res->allocate(temp, veh)) {
        state = State::ALLOCATED;
      }
    }
  };
  cbs[(int)State::ALLOCATED] = [&] {
    assert_valid;
    // execute
    auto &driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::TRAVELLING) {
      auto steps = get_step(driver_order, veh->send_queue_size);
      if (steps.empty()) {
        driver_order->state = data::order::DriverOrder::State::OPERATING;
        // LOG(WARNING) << "-------------------------";
      } else {
        LOG(DEBUG) << veh->name << "{" << int(state) << "} move \n";
        state = State::EXECUTING;
        move(steps);
      }

    } else if (driver_order->state ==
               data::order::DriverOrder::State::OPERATING) {
      auto dest = get_dest(driver_order);
      LOG(DEBUG) << veh->name << " action \n";
      state = State::EXECUTING;
      action(dest);
    } else {
      state = State::END;
      return;
    }
  };
  cbs[(int)State::EXECUTING] = [&] {
    auto veh = vehicle.lock();
    auto scheduler = veh->scheduler.lock();
    auto cur_ord = order;
    if (!veh || !scheduler || !cur_ord) {
      state = State ::DISPOSABLE;
      return;
    }
  };
  cbs[(int)State::EXECUTED] = [&] {
    auto veh = vehicle.lock();
    auto scheduler = veh->scheduler.lock();
    auto res = scheduler->resource.lock();
    if (!veh || !scheduler || !res) {
      LOG(ERROR) << "command drop\n";
      state = State::DISPOSABLE;
      return;
    }
    if (veh->paused) {
      return;
    }
    auto &driver_order = order->driverorders[order->current_driver_index];
    // free
    // 获取下次要分配的资源
    std::stringstream ss;
    auto next_res = get_next_allocate_res(driver_order, veh);
    if (order->state == data::order::TransportOrder::State::WITHDRAWL ||
        order->state == data::order::TransportOrder::State::FAILED ||
        order->state == data::order::TransportOrder::State::FINISHED) {
      next_res.clear();
    }
    std::vector<std::shared_ptr<RSSResource>> temp;
    std::shared_lock<std::shared_mutex> lock(veh->res_mut);
    for (auto &a : veh->allocated_resources) {
      for (auto &x : a) {
        if (order->state != data::order::TransportOrder::State::WITHDRAWL ||
            order->state != data::order::TransportOrder::State::FAILED) {
          if (x == veh->current_point) {
            continue;
          }
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
    lock.unlock();
    if (!res->free(temp, veh)) {
      return;
    }
    state = State::END;
    CLOG_IF(!ss.str().empty(), DEBUG, driver_log)
        << veh->name << " free { " << ss.str() << "}\n";
  };
  cbs[(int)State::END] = [&] {
    auto veh = vehicle.lock();
    if (!veh) {
      state = State::DISPOSABLE;
      return;
    }
    if (veh->paused) {
      return;
    }
    veh->command_done();
    state = State::DISPOSABLE;
  };
}
void Command::run_once() { cbs[(int)state](); }
Command::~Command() { CLOG(DEBUG, driver_log) << name << " drop\n"; }
void Command::vehicle_execute_cb(bool ret) {
  auto veh = vehicle.lock();
  auto scheduler = veh->scheduler.lock();
  auto cur_ord = order;
  if (!veh || !scheduler || !cur_ord) {
    state = State ::DISPOSABLE;
    return;
  }
  auto res = scheduler->resource.lock();
  if (!res) {
    state = State ::DISPOSABLE;
    return;
  }
  if (order->state == data ::order ::TransportOrder ::State ::WITHDRAWL) {
    state = State ::EXECUTED;
    return;
  };
  if (state == State::EXECUTING) {
    auto &driver_order = order->driverorders[order->current_driver_index];
    if (driver_order->state == data::order::DriverOrder::State::OPERATING) {
      if (ret) {
        driver_order->state = data::order::DriverOrder::State::FINISHED;
      } else {
        driver_order->state = data::order::DriverOrder::State::FAILED;
      }
    }
    state = State::EXECUTED;
  } else {
    LOG(FATAL) << "test";
  }
  CLOG(DEBUG, driver_log) << veh->name << "  " << (int)state << " execute cb\n";
}
DestPtr Command::get_dest(const DriverOrderPtr &order) {
  return order->destination;
}
std::vector<StepPtr> Command::get_step(const DriverOrderPtr &order,
                                       uint32_t size) {
  if (order->route->steps.empty()) {
    order->route->current_step.reset();
    return {};
  }
  order->route->current_step = order->route->steps.front();
  auto res = std::vector<StepPtr>();
  const auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.at(i));
    bool single_step = false;
    if (i + 1 < len) {
      for (auto act : order->route->steps.at(i + 1)->path->per_acts.acts) {
        if (act.completion_required) {
          single_step = true;
          break;
        }
      }
    }
    if (single_step) break;
  }
  order->route->steps.pop_front();
  return res;
}
std::vector<StepPtr> Command::get_step_nopop(const DriverOrderPtr &order,
                                             uint32_t size) {
  if (order->route->steps.empty() || !order) {
    order->route->current_step.reset();
    return {};
  }
  // auto step = order->route->steps.front();
  // order->route->current_steps.push_back(step);
  auto res = std::vector<StepPtr>();
  const auto len =
      size >= order->route->steps.size() ? order->route->steps.size() : size;
  for (auto i = 0; i < len; i++) {
    res.push_back(order->route->steps.at(i));
    // order->route->current_steps.push_back(order->route->steps.at(i));
    bool single_step = false;
    if (i + 1 < len) {
      for (auto act : order->route->steps.at(i + 1)->path->per_acts.acts) {
        if (act.completion_required) {
          single_step = true;
          break;
        }
      }
    }
    if (single_step) break;
  }
  return res;
}
std::vector<std::shared_ptr<RSSResource>> Command::get_future(
    const DriverOrderPtr &order) const {
  const auto veh = vehicle.lock();
  std::vector<std::shared_ptr<RSSResource>> res;
  if (order->route->steps.size() > veh->send_queue_size) {
    const auto next = *(order->route->steps.begin() + veh->send_queue_size);
    auto path = next->path;
    auto beg = next->path->destination_point.lock();
    auto end = next->path->source_point.lock();
    std::shared_lock<std::shared_mutex> lock(veh->res_mut);
    for (auto &r : veh->allocated_resources) {
      for (auto &x : r) {
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
    lock.unlock();
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

}  // namespace kernel::driver