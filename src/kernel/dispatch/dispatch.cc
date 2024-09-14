#include "../../../include/kernel/dispatch/dispatch.hpp"

#include <utility>

#include "../../../include/3rdparty/uuid/uuid.hpp"
#include "../../../include/component/util/tools.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel::dispatch {
VehPtr Dispatcher::select_vehicle(const allocate::PointPtr &start) {
  if (vehicles.empty()) {
    return nullptr;
  }
  std::vector<std::pair<Eigen::Vector3i, VehPtr>> idle_temp;
  std::vector<std::pair<Eigen::Vector3i, VehPtr>> busy_temp;
  std::vector<std::pair<Eigen::Vector3i, VehPtr>> charge_temp;

  for (auto &v : vehicles) {
    Eigen::Vector3i v_pos{0, 0, 0};
    v_pos = v->position;
    // if (v->state == driver::Vehicle::State::IDLE &&
    //     v->proc_state == driver::Vehicle::ProcState::AWAITING_ORDER) {
    if (v->state == driver::Vehicle::State::IDEL) {
      if (!v->paused) {
        idle_temp.emplace_back(v_pos, v);
      }
    } else if (v->state == driver::Vehicle::State::EXECUTING) {
      auto dest = v->orders.back()
                      ->driverorders.back()
                      ->destination->destination.lock();
      auto dest_check = find_res(dest->name);
      if (dest_check.first == allocate::ResourceManager::ResType::Point) {
        auto p = std::dynamic_pointer_cast<data::model::Point>(dest);
        v_pos = p->position;
      } else {
        auto p =
            std::dynamic_pointer_cast<data::model::Location>(dest)->link.lock();
        v_pos = p->position;
      }
      busy_temp.emplace_back(v_pos, v);
    } else if (v->state == driver::Vehicle::State::CHARGING) {
      charge_temp.emplace_back(v_pos, v);
    }
  }
  if (idle_temp.empty()) {
    if (busy_temp.empty()) {
      if (charge_temp.empty()) {
        return nullptr;
      } else {
        std::sort(charge_temp.begin(), charge_temp.end(),
                  [=](const std::pair<Eigen::Vector3i, VehPtr> &a,
                      const std::pair<Eigen::Vector3i, VehPtr> &b) {
                    auto dis_a = start->position - a.first;
                    auto dis_b = start->position - b.first;
                    return dis_a.norm() < dis_b.norm();
                  });
        return charge_temp.front().second;
      }
    } else {
      std::sort(charge_temp.begin(), charge_temp.end(),
                [=](const std::pair<Eigen::Vector3i, VehPtr> &a,
                    const std::pair<Eigen::Vector3i, VehPtr> &b) {
                  auto dis_a = start->position - a.first;
                  auto dis_b = start->position - b.first;
                  return dis_a.norm() < dis_b.norm();
                });
      return busy_temp.front().second;
    }
  } else {
    std::sort(charge_temp.begin(), charge_temp.end(),
              [=](const std::pair<Eigen::Vector3i, VehPtr> &a,
                  const std::pair<Eigen::Vector3i, VehPtr> &b) {
                auto dis_a = start->position - a.first;
                auto dis_b = start->position - b.first;
                return dis_a.norm() < dis_b.norm();
              });
    return idle_temp.front().second;
  }
}

std::vector<VehPtr> Dispatcher::deadlock_loop() {
  std::vector<VehPtr> res;
  // 死锁检测
  for (auto &v : vehicles) {
    std::stack<VehPtr> vs;
    std::vector<VehPtr> exist;
    vs.push(v);
    while (!vs.empty()) {
      auto &x = vs.top();
      vs.pop();
      if (auto it = std::find(exist.begin(), exist.end(), x);it != exist.end()) {
        return {it, exist.end()};
      }
      exist.push_back(x);

      auto depens = find_depends(x);
      if (!depens.empty()) {
        for (auto &x_dep : depens) {
          vs.push(x_dep);
        }
      } else {
        exist.pop_back();
      }
    }
  }
  return res;
}

std::set<VehPtr> Dispatcher::find_depends(const VehPtr &vs) {
  std::set<VehPtr> res;
  for (auto &f : vs->future_allocate_resources) {
    if (auto f_veh = f->owner.lock()) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);
      res.insert(veh);
    }
  }
  return res;
}

std::vector<VehPtr> Dispatcher::block_loop() {
  std::vector<VehPtr> res;
  // 阻挡检测
  for (auto &v : vehicles) {
    res.push_back(v);
    for (auto &x : v->future_allocate_resources) {
      if (auto f_veh = x->owner.lock()) {
        if (const auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);veh->state != driver::Vehicle::State::EXECUTING) {
          res.push_back(veh);
          return res;
        }
      }
    }
    res.pop_back();
  }
  return res;
}

void Dispatcher::stop() { dispose = true; }

Dispatcher::~Dispatcher() {
  stop();
  if (dispatch_th.joinable()) {
    dispatch_th.join();
  }
  CLOG(INFO, dispatch_log) << name << " close\n";
}

void Dispatcher::idle_detect() {
  auto now = get_now_utc_time();
  for (auto &v : vehicles) {
    if (v->state == driver::Vehicle::State::UNKNOWN) {
      continue;
    }
    if (v->process_charging) {
      continue;
    }
    //  充电
    if (v->energy_level <= v->energy_level_critical) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      v->cancel_all_order();
      v->state = driver::Vehicle::State::ERROR;
    } else if (v->energy_level <= v->energy_level_recharge) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      v->cancel_all_order();
      go_charge("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                    "_goto_charge",
                v);
    } else if (v->energy_level <= v->energy_level_good) {
      if (v->state == driver::Vehicle::State::IDEL) {
        go_charge("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                      "_goto_charge",
                  v);
      }
    } else {
      //
      if (v->state == driver::Vehicle::State::IDEL) {
        auto dt = now - v->idle_time;
        auto dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt);
        if (dt_s.count() > 15) {
          if (v->park_point && v->park_point != v->last_point) {
            go_home("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                        "_goto_park",
                    v);
          } else if (v->last_point->type !=
                     data::model::Point::Type::PARK_POSITION) {
            if (auto x = get_park_point(v->last_point)) {
              go_home("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                          "_goto_park",
                      v);
            }
          }
        }
      }
    }
  }
}

void Dispatcher::dispatch_once() {
  for (auto &v : vehicles) {
    if (v && v->current_order) {
      if (v->current_order->state ==
          data::order::TransportOrder::State::WITHDRAWL) {
        v->command_done();
      }
    }
  }
  auto current_vec = get_next_vec();
  if (current_vec.second.empty()) {
    return;
  }
  auto current = current_vec.second.front();
  if (current->state == data::order::TransportOrder::State::RAW) {
    // 重写
    if (current->driverorders.empty()) {
      return;
    }
    auto v = current->intended_vehicle.lock();
    if (!v) {
      if (auto_select) {
        auto dest = current->driverorders[current->current_driver_index]
                        ->destination->destination.lock();
        auto dest_check = find_res(dest->name);
        allocate::PointPtr start;
        if (dest_check.first == allocate::ResourceManager::ResType::Point) {
          start =
              std::dynamic_pointer_cast<data::model::Point>(dest_check.second);
        } else if (dest_check.first ==
                   allocate::ResourceManager::ResType::Location) {
          start = std::dynamic_pointer_cast<data::model::Location>(
                      dest_check.second)
                      ->link.lock();
        }
        auto v_select = select_vehicle(start);
        if (!v_select) {
          CLOG(WARNING, dispatch_log)
              << current->name << " not exist idle vehicle \n";
          return;
        } else {
          current->intended_vehicle = v_select;
        }
      }
    }
    if (!current->intended_vehicle.lock()) {
      CLOG(WARNING, dispatch_log)
          << current->name << " not exist intended_vehicle \n";
      return;
    }
    current->state = data::order::TransportOrder::State::ACTIVE;
    CLOG(INFO, dispatch_log) << current->name << " status: [active]\n";
  }
  if (current->state == data::order::TransportOrder::State::ACTIVE) {
    for (auto &x : current->dependencies) {
      if (const auto dep = x.lock()) {
        if (dep->state != data::order::TransportOrder::State::FINISHED) {
          return;
        }
      }
    }
    current->state = data::order::TransportOrder::State::DISPATCHABLE;
    CLOG(INFO, dispatch_log) << current->name << " status: [dispatchable]\n";
  }
  if (current->state == data::order::TransportOrder::State::DISPATCHABLE) {
    auto su = current->intended_vehicle.lock()->state;
    if (su == driver::Vehicle::State::IDEL ||
        su == driver::Vehicle::State::CHARGING) {
      current->intended_vehicle.lock()->receive_task(current);
      current->processing_vehicle = current->intended_vehicle;
      pop_order(current);
    } else if (su == driver::Vehicle::State::EXECUTING) {
      if (current->intended_vehicle.lock()->current_order->state ==
          data::order::TransportOrder::State::WITHDRAWL) {
        current->intended_vehicle.lock()->receive_task(current);
        current->processing_vehicle = current->intended_vehicle;
        pop_order(current);
      } else {
      }
    } else if (su == driver::Vehicle::State::UNAVAILABLE) {
      pop_order(current);
    } else if (su == driver::Vehicle::State::ERROR) {
      pop_order(current);
    } else {
      pop_order(current);
    }
    // }
  }
  if (current->state == data::order::TransportOrder::State::BEING_PROCESSED) {
    pop_order(current);
    CLOG(INFO, dispatch_log)
        << current->name << " status: [beging_processed]\n";

    // wait  do nothing
  }
  if (current->state == data::order::TransportOrder::State::WITHDRAWL) {
    pop_order(current);
    CLOG(WARNING, dispatch_log) << current->name << " status: [withdrawl]\n";
  }
  if (current->state == data::order::TransportOrder::State::FINISHED) {
    pop_order(current);
    CLOG(WARNING, dispatch_log) << current->name << " status: [finished]\n";
  }
  if (current->state == data::order::TransportOrder::State::FAILED) {
    pop_order(current);
    CLOG(WARNING, dispatch_log) << current->name << " status: [failed]\n";
  }
  if (current->state == data::order::TransportOrder::State::UNROUTABLE) {
    pop_order(current);
    CLOG(WARNING, dispatch_log) << current->name << " status: [unroutable]\n";
  }
}
void Dispatcher::brake_deadlock(const std::vector<VehPtr> &d_loop) {
  // TODO
  if (!d_loop.empty()) {
  }
}
void Dispatcher::brake_blocklock(const std::vector<VehPtr> &d_loop) {
  // TODO
  if (!d_loop.empty()) {
  }
}
void Dispatcher::run() {
  dispatch_th = std::thread([&] {
    CLOG(INFO, dispatch_log) << this->name << " run....\n";
    // int con = 0;
    while (!dispose) {
      std::unique_lock<std::mutex> lock(mut);
      cv.wait_for(lock, std::chrono::seconds(1),
                  [&] { return !order_empty || dispose; });
      idle_detect();
      auto deadloop = deadlock_loop();
      if (!deadloop.empty()) {
        std::stringstream ss;
        ss << "[";
        for (auto &v : deadloop) {
          if (v == deadloop.back()) {
            ss << v->name;
            break;
          }
          ss << v->name << " ,";
        }
        ss << "]\n";
        CLOG_EVERY_N(2, ERROR, dispatch_log) << "deadlock --> " << ss.str();
        brake_deadlock(deadloop);
      }
      auto blockloop = block_loop();
      if (!blockloop.empty()) {
        assert(blockloop.size() == 2);
        CLOG_EVERY_N(2, ERROR, dispatch_log)
            << "blocklock --> " << blockloop.front()->name << " blocked by "
            << blockloop.back()->name << "\n";
        brake_blocklock(blockloop);
      }
      dispatch_once();
    }
  });
}

}