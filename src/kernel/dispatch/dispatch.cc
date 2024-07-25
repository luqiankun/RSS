#include "../../../include/kernel/dispatch/dispatch.hpp"

#include <utility>

#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
#include "../../../include/main/tcs.hpp"

namespace kernel {
namespace dispatch {

std::shared_ptr<driver::Vehicle> Dispatcher::select_vehicle(
    std::shared_ptr<data::model::Point> start) {
  if (vehicles.empty()) {
    return nullptr;
  }
  std::vector<std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>>>
      idle_temp;
  std::vector<std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>>>
      busy_temp;
  std::vector<std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>>>
      charge_temp;

  for (auto& v : vehicles) {
    data::Vector3i v_pos{0, 0, 0};
    v_pos = v->position;
    // if (v->state == driver::Vehicle::State::IDLE &&
    //     v->proc_state == driver::Vehicle::ProcState::AWAITING_ORDER) {
    if (v->state == driver::Vehicle::State::IDEL) {
      if (!v->paused) {
        idle_temp.push_back(std::pair(v_pos, v));
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
      busy_temp.push_back(std::pair(v_pos, v));
    } else if (v->state == driver::Vehicle::State::CHARGING) {
      charge_temp.push_back(std::pair(v_pos, v));
    }
  }
  if (idle_temp.empty()) {
    if (busy_temp.empty()) {
      if (charge_temp.empty()) {
        return nullptr;
      } else {
        std::sort(
            charge_temp.begin(), charge_temp.end(),
            [=](std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> a,
                std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> b) {
              auto dis_a = start->position - a.first;
              auto dis_b = start->position - b.first;
              return dis_a.norm() < dis_b.norm();
            });
        return charge_temp.front().second;
      }
    } else {
      std::sort(
          charge_temp.begin(), charge_temp.end(),
          [=](std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> a,
              std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> b) {
            auto dis_a = start->position - a.first;
            auto dis_b = start->position - b.first;
            return dis_a.norm() < dis_b.norm();
          });
      return busy_temp.front().second;
    }
  } else {
    std::sort(
        charge_temp.begin(), charge_temp.end(),
        [=](std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> a,
            std::pair<data::Vector3i, std::shared_ptr<driver::Vehicle>> b) {
          auto dis_a = start->position - a.first;
          auto dis_b = start->position - b.first;
          return dis_a.norm() < dis_b.norm();
        });
    return idle_temp.front().second;
  }
}

std::vector<std::shared_ptr<driver::Vehicle>> Dispatcher::deadlock_loop() {
  std::vector<std::shared_ptr<driver::Vehicle>> res;
  // 死锁检测
  for (auto& v : vehicles) {
    std::stack<std::shared_ptr<driver::Vehicle>> vs;
    std::vector<std::shared_ptr<driver::Vehicle>> exist;
    vs.push(v);
    while (!vs.empty()) {
      auto& x = vs.top();
      vs.pop();
      auto it = std::find(exist.begin(), exist.end(), x);
      if (it != exist.end()) {
        return std::vector<std::shared_ptr<driver::Vehicle>>(it, exist.end());
      }
      exist.push_back(x);

      auto depens = find_owners(x);
      if (!depens.empty()) {
        for (auto& x : depens) {
          vs.push(x);
        }
      } else {
        exist.pop_back();
      }
    }
  }
  return res;
}

std::set<std::shared_ptr<driver::Vehicle>> Dispatcher::find_owners(
    const std::shared_ptr<driver::Vehicle>& vs) {
  std::set<std::shared_ptr<driver::Vehicle>> res;
  for (auto& f : vs->future_allocate_resources) {
    auto f_veh = f->owner.lock();
    if (f_veh) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);
      res.insert(veh);
    }
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
  auto now = std::chrono::system_clock::now();
  for (auto& v : vehicles) {
    if (v->state == driver::Vehicle::State::UNKNOWN) {
      continue;
    }
    if (v->process_chargeing) {
      continue;
    }
    //  充电
    if (v->engerg_level <= v->energy_level_critical) {
      v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      v->cancel_all_order();
      v->state = driver::Vehicle::State::ERROR;
    } else if (v->engerg_level <= v->engrgy_level_recharge) {
      v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      v->cancel_all_order();
      go_charge("TOder_" + std::to_string(now.time_since_epoch().count()) +
                    "_" + v->name + "_goto_charge",
                v);
    } else if (v->engerg_level <= v->energy_level_good) {
      if (v->state == driver::Vehicle::State::IDEL) {
        go_charge("TOder_" + std::to_string(now.time_since_epoch().count()) +
                      "_" + v->name + "_goto_charge",
                  v);
      }
    } else {
      //
      if (v->state == driver::Vehicle::State::IDEL) {
        auto dt = now - v->idle_time;
        auto dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt);
        if (dt_s.count() > 5) {
          if (v->last_point->type != data::model::Point::Type::PARK_POSITION) {
            go_home("TOder_" + std::to_string(now.time_since_epoch().count()) +
                        "_" + v->name + "_goto_park",
                    v);
          }
        }
      }
    }
  }
}

void Dispatcher::dispatch_once() {
  auto current = get_next_ord();
  if (!current) {
    return;
  } else {
    if (current->state == data::order::TransportOrder::State::RAW) {
      CLOG(INFO, dispatch_log) << current->name << " status: [raw]\n";
      if (current->driverorders.empty()) {
        current->state = data::order::TransportOrder::State::FINISHED;
        CLOG(INFO, dispatch_log) << current->name << " status: [finished]\n";
      } else {
        current->state = data::order::TransportOrder::State::ACTIVE;
        CLOG(INFO, dispatch_log) << current->name << " status: [active]\n";
      }
    }
    if (current->state == data::order::TransportOrder::State::ACTIVE) {
      auto v = current->intended_vehicle.lock();
      if (v) {
        // 订单指定了车辆
        current->state = data::order::TransportOrder::State::DISPATCHABLE;
        CLOG(INFO, dispatch_log)
            << current->name << " status: [dispatchable]\n";
        if (v->integration_level !=
            driver::Vehicle::integrationLevel::TO_BE_UTILIZED) {
          current->state = data::order::TransportOrder::State::FAILED;
          CLOG(INFO, dispatch_log) << current->name << " status: [failed]";
        }
      } else {
        if (auto_select) {  // 自动分配车辆
          auto dest = current->driverorders[current->current_driver_index]
                          ->destination->destination.lock();
          auto dest_check = find_res(dest->name);
          std::shared_ptr<data::model::Point> start;
          if (dest_check.first == allocate::ResourceManager::ResType::Point) {
            start = std::dynamic_pointer_cast<data::model::Point>(
                dest_check.second);
          } else if (dest_check.first ==
                     allocate::ResourceManager::ResType::Location) {
            start = std::dynamic_pointer_cast<data::model::Location>(
                        dest_check.second)
                        ->link.lock();
          }
          auto v = select_vehicle(start);
          if (v) {
            current->intended_vehicle = v;
            current->state = data::order::TransportOrder::State::DISPATCHABLE;
            CLOG(INFO, dispatch_log)
                << current->name << " status: [dispatchable]\n";
          } else {
            current->state = data::order::TransportOrder::State::FAILED;
            CLOG(INFO, dispatch_log) << current->name << " status: [failed]\n";
          }
        } else {
          current->state = data::order::TransportOrder::State::FAILED;
          CLOG(INFO, dispatch_log) << current->name << " status: [failed]\n";
        }
      }
    }
    if (current->state == data::order::TransportOrder::State::DISPATCHABLE) {
      current->state = data::order::TransportOrder::State::BEING_PROCESSED;
      CLOG(INFO, dispatch_log)
          << current->name << " status: [being_processed]\n";
      // TODO
      if (current->intended_vehicle.lock()->state ==
          driver::Vehicle::State::UNAVAILABLE) {
        current->state = data::order::TransportOrder::State::FAILED;
        CLOG(INFO, dispatch_log) << current->name << " status: [failed]\n";
      } else {
        current->intended_vehicle.lock()->receive_task(current);
        current->processing_vehicle = current->intended_vehicle;
      }
      // TODO
    }
    if (current->state == data::order::TransportOrder::State::BEING_PROCESSED) {
      // wait  do nothing
    }
    if (current->state == data::order::TransportOrder::State::WITHDRAWL) {
      // TODO
    } else if (current->state == data::order::TransportOrder::State::FINISHED) {
      // TODO
    } else if (current->state == data::order::TransportOrder::State::FAILED) {
      // TODO
    } else if (current->state ==
               data::order::TransportOrder::State::UNROUTABLE) {
      // TODO
    } else {
    }
  }
}
void Dispatcher::brake_deadlock(
    std::vector<std::shared_ptr<driver::Vehicle>> d_loop) {
  // TODO
  if (!d_loop.empty()) {
  }
}

void Dispatcher::run() {
  dispatch_th = std::thread([&] {
    CLOG(INFO, dispatch_log) << this->name << " run....\n";
    int con = 0;
    while (!dispose) {
      std::unique_lock<std::mutex> lock(mut);
      cv.wait_for(lock, std::chrono::seconds(1),
                  [&] { return !order_empty || dispose; });
      idle_detect();
      auto loop = deadlock_loop();
      if (!loop.empty()) {
        std::stringstream ss;
        ss << "[";
        for (auto& v : loop) {
          ss << v->name << " ,";
        }
        ss << "]";
        CLOG_EVERY_N(10, ERROR, dispatch_log) << "deadlock --> " << ss.str();
        brake_deadlock(loop);
      }
      dispatch_once();
    }
  });
}

}  // namespace dispatch
}  // namespace kernel