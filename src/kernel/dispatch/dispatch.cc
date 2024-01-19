#include "../../../include/kernel/dispatch/dispatch.hpp"

#include <utility>

#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel {
namespace dispatch {
std::shared_ptr<data::order::Route> Dispatcher::paths_to_route(
    std::vector<std::shared_ptr<data::model::Point>> ps) {
  std::string route_name = ps.front()->name + "_" + ps.back()->name;
  auto res = std::make_shared<data::order::Route>(route_name);
  int index{0};
  for (auto it = ps.begin(); it != ps.end() - 1; it++) {
    for (auto& x : resource.lock()->paths) {
      if (x->source_point.lock() == *it &&
          x->destination_point.lock() == *(it + 1)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->vehicle_orientation = data::order::Step::Orientation::FORWARD;
        step->path = x;
        res->steps.push_back(step);
        res->costs += x->length;
      } else if (x->source_point.lock() == *(it + 1) &&
                 x->destination_point.lock() == *(it)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->path = x;
        step->vehicle_orientation = data::order::Step::Orientation::BACKWARD;
        res->steps.push_back(step);
        res->costs += x->length;
      }
    }
  }
  if (!res->steps.empty()) {
    res->current_step = res->steps.front();
  }
  return res;
}

std::shared_ptr<data::order::DriverOrder> Dispatcher::route_to_driverorder(
    std::shared_ptr<data::order::Route> route,
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  auto ord =
      std::make_shared<data::order::DriverOrder>("driverorder_" + route->name);
  ord->destination = dest;
  ord->route = route;
  ord->state = data::order::DriverOrder::State::PRISTINE;
  return ord;
}

std::pair<Dispatcher::ResType, std::shared_ptr<TCSResource>> Dispatcher::find(
    const std::string& name) {
  std::shared_ptr<TCSResource> res;
  for (auto& p : resource.lock()->points) {
    if (p->name == name) {
      res = p;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Point,
                                                              res);
    }
  }
  for (auto& l : resource.lock()->locations) {
    if (l->name == name) {
      res = l;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Location,
                                                              res);
    }
  }
  return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Err, res);
}

void Dispatcher::add_task(std::vector<Oper> ops, std::size_t uuid,
                          int strategy) {
  auto time = std::chrono::system_clock::now();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>("transorder_" +
                                                    get_time_fmt(time));
  ord->uuid = uuid;
  ord->create_time = time;
  ord->state = data::order::TransportOrder::State::RAW;
  LOG(INFO) << "new ord " << ord->name << " uuid " << ord->uuid;
  if (ops.empty()) {
    LOG(WARNING) << ord->name << " op is null";
    ord->state = data::order::TransportOrder::State::FAILED;
    orderpool.lock()->ended_orderpool.push_back(ord);
  } else {
    for (auto& op : ops) {
      auto dest = std::get<0>(op);
      auto op_type = std::get<1>(op);
      auto dest_check = find(dest);
      if (dest_check.first == ResType::Err) {
        ord->state = data::order::TransportOrder::State::UNROUTABLE;
        orderpool.lock()->ended_orderpool.push_back(ord);
        return;
      }
      auto destination = res_to_destination(dest_check.second, op_type);
      auto dr =
          std::make_shared<data::order::DriverOrder>("driverorder_" + dest);
      dr->destination = destination;
      dr->transport_order = ord;
      ord->driverorders.push_back(dr);
    }
    if (strategy >= 0) {
      for (auto& v : vehicles) {
        if (v->uuid == uuid) {
          ord->intended_vehicle = v;
        }
      }
    }
    orderpool.lock()->orderpool.push_back(ord);
  }
}

std::shared_ptr<data::order::DriverOrder::Destination>
Dispatcher::res_to_destination(
    const std::shared_ptr<TCSResource>& res,
    data::order::DriverOrder::Destination::OpType op) {
  auto destination = std::make_shared<data::order::DriverOrder::Destination>();
  destination->operation = op;
  destination->destination = res;
  return destination;
}

std::shared_ptr<driver::Vehicle> Dispatcher::select_vehicle(
    std::shared_ptr<data::model::Point> start) {
  // TODO 距离判断
  if (vehicles.empty()) {
    return nullptr;
  }
  std::vector<std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>>>
      idle_temp;
  std::vector<std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>>>
      busy_temp;
  std::vector<std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>>>
      charge_temp;

  for (auto& v : vehicles) {
    Eigen::Vector2i v_pos{0, 0};
    v_pos.x() = v->position.x();
    v_pos.y() = v->position.y();
    // if (v->state == driver::Vehicle::State::IDLE &&
    //     v->proc_state == driver::Vehicle::ProcState::AWAITING_ORDER) {
    if (v->state == driver::Vehicle::State::IDLE) {
      if (!v->paused) {
        idle_temp.push_back(std::pair(v_pos, v));
      }
    } else if (v->state == driver::Vehicle::State::EXECUTING) {
      auto dest = v->orders.back()
                      ->driverorders.back()
                      ->destination->destination.lock();
      auto dest_check = find(dest->name);
      if (dest_check.first == ResType::Point) {
        auto p = std::dynamic_pointer_cast<data::model::Point>(dest);
        v_pos = p->pose;
      } else {
        auto p =
            std::dynamic_pointer_cast<data::model::Location>(dest)->link.lock();
        v_pos = p->pose;
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
            [=](std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>> a,
                std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>>
                    b) {
              auto dis_a = start->pose - a.first;
              auto dis_b = start->pose - b.first;
              return dis_a.norm() < dis_b.norm();
            });
        return charge_temp.front().second;
      }
    } else {
      std::sort(
          charge_temp.begin(), charge_temp.end(),
          [=](std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>> a,
              std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>> b) {
            auto dis_a = start->pose - a.first;
            auto dis_b = start->pose - b.first;
            return dis_a.norm() < dis_b.norm();
          });
      return busy_temp.front().second;
    }
  } else {
    std::sort(
        charge_temp.begin(), charge_temp.end(),
        [=](std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>> a,
            std::pair<Eigen::Vector2i, std::shared_ptr<driver::Vehicle>> b) {
          auto dis_a = start->pose - a.first;
          auto dis_b = start->pose - b.first;
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
  for (auto& f : vs->future_claim_resources) {
    auto f_veh = f->owner.lock();
    if (f_veh) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);
      res.insert(veh);
    }
  }
  return res;
}

void Dispatcher::cancel_all_order() {
  for (auto& x : orderpool.lock()->orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWN;
  }
  for (auto& x : orderpool.lock()->ended_orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWN;
  }
}

void Dispatcher::cancel_order(size_t order_uuid) {
  for (auto& x : orderpool.lock()->orderpool) {
    if (x->uuid == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWN;
    }
  }
  for (auto& x : orderpool.lock()->ended_orderpool) {
    if (x->uuid == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWN;
    }
  }
}

void Dispatcher::cancel_vehicle_all_order(size_t vehicle_uuid) {
  for (auto& v : vehicles) {
    if (v->uuid == vehicle_uuid) {
      for (auto& ord : v->orders) {
        if (ord->state == data::order::TransportOrder::State::RAW ||
            ord->state == data::order::TransportOrder::State::ACTIVE ||
            ord->state == data::order::TransportOrder::State::DISPATCHABLE ||
            ord->state == data::order::TransportOrder::State::BEING_PROCESSED)
          ord->state = data::order::TransportOrder::State::WITHDRAWN;
      }
    }
    v->orders.clear();
  }
}

void Dispatcher::stop() {
  for (auto& v : vehicles) {
    v->close();
  }
  dispose = true;
}

Dispatcher::~Dispatcher() {
  stop();
  if (dispatch_th.joinable()) {
    dispatch_th.join();
  }
  LOG(INFO) << name << " close";
}

void Dispatcher::idle_detect() {
  for (auto& v : vehicles) {
    if (v->state == driver::Vehicle::State::IDLE) {
      auto now = std::chrono::system_clock::now();
      auto dt = now - v->idle_time;
      auto dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt);
      if (dt_s.count() > 10) {
        if (v->current_point != v->init_point) {
          auto op = kernel::dispatch::Oper(
              v->init_point->name,
              data::order::DriverOrder::Destination::OpType::NOP);
          std::vector<kernel::dispatch::Oper> ops;
          ops.push_back(op);
          std::hash<std::string> hash_fn;
          add_task(ops, hash_fn("gohome"), hash_fn(name));
        }
      }
    }
  }
}

void Dispatcher::dispatch_once() {
  if (orderpool.lock()->orderpool.empty()) {
    return;
  } else {
    auto current = orderpool.lock()->orderpool.front();
    orderpool.lock()->ended_orderpool.push_back(current);
    orderpool.lock()->orderpool.pop_front();
    LOG(INFO) << current->name << " status: raw";
    if (current->state == data::order::TransportOrder::State::RAW) {
      // TODO
      if (current->driverorders.empty()) {
        current->state = data::order::TransportOrder::State::FINISHED;
        LOG(INFO) << current->name << " status: finished";
      } else {
        current->state = data::order::TransportOrder::State::ACTIVE;
        LOG(INFO) << current->name << " status: active";
      }
    }
    if (current->state == data::order::TransportOrder::State::ACTIVE) {
      // TODO
      auto v = current->intended_vehicle.lock();
      if (v) {
        // 订单指定了车辆
        current->state = data::order::TransportOrder::State::DISPATCHABLE;
        LOG(INFO) << current->name << " status: dispatchable";

      } else {
        // 自动分配车辆
        auto dest = current->driverorders[current->current_driver_index]
                        ->destination->destination.lock();
        auto dest_check = find(dest->name);
        std::shared_ptr<data::model::Point> start;
        if (dest_check.first == ResType::Point) {
          start =
              std::dynamic_pointer_cast<data::model::Point>(dest_check.second);
        } else if (dest_check.first == ResType::Location) {
          start = std::dynamic_pointer_cast<data::model::Location>(
                      dest_check.second)
                      ->link.lock();
        }
        auto v = select_vehicle(start);
        if (v) {
          current->intended_vehicle = v;
          current->state = data::order::TransportOrder::State::DISPATCHABLE;
          LOG(INFO) << current->name << " status: dispatchable";
        } else {
          current->state = data::order::TransportOrder::State::FAILED;
          LOG(INFO) << current->name << " status: failed";
        }
      }
    }
    if (current->state == data::order::TransportOrder::State::DISPATCHABLE) {
      current->state = data::order::TransportOrder::State::BEING_PROCESSED;
      LOG(INFO) << current->name << " status: being_processed";
      current->intended_vehicle.lock()->receive_task(current);
      current->processing_vehicle = current->intended_vehicle;
      // TODO
    }
    if (current->state == data::order::TransportOrder::State::BEING_PROCESSED) {
      // wait  do nothing
    }
    if (current->state == data::order::TransportOrder::State::WITHDRAWN) {
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
    LOG(INFO) << this->name << " run....";
    while (!dispose) {
      idle_detect();
      dispatch_once();
      auto loop = deadlock_loop();
      if (!loop.empty()) {
        std::stringstream ss;
        ss << "[";
        for (auto& v : loop) {
          ss << v->name << " ,";
        }
        ss << "]";
        LOG_EVERY_N(100, ERROR) << "deadlock --> " << ss.str();
        brake_deadlock(loop);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

}  // namespace dispatch
}  // namespace kernel